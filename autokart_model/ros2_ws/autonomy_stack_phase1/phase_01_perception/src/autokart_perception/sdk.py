"""Stable, nonauthorizing public API over the existing perception and geometry.

The base module imports no camera, OpenCV, ROS, HTTP, LAB, or simulator package.
Optional perception imports occur only in :meth:`PerceptionGeometrySDK.process_frame`.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from importlib import resources
import logging
from math import isfinite
from pathlib import Path
from time import monotonic_ns, perf_counter
from typing import Any, Iterable, Mapping, Sequence

import yaml

from .shared_core import AlgorithmCore, BrakeCapability, PoseEstimate
from .types import Boundaries, Cone, Frame, Gate, PathSample, Result


LOGGER = logging.getLogger(__name__)
SOURCES = frozenset({"LIVE", "REPLAY", "SIMULATED", "SYNTHETIC"})


class SDKError(Exception):
    """An API input, dependency, or configuration failure with a stable code."""

    def __init__(self, code: str, detail: str):
        super().__init__(f"{code}: {detail}")
        self.code = code
        self.detail = detail


class ConfigurationError(SDKError):
    pass


class InputValidationError(SDKError):
    pass


class DependencyError(SDKError):
    pass


@dataclass(frozen=True)
class GeometrySnapshot:
    graph_status: str
    delaunay_edges: tuple[tuple[int, int], ...]
    delaunay_triangles: tuple[tuple[int, int, int], ...]
    gabriel_edges: tuple[tuple[int, int], ...]
    boundaries: Boundaries
    gates: tuple[Gate, ...]
    rejected_candidates: tuple[Mapping[str, Any], ...]
    centerline_m: tuple[tuple[float, float], ...]
    path_profile: tuple[PathSample, ...]
    topology_reasons: tuple[str, ...]
    skidpad_topology: Mapping[str, Any] | None


@dataclass(frozen=True)
class PlanningSnapshot:
    valid: bool
    path_frame: str
    local_path_m: tuple[tuple[float, float], ...]
    stopping_distance_m: float | None
    speed_profile_provenance: str


@dataclass(frozen=True)
class ControlCandidate:
    valid: bool
    target_speed_mps: float
    steering_angle_rad: float
    expiry_ns: int
    published_to_vcu: bool
    target_point_base_m: tuple[float, float] | None = None


@dataclass(frozen=True)
class SDKResult:
    source: str
    algorithm_version: str
    configuration_sha256: str
    timestamp_ns: int
    coordinate_frames: Mapping[str, str]
    observations: tuple[Cone, ...]
    tracked_cones: tuple[Mapping[str, Any], ...]
    geometry: GeometrySnapshot
    planning: PlanningSnapshot
    control_candidate: ControlCandidate
    mission_state: Mapping[str, Any]
    reasons: tuple[str, ...]
    status: str
    processing_ms: float
    vehicle_authorized: bool
    perception: Mapping[str, Any] | None
    perception_result: Result | None
    _payload: dict

    def to_dict(self) -> dict:
        """Return the JSON-safe diagnostic contract used by CORE and LAB."""
        return deepcopy(self._payload)


def _result(raw: dict, elapsed_ms: float, perception_result: Result | None = None) -> SDKResult:
    geometry = raw["geometry"]
    planning = raw["planning"]
    control = raw["control_candidate"]
    reasons = tuple(raw["reasons"])
    perception = raw.get("perception")
    if "STALE_FRAME" in reasons or (perception and perception.get("safety_state") == "STALE"):
        status = "STALE"
    elif perception and perception.get("safety_state") == "SENSOR_FAULT":
        status = "SENSOR_FAULT"
    elif "INVALID_GEOMETRY" in reasons:
        status = "INVALID_GEOMETRY"
    elif "NO_VALID_PATH" in reasons or "LOOP_REFERENCE_UNAVAILABLE" in reasons:
        status = "AMBIGUOUS"
    elif reasons:
        status = "DEGRADED"
    else:
        status = "VALID"
    raw["sdk_status"] = status
    raw["processing_ms"] = elapsed_ms
    return SDKResult(
        source=raw["source"], algorithm_version=raw["algorithm_version"],
        configuration_sha256=raw["configuration_sha256"],
        timestamp_ns=raw["timestamp_ns"], coordinate_frames=raw["frames"],
        observations=tuple(Cone(**item) for item in raw["observations"]),
        tracked_cones=tuple(raw["tracked_cones"]),
        geometry=GeometrySnapshot(
            graph_status=geometry["status"],
            delaunay_edges=tuple(tuple(edge) for edge in geometry["delaunay_edges"]),
            delaunay_triangles=tuple(tuple(triangle) for triangle in geometry["delaunay_triangles"]),
            gabriel_edges=tuple(tuple(edge) for edge in geometry["gabriel_edges"]),
            boundaries=Boundaries(**geometry["boundaries"]),
            gates=tuple(Gate(**item) for item in geometry["gates"]),
            rejected_candidates=tuple(geometry["rejected_candidates"]),
            centerline_m=tuple(tuple(point) for point in geometry["centerline"]),
            path_profile=tuple(PathSample(**item) for item in geometry["path_profile"]),
            topology_reasons=tuple(geometry["topology_reasons"]),
            skidpad_topology=geometry["skidpad_topology"]),
        planning=PlanningSnapshot(planning["path_valid"], planning["path_frame"],
                                  tuple(tuple(point) for point in planning["local_path"]),
                                  planning["stopping_distance_m"], planning["speed_profile_provenance"]),
        control_candidate=ControlCandidate(**control), mission_state=raw["mission_state"],
        reasons=reasons, status=status, processing_ms=elapsed_ms,
        vehicle_authorized=False, perception=perception,
        perception_result=perception_result, _payload=raw)


def default_config() -> dict:
    """Load the wheel's pinned default configuration; calibration is unverified."""
    text = resources.files("autokart_perception").joinpath("data/perception.yaml").read_text()
    return yaml.safe_load(text)


def validate_config(config: Mapping[str, Any]) -> dict:
    if not isinstance(config, Mapping):
        raise ConfigurationError("CONFIG_NOT_MAPPING", "configuration must be a mapping")
    validated = dict(config)
    required_positive = ("min_track_width_m", "max_track_width_m", "min_gate_quality",
                         "min_centerline_gates", "max_lateral_jump_m",
                         "max_heading_change_deg", "max_width_change_m")
    for key in required_positive:
        value = validated.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value) or value <= 0:
            raise ConfigurationError("CONFIG_INVALID", f"{key} must be positive and finite")
    if validated["min_track_width_m"] >= validated["max_track_width_m"]:
        raise ConfigurationError("CONFIG_INVALID", "minimum track width must be less than maximum")
    if type(validated["min_centerline_gates"]) is not int or validated["min_centerline_gates"] < 2:
        raise ConfigurationError("CONFIG_INVALID", "min_centerline_gates must be an integer >= 2")
    return validated


class PerceptionGeometrySDK:
    """Stateful SDK session. One instance retains a bounded local cone map."""

    def __init__(self, config: Mapping[str, Any] | None = None, *, detector: Any = None,
                 wheelbase_m: float = 1.15):
        self.config = validate_config(default_config() if config is None else config)
        if not isfinite(wheelbase_m) or wheelbase_m <= 0:
            raise ConfigurationError("CONFIG_INVALID", "wheelbase_m must be positive and finite")
        self.algorithms = AlgorithmCore(self.config, wheelbase_m=wheelbase_m)
        self.detector = detector
        self._pipeline = None

    @classmethod
    def from_yaml(cls, path: str | Path, *, detector: Any = None,
                  wheelbase_m: float = 1.15) -> "PerceptionGeometrySDK":
        try:
            config = yaml.safe_load(Path(path).read_text())
        except (OSError, yaml.YAMLError) as exc:
            raise ConfigurationError("CONFIG_READ_FAILED", str(exc)) from exc
        return cls(config, detector=detector, wheelbase_m=wheelbase_m)

    def process_cones(self, cones: Sequence[Cone], pose: PoseEstimate | None = None,
                      *, timestamp_ns: int | None = None, source: str = "SIMULATED",
                      mission: str = "STRAIGHT_TRACK", obstacles: Iterable[dict] = (),
                      brake_capability: BrakeCapability | None = None,
                      stop_distance_m: float | None = None) -> SDKResult:
        if source not in SOURCES:
            raise InputValidationError("INVALID_SOURCE", source)
        if not isinstance(cones, Sequence) or any(not isinstance(item, Cone) for item in cones):
            raise InputValidationError("INVALID_CONE_TYPE", "cones must be a sequence of Cone")
        if timestamp_ns is None:
            if source == "LIVE":
                raise InputValidationError("LIVE_TIMESTAMP_REQUIRED", "live observations need an explicit timestamp")
            timestamp_ns = max((item.observed_ns for item in cones if item.observed_ns > 0),
                               default=monotonic_ns())
        if not isinstance(timestamp_ns, int) or timestamp_ns < 0:
            raise InputValidationError("INVALID_TIMESTAMP", "timestamp_ns must be nonnegative int")
        start = perf_counter()
        raw = self.algorithms.evaluate(cones, pose, mission, now_ns=timestamp_ns,
                                       source=source, obstacles=obstacles,
                                       brake_capability=brake_capability,
                                       stop_distance_m=stop_distance_m)
        output = _result(raw, (perf_counter() - start) * 1000)
        if output.status != "VALID":
            LOGGER.warning("geometry status=%s reasons=%s", output.status, output.reasons)
        return output

    def process_frame(self, frame: Frame, pose: PoseEstimate | None = None,
                      *, mission: str = "STRAIGHT_TRACK", pipeline: Any = None) -> SDKResult:
        """Run existing RGB-D/recording pipeline, then the same shared geometry."""
        if not isinstance(frame, Frame):
            raise InputValidationError("INVALID_FRAME_TYPE", "frame must be Frame")
        try:
            from .pipeline import Pipeline
        except ImportError as exc:
            raise DependencyError("PERCEPTION_EXTRA_MISSING", "install autokart-perception[perception]") from exc
        if pipeline is None:
            if self._pipeline is None:
                self._pipeline = Pipeline(self.detector, self.config, synthetic=False)
            pipeline = self._pipeline
        start = perf_counter()
        perception = pipeline.process(frame)
        # Live coordinates are diagnostic until RAW16 scale and the complete
        # camera-to-base_link transform pass an independent survey. Do not let
        # candidate coordinates feed the shared graph or planner.
        qualified_cones = perception.cones if frame.source != "LIVE" else ()
        if frame.source == "REPLAY" and frame.calibration.get("recorded_source") not in {"SYNTHETIC", "SIMULATED"}:
            qualified_cones = ()
        raw = self.algorithms.evaluate(qualified_cones, pose, mission,
                                       now_ns=frame.timestamp_ns, source=frame.source)
        synthetic_diagnostic = (frame.source == "SYNTHETIC" and pipeline.synthetic and
                                perception.status == "SYNTHETIC_GEOMETRY" and
                                set(perception.safety.reasons) <= {
                                    "NON_LIVE_SOURCE", "MODEL_UNVERIFIED", "CALIBRATION_UNVERIFIED"})
        if not synthetic_diagnostic and (
            perception.status not in {"OK", "RECORDED_OBSERVATIONS"} or
            perception.safety.state != "VALID"
        ):
            raw["reasons"] = sorted(set(raw["reasons"] + ["PERCEPTION_INVALID"]))
            raw["planning"]["path_valid"] = False
            raw["planning"]["local_path"] = []
            raw["control_candidate"].update(valid=False, target_speed_mps=0.0,
                                             steering_angle_rad=0.0,
                                             expiry_ns=frame.timestamp_ns)
        raw["perception"] = {
            "status": perception.status, "safety_state": perception.safety.state,
            "safety_reasons": list(perception.safety.reasons),
            "cone_count": len(perception.cones),
            "detections": [item.__dict__.copy() for item in perception.detections],
            "processing_ms": perception.diagnostics.get("processing_ms"),
            "frame_drops": frame.drops, "frame_sequence": frame.rgb_sequence,
            "provenance": "INJECTED_SYNTHETIC_CONES" if synthetic_diagnostic else frame.source,
            "diagnostics": {
                key: perception.diagnostics.get(key)
                for key in ("valid_depth_pct", "inference_ms", "depth_localization_ms",
                            "delaunay_ms", "gabriel_ms", "gate_ms",
                            "centerline_curvature_ms", "rgb_fps", "depth_fps",
                            "end_to_end_fps", "frame_age_ms", "cone_details",
                            "rgb_depth_registration", "calibration_status",
                            "frame_frozen", "frame_out_of_order")
            },
        }
        output = _result(raw, (perf_counter() - start) * 1000, perception)
        if output.status != "VALID":
            LOGGER.warning("frame status=%s reasons=%s", output.status, output.reasons)
        return output


def process_cones(cones: Sequence[Cone], pose: PoseEstimate | None = None, **kwargs: Any) -> SDKResult:
    """Convenience call for one independent observation set; use a session for tracking."""
    return PerceptionGeometrySDK().process_cones(cones, pose, **kwargs)


def process_frame(frame: Frame, pose: PoseEstimate | None = None, **kwargs: Any) -> SDKResult:
    """Convenience call for one frame; use a session for continuous camera input."""
    return PerceptionGeometrySDK().process_frame(frame, pose, **kwargs)
