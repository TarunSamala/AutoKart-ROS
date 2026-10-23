"""Cone-detection backends for autokart_perception_adapter.

Two backends:

- ``SyntheticConeSource``: zero external dependency, deterministic fixed cone
  layout. Makes this package buildable/testable/launchable with nothing else
  installed (same role as autokart_hp60c's SyntheticBackend).
- ``AlgorithmBackend``: a thin adapter around the existing, already-tested
  perception algorithm package (YOLO26s ONNX detection + RGB/depth fusion +
  the full safety/qualification gating), imported lazily from a path given at
  runtime via a ROS parameter -- never hardcoded, never reimplemented here.

Neither backend talks to a camera. Both take already-captured RGB/depth/
intrinsics (e.g. from autokart_hp60c's topics, or from a replay/synthetic
frame produced by the algorithm backend itself) and return qualified cones.
"""
from __future__ import annotations

import importlib
import json
import hashlib
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Protocol


class PerceptionBackendError(RuntimeError):
    pass


@dataclass(frozen=True)
class ConeObservation:
    id: int
    label: str          # "blue" | "yellow" | "orange" | "large_orange" | anything else -> dropped
    confidence: float
    x_m: float
    y_m: float
    sigma_m: float


@dataclass
class PerceptionOutcome:
    valid: bool
    reasons: tuple[str, ...]
    cones: tuple[ConeObservation, ...]
    source: str                      # "LIVE" | "REPLAY" | "SYNTHETIC"
    model_sha256: str = "NONE"
    processing_ms: float = 0.0
    detections: tuple[dict, ...] = ()
    diagnostics: dict = field(default_factory=dict)
    obstacle_coverage_valid: bool = False
    obstacles: tuple[dict, ...] = ()


class PerceptionBackend(Protocol):
    def process(self, rgb, depth, fx: float, fy: float, cx: float, cy: float,
                timestamp_ns: int, source_hint: str) -> PerceptionOutcome: ...


class SyntheticConeSource:
    """Ignores the image entirely and returns a fixed, deterministic cone
    layout, always qualified. Exists purely for portability/testing, exactly
    like autokart_hp60c's SyntheticBackend never claims LIVE provenance."""

    _LAYOUT = (
        (0, "blue", 2.0, 1.5), (1, "yellow", 2.0, -1.5),
        (2, "blue", 5.0, 1.5), (3, "yellow", 5.0, -1.5),
    )

    def process(self, rgb, depth, fx, fy, cx, cy, timestamp_ns, source_hint) -> PerceptionOutcome:
        import math
        cones = tuple(
            ConeObservation(id=i, label=label, confidence=0.95, x_m=x, y_m=y,
                             sigma_m=0.05)
            for i, label, x, y in self._LAYOUT
        )
        return PerceptionOutcome(valid=True, reasons=(), cones=cones, source="SYNTHETIC", obstacle_coverage_valid=True)


class AlgorithmBackend:
    """Reuses autokart_perception.{pipeline,application,detector} exactly as
    the existing, already-tested ROS wrapper in kart/ros2_ws does: a
    CoreRuntime + Pipeline pair, fed one frame at a time via
    ``sdk.process_frame``. Dependency path resolved at runtime via a
    parameter -- see DEPENDENCIES.md."""

    def __init__(self, algorithm_package_path: str, model_path: Optional[str],
                 model_manifest: Optional[str], calibration_artifact: Optional[str], obstacle_config_path: Optional[str] = None):
        if not algorithm_package_path.strip():
            raise PerceptionBackendError("ALGORITHM_PACKAGE_PATH_NOT_CONFIGURED")
        src = Path(algorithm_package_path).expanduser()
        if not src.is_dir():
            raise PerceptionBackendError(f"DEPENDENCY_PATH_NOT_FOUND:{src}")
        src_str = str(src)
        sys.path[:] = [x for x in sys.path if x != src_str]
        sys.path.insert(0, src_str)
        try:
            application = importlib.import_module("autokart_perception.application")
            detector_module = importlib.import_module("autokart_perception.detector")
            pipeline_module = importlib.import_module("autokart_perception.pipeline")
            types_module = importlib.import_module("autokart_perception.types")
            for module in (application, detector_module, pipeline_module, types_module):
                if not Path(module.__file__).resolve().is_relative_to(src.resolve()):
                    raise ValueError("DEPENDENCY_ORIGIN_MISMATCH")
        except Exception as exc:
            raise PerceptionBackendError(f"SDK_ADAPTER_IMPORT_FAILED:{type(exc).__name__}:{exc}") from exc

        self._Frame = types_module.Frame
        self._Intrinsics = types_module.Intrinsics
        self._core = application.CoreRuntime()
        detector = None
        if model_path and model_manifest:
            try:
                detector = detector_module.Yolo26OnnxDetector(
                    Path(model_path), manifest_path=Path(model_manifest))
            except Exception as exc:
                raise PerceptionBackendError(f"MODEL_LOAD_FAILED:{type(exc).__name__}:{exc}") from exc
        config = dict(self._core.config)
        if calibration_artifact:
            config["calibration_artifact"] = calibration_artifact
        self._synthetic = detector is None
        self.depth_scale, self.aligned = 0.001, True
        self._pipeline = pipeline_module.Pipeline(detector, config, synthetic=detector is None)
        self._model_sha256 = getattr(detector, "model_sha256", "NONE")
        self.obstacle_config = None
        self.obstacle_calibration_verified = False
        self.obstacle_alignment_verified = False
        if obstacle_config_path:
            try:
                from .depth_obstacles import DepthObstacleConfig
                record = json.loads(Path(obstacle_config_path).read_text())
                self.obstacle_config = DepthObstacleConfig(**record["configuration"])
                calibration_hash = hashlib.sha256(Path(calibration_artifact).read_bytes()).hexdigest() if calibration_artifact else None
                self.obstacle_calibration_verified = (record.get("calibration_verified") is True and
                    self.obstacle_config.calibration_hash == calibration_hash)
                self.obstacle_alignment_verified = record.get("depth_alignment_verified") is True
            except (ValueError,TypeError,KeyError,OSError) as exc:
                raise PerceptionBackendError(f"OBSTACLE_CONFIG_INVALID:{exc}") from exc

    def process(self, rgb, depth, fx, fy, cx, cy, timestamp_ns, source_hint) -> PerceptionOutcome:
        source_hint = {"LIVE_PHYSICAL":"LIVE", "SIMULATED":"SYNTHETIC"}.get(source_hint, source_hint)
        if source_hint == "LIVE" and self._synthetic:
            raise PerceptionBackendError("LIVE_MODEL_NOT_CONFIGURED")
        source_timestamp_ns = getattr(self, "source_timestamp_ns", timestamp_ns) if source_hint == "LIVE" else timestamp_ns
        frame = self._Frame(
            rgb=rgb, depth=depth, timestamp_ns=source_timestamp_ns,
            rgb_timestamp=timestamp_ns // 1_000_000, depth_timestamp=timestamp_ns // 1_000_000,
            rgb_sequence=getattr(self, "capture_sequence", 0), depth_sequence=getattr(self, "capture_sequence", 0),
            intrinsics=self._Intrinsics(fx, fy, cx, cy), depth_scale=self.depth_scale,
            aligned=self.aligned, source="SYNTHETIC" if self._synthetic else source_hint,
        )
        t0 = time.monotonic()
        result = self._core.sdk.process_frame(frame, pipeline=self._pipeline)
        processing_ms = (time.monotonic() - t0) * 1000.0
        status = result.perception_result
        reasons = tuple(status.safety.reasons) if status.safety else ()
        # Mirrors the LIVE_CONE_BLOCKERS/qualified() gating already proven in
        # kart/ros2_ws/.../autokart_perception_ros/node.py: LIVE frames
        # additionally require a verified mount/calibration before cones ever
        # count as metric-qualified, not just "detector ran without error."
        blocking = list(reasons)
        if source_hint == "LIVE" and status.diagnostics.get("calibration_status") != "MOUNT_VERIFIED":
            blocking.insert(0, "LIVE_METRIC_CALIBRATION_UNVERIFIED")
        valid = not blocking
        cones = tuple(
            ConeObservation(id=c.id, label=c.label, confidence=c.confidence,
                             x_m=c.xy[0], y_m=c.xy[1], sigma_m=c.sigma_m)
            for c in status.cones
        ) if valid else ()
        obstacle_result = {"valid":False,"coverage_complete":False,"obstacles":[],
                           "reason_codes":["OBSTACLE_CONFIGURATION_UNAVAILABLE"]}
        if self.obstacle_config is not None:
            from .depth_obstacles import extract_depth_obstacles
            matched = (abs(fx-self.obstacle_config.fx)<1e-6 and abs(fy-self.obstacle_config.fy)<1e-6 and
                       abs(cx-self.obstacle_config.cx)<1e-6 and abs(cy-self.obstacle_config.cy)<1e-6 and
                       abs(self.depth_scale-self.obstacle_config.depth_scale_m)<1e-12)
            obstacle_result = extract_depth_obstacles(depth,self.obstacle_config,
                calibration_verified=self.obstacle_calibration_verified and matched and valid,
                depth_alignment_verified=self.obstacle_alignment_verified and self.aligned)
        return PerceptionOutcome(valid=valid, reasons=tuple(blocking), cones=cones,
                                  source="SYNTHETIC" if self._synthetic else source_hint, model_sha256=self._model_sha256,
                                  detections=tuple(dict(d.__dict__) for d in status.detections),
                                  diagnostics={**dict(status.diagnostics),"obstacle_processing":obstacle_result},
                                  obstacles=tuple(obstacle_result["obstacles"]),
                                  obstacle_coverage_valid=valid and obstacle_result["valid"] and obstacle_result["coverage_complete"],
                                  processing_ms=processing_ms)


def build_backend(name: str, **kwargs) -> PerceptionBackend:
    if name == "synthetic":
        return SyntheticConeSource()
    if name == "algorithm":
        return AlgorithmBackend(
            algorithm_package_path=kwargs["algorithm_package_path"],
            model_path=kwargs.get("model_path") or None,
            model_manifest=kwargs.get("model_manifest") or None,
            calibration_artifact=kwargs.get("calibration_artifact") or None,
            obstacle_config_path=kwargs.get("obstacle_config_path") or None,
        )
    raise PerceptionBackendError(f"UNKNOWN_BACKEND:{name}")
