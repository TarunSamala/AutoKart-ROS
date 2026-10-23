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
        return PerceptionOutcome(valid=True, reasons=(), cones=cones, source="SYNTHETIC")


# Port of qualified()/LIVE_CONE_BLOCKERS in
# kart/ros2_ws/src/autokart_nodes/autokart_perception/autokart_perception_ros/node.py.
# Only perception-integrity reasons block a LIVE frame's cones; downstream
# reasons such as INVALID_POSE / NO_VALID_PATH must not, because this node has
# no pose input and would otherwise never qualify a LIVE cone.
LIVE_CONE_BLOCKERS = frozenset({
    "INVALID_SOURCE", "INFERENCE_FAILURE", "MODEL_UNVERIFIED", "MODEL_NOT_AVAILABLE",
    "METRIC_LOCALIZATION_FAILED", "TIMESTAMP_MISMATCH", "DEPTH_UNAVAILABLE", "FRAME_FROZEN",
    "FRAME_OUT_OF_ORDER", "INFERENCE_OVERLOAD", "PROCESSING_OVERLOAD", "FRAME_TOO_OLD",
    "INVALID_FRAME_FORMAT", "DEPTH_ALIGNMENT_UNAVAILABLE",
})


def qualify(perception, source: str, calibration: dict) -> tuple[bool, tuple[str, ...]]:
    if perception is None:
        return False, ("PERCEPTION_RESULT_MISSING",)
    reasons = list(perception.safety.reasons) if perception.safety else []
    diagnostics = perception.diagnostics
    nonmonotone = diagnostics.get("frame_frozen") or diagnostics.get("frame_out_of_order")
    if source == "LIVE":
        blocking = [r for r in reasons if r in LIVE_CONE_BLOCKERS or r.startswith(("CALIBRATION", "MOUNT"))]
        if diagnostics.get("calibration_status") != "MOUNT_VERIFIED":
            blocking.insert(0, "LIVE_METRIC_CALIBRATION_UNVERIFIED")
        if nonmonotone:
            blocking.append("NONMONOTONE_FRAME")
        return not blocking, tuple(blocking)
    if nonmonotone:
        return False, ("NONMONOTONE_FRAME", *reasons)
    if source == "SYNTHETIC" and perception.status == "SYNTHETIC_GEOMETRY":
        return True, ()
    if (source == "REPLAY" and perception.status == "RECORDED_OBSERVATIONS"
            and calibration.get("recorded_source") in {"SYNTHETIC", "SIMULATED"}
            and not diagnostics.get("calibration_mismatch")
            and not diagnostics.get("replay_provenance_missing")):
        return True, ()
    return False, ("REPLAY_METRIC_PROVENANCE_UNQUALIFIED", *reasons)


class AlgorithmBackend:
    """Reuses autokart_perception.{pipeline,application,detector} exactly as
    the existing, already-tested ROS wrapper in kart/ros2_ws does: a
    CoreRuntime + Pipeline pair, fed one frame at a time via
    ``sdk.process_frame``. Dependency path resolved at runtime via a
    parameter -- see DEPENDENCIES.md."""

    def __init__(self, algorithm_package_path: str, model_path: Optional[str],
                 model_manifest: Optional[str], calibration_artifact: Optional[str]):
        src = Path(algorithm_package_path).expanduser()
        if not src.is_dir():
            raise PerceptionBackendError(f"DEPENDENCY_PATH_NOT_FOUND:{src}")
        src_str = str(src)
        if src_str not in sys.path:
            sys.path.insert(0, src_str)
        try:
            application = importlib.import_module("autokart_perception.application")
            detector_module = importlib.import_module("autokart_perception.detector")
            pipeline_module = importlib.import_module("autokart_perception.pipeline")
            types_module = importlib.import_module("autokart_perception.types")
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
        self._pipeline = pipeline_module.Pipeline(detector, config, synthetic=detector is None)
        self._model_sha256 = getattr(detector, "model_sha256", "NONE")

    def process(self, rgb, depth, fx, fy, cx, cy, timestamp_ns, source_hint) -> PerceptionOutcome:
        frame = self._Frame(
            rgb=rgb, depth=depth, timestamp_ns=timestamp_ns,
            rgb_timestamp=timestamp_ns // 1_000_000, depth_timestamp=timestamp_ns // 1_000_000,
            rgb_sequence=0, depth_sequence=0,
            intrinsics=self._Intrinsics(fx, fy, cx, cy), depth_scale=0.001,
            aligned=True, source=source_hint,
        )
        t0 = time.monotonic()
        result = self._core.sdk.process_frame(frame, pipeline=self._pipeline)
        processing_ms = (time.monotonic() - t0) * 1000.0
        status = result.perception_result
        valid, blocking = qualify(status, source_hint, frame.calibration)
        cones = tuple(
            ConeObservation(id=c.id, label=c.label, confidence=c.confidence,
                             x_m=c.xy[0], y_m=c.xy[1], sigma_m=c.sigma_m)
            for c in status.cones
        ) if valid else ()
        return PerceptionOutcome(valid=valid, reasons=tuple(blocking), cones=cones,
                                  source=source_hint, model_sha256=self._model_sha256,
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
        )
    raise PerceptionBackendError(f"UNKNOWN_BACKEND:{name}")
