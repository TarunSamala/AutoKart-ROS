"""Single-owner HP60C/replay diagnostic server for the existing Mission Control UI.

No actuator transport exists here. Each API snapshot and its images come from
one Frame and one execution of the existing PerceptionGeometrySDK.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from hashlib import sha256
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hmac
import json
import logging
import mimetypes
import os
from pathlib import Path
import threading
import time
from urllib.parse import urlparse

import cv2
import numpy as np

from .camera import CameraError, HP60C
from .detector import ConeDetector, DetectorError, MODEL_SHA256, Yolo11OnnxDetector, Yolo26OnnxDetector
from .depth import DepthError, validate_frame
from .live_render import depth_preview, encode_jpeg, rgb_preview
from .pipeline import Pipeline
from .recording import Recorder, Replay
from .sdk import PerceptionGeometrySDK
from .shared_core import PoseEstimate
from .synthetic import Synthetic
from .temporal import estimate_rgbd_motion
from .types import Frame


ROOT = Path(__file__).resolve().parents[3]
SIM_SOURCE = "SIMULATED"
CONENET_MODEL = ROOT.parent / "training_runs/conenet_v1_20261014_114447/fit/weights/best.onnx"
CONENET_MANIFEST = ROOT.parent / "training_runs/conenet_v1_20261014_114447/model.json"
CONENET_SHA256 = "0554f1fb28d19a4defac6b65b33e790ac2398e2f577fe353d01cbe28743aa103"
CONENET_CLASSES = ("blue", "large_orange", "orange", "unknown", "yellow")
CONENET_V2_MODEL = ROOT.parent / "models/conenet_v2/model.onnx"
CONENET_V2_MANIFEST = ROOT.parent / "models/conenet_v2/model.json"
CONENET_V2_SHA256 = "80d8c6e84dc312d5e47d8983618253ea6d1cab6f8fb2dd73af1cb00c33f07224"
YOLO26S_MODEL = ROOT.parent / "models/yolo26s/best.onnx"
YOLO26S_MANIFEST = ROOT.parent / "models/yolo26s/best.json"
YOLO26S_SHA256 = "425a89cef2eda9db9c84fbf30cad187e510510b95d7b7e9904003fd22cec1158"
CONESLAYER_MODEL = ROOT / "phase_01_perception/models/coneslayer-simplified.onnx"


def live_detector(kind: str, model_path: Path | None = None,
                  manifest_path: Path | None = None):
    """Select a pinned live detector; ConeSlayer remains an explicit rollback."""
    if kind in {"conenet-v1", "conenet-v2"}:
        default_model, default_manifest, expected_sha, display_name = (
            (CONENET_MODEL, CONENET_MANIFEST, CONENET_SHA256, "ConeNet v1")
            if kind == "conenet-v1" else
            (CONENET_V2_MODEL, CONENET_V2_MANIFEST, CONENET_V2_SHA256, "ConeNet v2"))
        model = model_path or default_model
        manifest = manifest_path or (default_manifest if model_path is None else model.with_suffix(".json"))
        detector = Yolo11OnnxDetector(model, manifest_path=manifest)
        if (detector.model_sha256 != expected_sha or
                tuple(detector.classes) != CONENET_CLASSES):
            raise DetectorError("CONENET_IDENTITY_MISMATCH")
        detector.display_name = display_name
        return detector
    if kind == "yolo26s":
        model = model_path or YOLO26S_MODEL
        manifest = manifest_path or (YOLO26S_MANIFEST if model_path is None else model.with_suffix(".json"))
        detector = Yolo26OnnxDetector(model, manifest_path=manifest)
        if detector.model_sha256 != YOLO26S_SHA256:
            raise DetectorError("YOLO26S_IDENTITY_MISMATCH")
        detector.display_name = "YOLO26s"
        return detector
    if kind == "coneslayer":
        if manifest_path is not None:
            raise DetectorError("CONESLAYER_MANIFEST_NOT_USED")
        return ConeDetector(model_path or CONESLAYER_MODEL,
                            expected_sha256=MODEL_SHA256, label="orange")
    raise DetectorError("UNKNOWN_LIVE_DETECTOR")


@dataclass(frozen=True)
class PublishedFrame:
    frame_id: str
    meta: dict
    rgb_jpeg: bytes
    depth_jpeg: bytes


class SnapshotStore:
    """Bounded immutable snapshots; a fault never exposes the prior geometry."""

    def __init__(self, source: str, maximum_age_ms: float = 250):
        self.source = source
        self.maximum_age_ns = int(maximum_age_ms * 1e6)
        self.lock = threading.Lock()
        self.frames: deque[PublishedFrame] = deque(maxlen=32)
        self.fault_reason = "WAITING_FOR_FRAME"
        self.last_good_ns: int | None = None

    def publish(self, frame: PublishedFrame) -> None:
        with self.lock:
            self.frames.append(frame)
            self.fault_reason = ""
            self.last_good_ns = time.monotonic_ns()

    def fault(self, reason: str) -> None:
        with self.lock:
            self.fault_reason = reason

    def current(self) -> dict:
        with self.lock:
            latest = self.frames[-1] if self.frames else None
            fault = self.fault_reason
            last_good = self.last_good_ns
        age_ms = (time.monotonic_ns() - last_good) / 1e6 if last_good is not None else None
        if latest is None or fault or (self.source == "LIVE" and
                                      (age_ms is None or age_ms > self.maximum_age_ns / 1e6)):
            reason = fault or "STALE_FRAME"
            return {"schema_version": 1, "frame_id": None, "source": self.source,
                    "status": "STALE" if reason == "STALE_FRAME" else "SENSOR_FAULT",
                    "reasons": [reason], "last_good_age_ms": age_ms,
                    "frame_age_ms": ((time.monotonic_ns() - latest.meta["capture_timestamp_ns"]) / 1e6
                                     if latest and latest.meta.get("capture_timestamp_ns") else None),
                    "camera_streaming": False,
                    "geometry": empty_geometry(), "observations": [],
                    "candidate_observations": [], "provisional_3d_observations": [], "rejected_candidates": [],
                    "detections": [],
                    "camera_motion": None,
                    "planning": {"path_valid": False, "local_path": []},
                    "control_candidate": {"valid": False, "target_speed_mps": 0.0,
                                          "steering_angle_rad": 0.0, "published_to_vcu": False},
                    "vehicle_authorized": False, "ready": False}
        capture_ns = latest.meta.get("capture_timestamp_ns")
        return {**latest.meta, "last_good_age_ms": age_ms,
                "frame_age_ms": (time.monotonic_ns() - capture_ns) / 1e6 if capture_ns else None}

    def image(self, frame_id: str, kind: str) -> bytes | None:
        with self.lock:
            frame = next((item for item in self.frames if item.frame_id == frame_id), None)
        if frame is None:
            return None
        return frame.rgb_jpeg if kind == "rgb" else frame.depth_jpeg if kind == "depth" else None


SERVICE_NAME = "autokart_perception.live_mission"
HOP_STATES = ("OK", "NO_FRAME", "STALE", "SDK_ERROR", "CALIBRATION_UNVERIFIED",
              "DEGRADED", "NOT_APPLICABLE", "STARTING")
# Worst first; NOT_APPLICABLE never contributes to the overall state.
_HOP_SEVERITY = {"SDK_ERROR": 6, "NO_FRAME": 5, "STALE": 4, "DEGRADED": 3,
                 "CALIBRATION_UNVERIFIED": 2, "STARTING": 1, "OK": 0}
STARTUP_GRACE_S = 5.0
NON_LIVE_STALE_MS = 1000.0  # sim/replay are paced at ~10 Hz by the frame owner.
FPS_WINDOW_S = 2.0
# Codes the frame owner treats as transient while the vendor stream settles.
TRANSIENT_CAMERA_CODES = frozenset({"FRAME_TIMEOUT", "CALIBRATION_UNAVAILABLE", "STALE_FRAME"})
_GEOMETRY_FAULT_STATUSES = ("INVALID_REPLAY", "FRAME_FROZEN", "FRAME_OUT_OF_ORDER",
                            "CALIBRATION_MISMATCH", "MOUNT_CALIBRATION_INVALID",
                            "MOUNT_CALIBRATION_UNAVAILABLE")


def _finite(value):
    """JSON-safe copy: non-finite floats become null (responses use allow_nan=False)."""
    if isinstance(value, float):
        return value if np.isfinite(value) else None
    if isinstance(value, (np.floating, np.integer)):
        return _finite(value.item())
    if isinstance(value, dict):
        return {key: _finite(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite(item) for item in value]
    return value


def camera_error_code(exc: BaseException) -> str:
    """Stable code from a CameraError message such as 'SDK_START_FAILED:-3'."""
    return str(exc).split(":", 1)[0].strip() or type(exc).__name__


class PipelineHealth:
    """Per-hop counters written by the frame owner and HTTP threads.

    Every value is a count or timestamp of something this process actually
    observed; nothing is inferred when a hop has produced no data.
    """

    def __init__(self, source_label: str, max_frame_age_ms: float):
        self.lock = threading.Lock()
        self.source_label = source_label
        self.stale_ms = (float(max_frame_age_ms) if source_label == "LIVE"
                         else max(float(max_frame_age_ms), NON_LIVE_STALE_MS))
        self.started_at_ns = time.time_ns()
        self.started_monotonic_ns = time.monotonic_ns()
        # sdk_capture
        self.source_opens = 0
        self.frames_received = 0
        self.last_frame_stamp_ns: int | None = None
        self.last_frame_ingest_ns: int | None = None
        self.last_frame_sequence: int | None = None
        self.sdk_received_frames: int | None = None
        self.sdk_mailbox_overwrites: int | None = None
        self.sdk_callback_fps: float | None = None
        self.ingest_times: deque[int] = deque(maxlen=512)
        self.capture_errors = 0
        self.consecutive_capture_errors = 0
        self.capture_error_codes: dict[str, int] = {}
        self.last_capture_error: str | None = None
        self.last_capture_error_detail: str | None = None
        self.last_capture_error_ns: int | None = None
        self.worker_exit_reason: str | None = None
        # processing (frame received but snapshot not produced)
        self.processing_failures = 0
        self.last_processing_error: str | None = None
        self.last_processing_error_ns: int | None = None
        # detector
        self.inference_count = 0
        self.inference_failures = 0
        self.last_inference_ms: float | None = None
        self.last_inference_ns: int | None = None
        self.last_detection_count: int | None = None
        self.last_detector_error: str | None = None
        self.last_detector_error_ns: int | None = None
        # geometry
        self.processed_frames = 0
        self.qualified_frames = 0
        self.last_processed_ns: int | None = None
        self.last_frame_id: str | None = None
        self.last_perception_status: str | None = None
        self.last_snapshot_status: str | None = None
        self.last_reasons: list[str] = []
        self.last_counts: dict = {}
        self.last_geometry_status: str | None = None
        self.last_path_valid: bool | None = None
        self.last_camera_streaming: bool | None = None
        self.last_calibration_applies: bool | None = None
        # http
        self.snapshots_served = 0
        self.image_requests = 0
        self.image_not_found = 0
        self.diagnostics_served = 0
        self.unauthorized_requests = 0
        self.last_client_request_ns: int | None = None

    # -- writers -------------------------------------------------------
    def note_source_open(self) -> None:
        with self.lock:
            self.source_opens += 1

    def note_frame(self, frame: Frame) -> None:
        now = time.monotonic_ns()
        with self.lock:
            self.frames_received += 1
            self.consecutive_capture_errors = 0
            self.last_frame_stamp_ns = int(frame.timestamp_ns)
            self.last_frame_ingest_ns = now
            self.last_frame_sequence = int(frame.rgb_sequence)
            self.ingest_times.append(now)
            if frame.source == "LIVE":
                self.sdk_received_frames = int(frame.received_frames)
                self.sdk_mailbox_overwrites = int(frame.drops)
                self.sdk_callback_fps = ((frame.received_frames - 1) * 1e9 / frame.capture_elapsed_ns
                                         if frame.received_frames > 1 and frame.capture_elapsed_ns > 0
                                         else None)

    def note_capture_error(self, code: str, detail: str) -> None:
        with self.lock:
            self.capture_errors += 1
            self.consecutive_capture_errors += 1
            self.capture_error_codes[code] = self.capture_error_codes.get(code, 0) + 1
            self.last_capture_error = code
            self.last_capture_error_detail = detail
            self.last_capture_error_ns = time.monotonic_ns()

    def note_processing_error(self, reason: str) -> None:
        with self.lock:
            self.processing_failures += 1
            self.last_processing_error = reason
            self.last_processing_error_ns = time.monotonic_ns()

    def note_processed(self, meta: dict, perception_status: str, diagnostics: dict,
                       calibration_applies: bool) -> None:
        now = time.monotonic_ns()
        geometry = meta.get("geometry") or {}
        with self.lock:
            self.processed_frames += 1
            self.last_processed_ns = now
            self.last_frame_id = meta.get("frame_id")
            self.last_perception_status = perception_status
            self.last_snapshot_status = meta.get("status")
            self.last_reasons = list(meta.get("reasons", []))
            self.last_camera_streaming = bool(meta.get("camera_streaming"))
            self.last_calibration_applies = calibration_applies
            self.last_geometry_status = geometry.get("status")
            self.last_path_valid = bool((meta.get("planning") or {}).get("path_valid"))
            qualified = len(meta.get("observations", []))
            self.qualified_frames += 1 if qualified else 0
            self.last_counts = {
                "qualified_observations": qualified,
                "candidate_observations": len(meta.get("candidate_observations", [])),
                "provisional_3d_observations": len(meta.get("provisional_3d_observations", [])),
                "rejected_candidates": len(meta.get("rejected_candidates", [])),
                "delaunay_edges": len(geometry.get("delaunay_edges", [])),
                "gabriel_edges": len(geometry.get("gabriel_edges", [])),
                "gates": len(geometry.get("gates", [])),
                "centerline_points": len(geometry.get("centerline", [])),
            }
            if "inference_ms" in diagnostics:
                self.inference_count += 1
                self.last_inference_ms = diagnostics.get("inference_ms")
                self.last_inference_ns = now
                self.last_detection_count = len(meta.get("detections", []))
            if str(perception_status).startswith("INFERENCE_FAILURE"):
                self.inference_failures += 1
                self.last_detector_error = str(perception_status)
                self.last_detector_error_ns = now

    def note_worker_exit(self, reason: str | None) -> None:
        with self.lock:
            self.worker_exit_reason = reason or "WORKER_EXITED"

    def note_http(self, kind: str) -> None:
        with self.lock:
            if kind == "snapshot":
                self.snapshots_served += 1
            elif kind == "image":
                self.image_requests += 1
            elif kind == "image_not_found":
                self.image_requests += 1
                self.image_not_found += 1
            elif kind == "diagnostics":
                self.diagnostics_served += 1
            elif kind == "unauthorized":
                self.unauthorized_requests += 1
            if kind in {"snapshot", "image", "image_not_found"}:
                self.last_client_request_ns = time.monotonic_ns()

    # -- report --------------------------------------------------------
    def report(self, *, mode: str, detector, mount_status: str, worker_alive: bool,
               camera_owner: bool) -> dict:
        now = time.monotonic_ns()

        def age_ms(stamp):
            return (now - stamp) / 1e6 if stamp is not None else None

        with self.lock:
            uptime_s = (now - self.started_monotonic_ns) / 1e9
            in_grace = uptime_s < STARTUP_GRACE_S
            window = [t for t in self.ingest_times if now - t <= FPS_WINDOW_S * 1e9]
            measured_fps = ((len(window) - 1) * 1e9 / (window[-1] - window[0])
                            if len(window) > 1 and window[-1] > window[0] else
                            0.0 if self.frames_received else None)
            # LIVE and SIMULATED stamps share this host's monotonic clock; REPLAY
            # stamps are the recorded clock, so their age is measured at ingest.
            if self.source_label == "REPLAY":
                frame_age, age_basis = age_ms(self.last_frame_ingest_ns), "HOST_INGEST_MONOTONIC"
            else:
                frame_age = age_ms(self.last_frame_stamp_ns) if self.last_frame_stamp_ns else None
                age_basis = "CAPTURE_STAMP_MONOTONIC"
            error_is_current = (self.last_capture_error_ns is not None and
                                (self.last_frame_ingest_ns is None or
                                 self.last_capture_error_ns > self.last_frame_ingest_ns))

            # sdk_capture
            capture_reason = None
            if error_is_current and mode == "live":
                if (self.last_capture_error in TRANSIENT_CAMERA_CODES and
                        self.frames_received == 0 and in_grace):
                    capture_state = "STARTING"
                elif self.last_capture_error == "STALE_FRAME":
                    capture_state = "STALE"
                else:
                    capture_state = "SDK_ERROR"
                capture_reason = self.last_capture_error
            elif error_is_current:
                capture_state, capture_reason = "NO_FRAME", self.last_capture_error
            elif not worker_alive and self.worker_exit_reason is not None:
                capture_state, capture_reason = "NO_FRAME", self.worker_exit_reason
            elif self.frames_received == 0:
                capture_state = "STARTING" if in_grace else "NO_FRAME"
                capture_reason = "WAITING_FOR_FRAME"
            elif frame_age is not None and frame_age > self.stale_ms:
                capture_state, capture_reason = "STALE", "FRAME_AGE_EXCEEDED"
            elif mode == "live" and self.last_camera_streaming is False:
                capture_state, capture_reason = "DEGRADED", "RGB_DEPTH_NOT_STREAMING"
            else:
                capture_state = "OK"
            capture = {
                "name": "sdk_capture", "state": capture_state, "reason": capture_reason,
                "source": self.source_label,
                "uses_vendor_sdk": mode == "live",
                "last_error": self.last_capture_error,
                "last_error_detail": self.last_capture_error_detail,
                "last_error_age_ms": age_ms(self.last_capture_error_ns),
                "error_counts": dict(self.capture_error_codes),
                "consecutive_errors": self.consecutive_capture_errors,
                "source_opens": self.source_opens,
                "frames_received": self.frames_received,
                "last_frame_stamp_ns": self.last_frame_stamp_ns,
                "last_frame_sequence": self.last_frame_sequence,
                "frame_age_ms": frame_age, "frame_age_basis": age_basis,
                "stale_after_ms": self.stale_ms,
                "measured_fps": measured_fps, "fps_window_s": FPS_WINDOW_S,
                "sdk_callback_fps": self.sdk_callback_fps,
                "sdk_received_frames": self.sdk_received_frames,
                "drops": {"sdk_mailbox_overwrites": self.sdk_mailbox_overwrites,
                          "processing_failures": self.processing_failures},
                "last_processing_error": self.last_processing_error,
                "worker_alive": worker_alive,
                "worker_exit_reason": self.worker_exit_reason,
            }

            def downstream_state(last_ns, *, grace_ok=True):
                if last_ns is None:
                    return ("STARTING" if in_grace and grace_ok and capture_state in {"OK", "STARTING"}
                            else "NO_FRAME")
                if age_ms(last_ns) > self.stale_ms:
                    return "STALE"
                return None

            # detector
            detector_reason = None
            if detector is None:
                detector_state = "NOT_APPLICABLE"
                detector_reason = {"sim": "SIMULATED_GEOMETRY_NO_INFERENCE",
                                   "replay": "RECORDED_OBSERVATIONS_NO_INFERENCE"}.get(
                                       mode, "MODEL_NOT_AVAILABLE")
                if mode == "live":
                    detector_state = "DEGRADED"
            else:
                last_activity = max(filter(None, (self.last_inference_ns, self.last_detector_error_ns)),
                                    default=None)
                detector_state = downstream_state(last_activity)
                if detector_state is None:
                    if (self.last_detector_error_ns is not None and
                            (self.last_inference_ns is None or
                             self.last_detector_error_ns >= self.last_inference_ns)):
                        detector_state, detector_reason = "DEGRADED", self.last_detector_error
                    else:
                        detector_state = "OK"
            detector_hop = {
                "name": "detector", "state": detector_state, "reason": detector_reason,
                "detector": (getattr(detector, "display_name", None) or
                             ("ConeSlayer" if detector is not None else None)),
                "model_sha256": getattr(detector, "model_sha256", None) if detector is not None else None,
                "inference_count": self.inference_count,
                "inference_failures": self.inference_failures,
                "last_latency_ms": self.last_inference_ms,
                "last_inference_age_ms": age_ms(self.last_inference_ns),
                "last_detection_count": self.last_detection_count,
                "last_error": self.last_detector_error,
                "last_error_age_ms": age_ms(self.last_detector_error_ns),
            }

            # geometry
            geometry_reason = None
            geometry_state = downstream_state(self.last_processed_ns)
            calibration_applies = bool(self.last_calibration_applies)
            if geometry_state is None:
                status = str(self.last_perception_status or "")
                if status.startswith(_GEOMETRY_FAULT_STATUSES) or mount_status in {
                        "MOUNT_CALIBRATION_INVALID", "MOUNT_CALIBRATION_UNAVAILABLE"}:
                    geometry_state, geometry_reason = "DEGRADED", status or mount_status
                elif calibration_applies and (mount_status != "MOUNT_VERIFIED" or
                                              "CALIBRATION_UNVERIFIED" in self.last_reasons):
                    geometry_state, geometry_reason = "CALIBRATION_UNVERIFIED", mount_status
                else:
                    geometry_state = "OK"
            geometry_hop = {
                "name": "geometry", "state": geometry_state, "reason": geometry_reason,
                "calibration_status": mount_status,
                "calibration_applies": self.last_calibration_applies,
                "processed_frames": self.processed_frames,
                "qualified_frames": self.qualified_frames,
                "last_frame_id": self.last_frame_id,
                "last_processed_age_ms": age_ms(self.last_processed_ns),
                "perception_status": self.last_perception_status,
                "snapshot_status": self.last_snapshot_status,
                "geometry_status": self.last_geometry_status,
                "path_valid": self.last_path_valid,
                "reason_codes": list(self.last_reasons),
                "counts": dict(self.last_counts),
            }

            http_hop = {
                "name": "http", "state": "OK", "reason": None,
                "snapshots_served": self.snapshots_served,
                "image_requests": self.image_requests,
                "image_not_found": self.image_not_found,
                "diagnostics_served": self.diagnostics_served,
                "unauthorized_requests": self.unauthorized_requests,
                "last_client_request_age_ms": age_ms(self.last_client_request_ns),
            }
            hops = [capture, detector_hop, geometry_hop, http_hop]
            started_at_ns = self.started_at_ns
        overall = max((hop["state"] for hop in hops if hop["state"] != "NOT_APPLICABLE"),
                      key=_HOP_SEVERITY.__getitem__)
        return _finite({
            "schema_version": 1,
            "service": {"name": SERVICE_NAME, "mode": mode, "source": self.source_label,
                        "pid": os.getpid(), "started_at_ns": started_at_ns,
                        "uptime_s": uptime_s, "camera_owner": camera_owner,
                        "detector": detector_hop["detector"],
                        "model_sha256": detector_hop["model_sha256"]},
            "hops": hops, "overall": overall, "vehicle_authorized": False,
        })


def empty_geometry() -> dict:
    return {"delaunay_edges": [], "gabriel_edges": [], "gates": [],
            "centerline": [], "boundaries": {"left_ids": [], "right_ids": []},
            "skidpad_topology": None}


class LiveSession:
    def __init__(self, *, mode: str, detector=None, config_path: Path | None = None,
                 record_path: Path | None = None, max_record_frames: int = 0,
                 log_path: Path | None = None, max_log_frames: int = 20000,
                 frame_limit: int = 0, source_factory=None,
                 bridge_library: Path | None = None,
                 camera_configuration: Path | None = None):
        if mode not in {"live", "replay", "sim"}:
            raise ValueError("INVALID_MODE")
        if record_path is not None and max_record_frames <= 0:
            raise ValueError("RECORD_FRAME_LIMIT_REQUIRED")
        self.mode = mode
        self.source_label = {"live": "LIVE", "replay": "REPLAY", "sim": SIM_SOURCE}[mode]
        self.detector = detector
        self.config_path = config_path
        self.sdk = self._new_sdk()
        self.pipeline = Pipeline(detector, self.sdk.config, synthetic=mode == "sim")
        self.store = SnapshotStore(self.source_label, self.sdk.config["max_frame_age_ms"])
        self.health = PipelineHealth(self.source_label, self.sdk.config["max_frame_age_ms"])
        self.recorder = Recorder(record_path, detector.version if detector else "none",
                                 getattr(detector, "model_sha256", None)) if record_path else None
        self.max_record_frames = max_record_frames
        self.recorded = 0
        self.log_path = log_path
        self.max_log_frames = max_log_frames
        self.logged = 0
        self.frame_limit = frame_limit
        self.processed = 0
        self.stop_event = threading.Event()
        self.worker: threading.Thread | None = None
        self.source = None
        self.source_factory = source_factory
        self.bridge_library = bridge_library
        self.camera_configuration = camera_configuration
        self.previous_motion_frame: Frame | None = None

    def _new_sdk(self) -> PerceptionGeometrySDK:
        return (PerceptionGeometrySDK(detector=self.detector) if self.config_path is None else
                PerceptionGeometrySDK.from_yaml(self.config_path, detector=self.detector))

    def _make_source(self):
        if self.source_factory is not None:
            return self.source_factory()
        if self.mode == "live":
            return HP60C(library=self.bridge_library,
                         configuration=self.camera_configuration,
                         max_age_ms=self.sdk.config["max_frame_age_ms"])
        if self.mode == "sim":
            return Synthetic()
        raise ValueError("REPLAY_SOURCE_REQUIRED")

    def process_one(self, frame: Frame) -> PublishedFrame:
        if frame.source != self.source_label and not (self.mode == "sim" and frame.source == "SYNTHETIC"):
            raise ValueError("SOURCE_MISMATCH")
        started = time.perf_counter()
        pose = (PoseEstimate(0.0, 0.0, 0.0, 0.0, frame.timestamp_ns, .05)
                if self.mode == "sim" else None)
        result = self.sdk.process_frame(frame, pose, pipeline=self.pipeline)
        perception = result.perception_result
        if perception is None:
            raise RuntimeError("PERCEPTION_RESULT_MISSING")
        if self.recorder is not None and self.recorded < self.max_record_frames:
            self.recorder.write(perception)
            self.recorded += 1
        depth_image, depth_stats = depth_preview(frame.depth)
        # The bundled bench mount and HP60C depth scale are not surveyed.
        # Never project a diagnostic candidate path into camera pixels here.
        rgb_image = rgb_preview(frame, result, extrinsic_verified=False)
        rgb_jpeg, depth_jpeg = encode_jpeg(rgb_image), encode_jpeg(depth_image)
        raw = result.to_dict()
        motion = None
        motion_input_accepted = (perception.status in {"OK", "METRIC_LOCALIZATION_FAILED",
                                                        "MODEL_NOT_AVAILABLE", "RECORDED_OBSERVATIONS"}
                                 and not perception.diagnostics.get("frame_frozen")
                                 and not perception.diagnostics.get("frame_out_of_order"))
        if frame.source in {"LIVE", "REPLAY"} and motion_input_accepted:
            try:
                validate_frame(frame, self.sdk.config["max_frame_age_ms"])
            except DepthError:
                self.previous_motion_frame = None
            else:
                if self.previous_motion_frame is None:
                    motion = {"status": "MOTION_REFERENCE_UNAVAILABLE",
                              "previous_timestamp_ns": 0,
                              "current_timestamp_ns": frame.timestamp_ns,
                              "matched_points": 0, "inlier_count": 0,
                              "reprojection_rms_px": None,
                              "qualified": False, "frame": "CAMERA_OPTICAL"}
                else:
                    estimate = estimate_rgbd_motion(self.previous_motion_frame, frame)
                    motion = {"status": estimate.status,
                              "previous_timestamp_ns": estimate.previous_timestamp_ns,
                              "current_timestamp_ns": estimate.current_timestamp_ns,
                              "matched_points": estimate.matched_points,
                              "inlier_count": estimate.inlier_count,
                              "reprojection_rms_px": estimate.reprojection_rms_px,
                              "qualified": False, "frame": "CAMERA_OPTICAL"}
                self.previous_motion_frame = frame
        else:
            self.previous_motion_frame = None
        unqualified_live_origin = (frame.source == "LIVE" or
            (frame.source == "REPLAY" and
             frame.calibration.get("recorded_source") not in {"SYNTHETIC", "SIMULATED"}))
        self.processed += 1
        frame_id = f"{self.source_label.lower()}-{self.processed}-{frame.timestamp_ns}"
        now_ns = time.monotonic_ns()
        reasons = sorted(set(raw["reasons"] + ["DEPTH_SCALE_UNVERIFIED", "CALIBRATION_UNVERIFIED"]
                             if frame.source == "LIVE" else raw["reasons"]))
        health = "SENSOR_FAULT" if result.perception and result.perception["safety_state"] == "SENSOR_FAULT" else result.status
        if frame.source == "LIVE" and health == "VALID":
            health = "DEGRADED"
        diagnostics = perception.diagnostics
        provisional = []
        for item in diagnostics.get("cone_details", []):
            if item.get("position_kind") != "PROVISIONAL_3D_POSITION":
                continue
            body = item.get("nominal_body_xyz_m")
            if body is not None and not np.isfinite(body).all():
                continue
            provisional.append({"id": item["id"], "label": item["label"],
                "confidence": item["confidence"], "box": item["box"],
                "optical_xyz_m": item["optical_xyz_m"],
                "nominal_body_xyz_m": body, "nominal_xy_m": body[:2] if body is not None else None,
                "uncertainty_m": item["surface_uncertainty_m"],
                "ground_contact_status": item.get("ground_contact_status", "UNAVAILABLE"),
                "position_kind": "PROVISIONAL_3D_POSITION"})
        meta = {
            "schema_version": 1, "frame_id": frame_id, "source": self.source_label,
            "capture_timestamp_ns": frame.timestamp_ns, "processed_timestamp_ns": now_ns,
            "camera_frame": "hp60c_optical" if frame.source == "LIVE" else "recorded_optical",
            "base_frame": "camera_ground_nominal" if unqualified_live_origin else "base_link",
            "calibration_id": diagnostics.get("calibration_sha256"),
            "calibration_status": self.pipeline.mount_status,
            "camera_id": "HP60C" if frame.source == "LIVE" else self.source_label,
            "camera_sequence": frame.rgb_sequence, "depth_sequence": frame.depth_sequence,
            "intrinsics": frame.intrinsics.__dict__.copy(),
            "rgb_depth_timestamp_delta_ms": abs(frame.rgb_timestamp-frame.depth_timestamp),
            "rgb_depth_pair": ("SDK_PAIRED_CALLBACK" if frame.source == "LIVE" else
                               "RECORDED_PAIR" if frame.source == "REPLAY" else "SIMULATED_PAIR"),
            "pose_status": "SIMULATED" if pose is not None else "UNAVAILABLE",
            "rgb_depth_alignment": diagnostics.get("rgb_depth_registration"),
            "depth_stats": depth_stats, "depth_scale_m_per_raw_candidate": frame.depth_scale,
            "depth_scale_provenance": "VENDOR_ROS_DIVIDE_BY_1000_UNSURVEYED",
            "depth_scale_qualified": False,
            "model": {"name": (getattr(self.detector, "display_name", None) or
                               "ConeSlayer" if self.detector else "none"),
                      "version": self.detector.version if self.detector else "none",
                      "sha256": self.detector.model_sha256 if self.detector else None,
                      "supported_classes": list(self.detector.supported_classes) if self.detector else [],
                      "qualification": ("EXPERIMENTAL_HP60C_VALIDATION_PENDING"
                                        if getattr(self.detector, "stage", None) == "experimental" else
                                        "ORANGE_ONLY_UNVERIFIED_IN_FIELD" if self.detector else "NO_DETECTOR")},
            "algorithm_version": result.algorithm_version,
            "configuration_sha256": result.configuration_sha256,
            "status": health, "reasons": reasons,
            "detections": result.perception["detections"] if result.perception else [],
            "observations": raw["observations"],
            "candidate_observations": ([item.__dict__.copy() for item in perception.cones]
                                       if unqualified_live_origin else []),
            "provisional_3d_observations": provisional if unqualified_live_origin else [],
            "rejected_candidates": [item for item in diagnostics.get("cone_details", [])
                                    if item.get("localization_status") not in
                                    {"METRIC_OK", "PROVISIONAL_3D_POSITION"}],
            "tracked_cones": raw["tracked_cones"],
            "tracking_status": "FRAME_LOCAL_IDS_ONLY_WITHOUT_POSE",
            "camera_motion": motion,
            "geometry": raw["geometry"], "planning": raw["planning"],
            "control_candidate": raw["control_candidate"],
            "mission_state": raw["mission_state"],
            "perception": result.perception,
            "detector_counts": {**diagnostics.get("detector_counts", {}),
                                "application_detections": len(result.perception["detections"]),
                                "depth_localized_candidates": len(perception.cones),
                                "validated_metric": 0},
            "stream_metrics": {
                "camera_callback_fps": ((frame.received_frames - 1) * 1e9 / frame.capture_elapsed_ns
                                        if frame.received_frames > 1 and frame.capture_elapsed_ns > 0 else None),
                "received_frames": frame.received_frames,
                "mailbox_overwrites": frame.drops, "mailbox_capacity": 1,
                "processed_fps": diagnostics.get("end_to_end_fps"),
                "inference_fps": diagnostics.get("inference_fps")},
            "latencies_ms": {
                "capture_to_publish": (now_ns - frame.timestamp_ns) / 1e6 if frame.source == "LIVE" else None,
                "inference": diagnostics.get("inference_ms"),
                "depth_localization": diagnostics.get("depth_localization_ms"),
                "delaunay": diagnostics.get("delaunay_ms"),
                "gabriel": diagnostics.get("gabriel_ms"),
                "gates": diagnostics.get("gate_ms"),
                "centerline_curvature": diagnostics.get("centerline_curvature_ms"),
                "sdk_total": result.processing_ms,
                "snapshot_total": (time.perf_counter() - started) * 1000},
            "camera_streaming": bool(frame.source == "LIVE" and frame.aligned and
                                     depth_stats["valid_pixels"] > 0),
            "ready": False,  # Metric depth, surveyed mount, and pose are unqualified.
            "vehicle_authorized": False,
        }
        published = PublishedFrame(frame_id, meta, rgb_jpeg, depth_jpeg)
        self.store.publish(published)
        self.health.note_processed(meta, perception.status, diagnostics,
                                   calibration_applies=unqualified_live_origin)
        if self.log_path is not None and self.logged < self.max_log_frames:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            with self.log_path.open("a") as stream:
                stream.write(json.dumps({"frame_id": frame_id, "source": self.source_label,
                    "capture_timestamp_ns": frame.timestamp_ns, "processed_timestamp_ns": now_ns,
                    "model_sha256": meta["model"]["sha256"],
                    "depth_stats": depth_stats, "status": health, "reasons": reasons,
                    "detections": len(meta["detections"]), "observations": len(meta["observations"]),
                    "candidate_observations": len(meta["candidate_observations"]),
                    "provisional_3d_observations": len(meta["provisional_3d_observations"]),
                    "cone_details": diagnostics.get("cone_details", []),
                    "detector_counts": meta["detector_counts"],
                    "camera_motion": motion,
                    "stream_metrics": meta["stream_metrics"],
                    "camera_sequence": frame.rgb_sequence, "depth_sequence": frame.depth_sequence,
                    "detected_classes": [item["label"] for item in meta["detections"]],
                    "delaunay_edges": len(meta["geometry"]["delaunay_edges"]),
                    "gabriel_edges": len(meta["geometry"]["gabriel_edges"]),
                    "geometry_status": meta["geometry"].get("status"),
                    "boundary_status": meta["geometry"].get("boundaries", {}).get("status"),
                    "centerline_status": meta["geometry"].get("centerline_status"),
                    "topology_reasons": meta["geometry"].get("topology_reasons", []),
                    "gates": len(meta["geometry"]["gates"]),
                    "centerline_points": len(meta["geometry"]["centerline"]),
                    "perception_status": perception.status,
                    "latencies_ms": meta["latencies_ms"], "vehicle_authorized": False}) + "\n")
            self.logged += 1
        return published

    def _close_source(self) -> None:
        self.previous_motion_frame = None
        if self.source is not None and hasattr(self.source, "close"):
            self.source.close()
        self.source = None

    def camera_owner(self) -> bool:
        """True only while this process holds an open HP60C vendor SDK session."""
        source = self.source
        return self.mode == "live" and isinstance(source, HP60C) and bool(source.started)

    def pipeline_health(self) -> dict:
        return self.health.report(mode=self.mode, detector=self.detector,
                                  mount_status=self.pipeline.mount_status,
                                  worker_alive=bool(self.worker and self.worker.is_alive()),
                                  camera_owner=self.camera_owner())

    def run(self) -> None:
        try:
            while not self.stop_event.is_set() and (not self.frame_limit or self.processed < self.frame_limit):
                capturing = True
                try:
                    if self.source is None:
                        self.source = self._make_source()
                        self.health.note_source_open()
                        self.sdk = self._new_sdk()
                        self.pipeline = Pipeline(self.detector, self.sdk.config,
                                                 synthetic=self.mode == "sim")
                    frame = self.source.read()
                    if frame is None:
                        self.store.fault("REPLAY_COMPLETE")
                        break
                    self.health.note_frame(frame)
                    capturing = False
                    self.process_one(frame)
                    if self.mode != "live":
                        self.stop_event.wait(.1)
                except Exception as exc:
                    reason = f"{type(exc).__name__}:{exc}"
                    if capturing:
                        code = (camera_error_code(exc) if isinstance(exc, CameraError) else
                                "SOURCE_OPEN_FAILED" if self.source is None else "SOURCE_READ_FAILED")
                        self.health.note_capture_error(code, reason)
                    else:
                        self.health.note_processing_error(reason)
                    self.store.fault(reason)
                    self.previous_motion_frame = None
                    if self.recorder is not None:
                        self.recorder.record_fault(reason, self.source_label)
                    if (self.mode == "live" and isinstance(exc, CameraError) and
                        str(exc) in {"FRAME_TIMEOUT", "CALIBRATION_UNAVAILABLE", "STALE_FRAME"}):
                        # The vendor stream and intrinsics can arrive after startup.
                        # Keep one SDK owner while waiting; publish no old geometry.
                        self.stop_event.wait(.1)
                        continue
                    logging.getLogger(__name__).exception("Frame owner fault: %s", reason)
                    self.pipeline.note_sensor_fault(reason)
                    self._close_source()
                    if self.mode != "live":
                        break
                    self.stop_event.wait(1.0)
            if self.frame_limit and self.processed >= self.frame_limit:
                self.store.fault("FRAME_LIMIT_REACHED")
        finally:
            self._close_source()
            self.health.note_worker_exit(self.store.fault_reason or
                                         ("STOPPED" if self.stop_event.is_set() else None))

    def start(self) -> None:
        self.worker = threading.Thread(target=self.run, name="autokart-live-frame-owner", daemon=True)
        self.worker.start()

    def close(self) -> None:
        self.stop_event.set()
        if self.worker is not None:
            self.worker.join(timeout=3)
        self._close_source()


PIPELINE_DIAGNOSTICS_PATH = "/api/v1/diagnostics/pipeline"


def build_server(session: LiveSession, *, host: str, port: int, static_root: Path,
                 token: str | None = None) -> ThreadingHTTPServer:
    """Bind the read-only diagnostic HTTP server; the caller starts the session."""
    static_root = static_root.resolve()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args) -> None:
            # High-rate image polling is accounted for in the bounded frame log.
            if len(args) > 1 and str(args[1]).startswith(("4", "5")):
                super().log_message(format, *args)

        def _authorized(self) -> bool:
            if token is None:
                return True
            supplied = self.headers.get("Authorization", "")
            return hmac.compare_digest(supplied, "Bearer " + token)

        def _send(self, code: int, data: bytes, content_type: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _json(self, code: int, value: dict) -> None:
            self._send(code, json.dumps(value, allow_nan=False).encode(), "application/json")

        def do_GET(self) -> None:
            path = urlparse(self.path).path
            if path.startswith("/api/") or path in {"/health", "/ready"}:
                if not self._authorized():
                    session.health.note_http("unauthorized")
                    self._json(401, {"error": "AUTH_REQUIRED"})
                    return
            if path == PIPELINE_DIAGNOSTICS_PATH:
                session.health.note_http("diagnostics")
                self._json(200, session.pipeline_health())
                return
            current = session.store.current()
            if path == "/health":
                self._json(200, {"server": "RUNNING", "source": session.source_label,
                                 "vehicle_authorized": False})
            elif path == "/ready":
                ready = bool(current.get("ready"))
                self._json(200 if ready else 503, {"ready": ready,
                                                    "status": current["status"],
                                                    "vehicle_authorized": False})
            elif path == "/api/snapshot":
                session.health.note_http("snapshot")
                self._json(200, current)
            elif path.startswith("/api/image/"):
                parts = path.split("/")
                if len(parts) != 5 or parts[4] not in {"rgb", "depth"}:
                    self._json(404, {"error": "NOT_FOUND"})
                    return
                data = session.store.image(parts[3], parts[4])
                session.health.note_http("image" if data is not None else "image_not_found")
                if data is None:
                    self._json(404, {"error": "SNAPSHOT_EXPIRED"})
                else:
                    self._send(200, data, "image/jpeg")
            elif path == "/":
                self.send_response(302)
                self.send_header("Location", "/live")
                self.send_header("Content-Length", "0")
                self.end_headers()
            else:
                if path != "/live" and not path.startswith("/assets/"):
                    self._json(404, {"error": "NOT_FOUND"})
                    return
                candidate = (static_root / (path.lstrip("/") if path.startswith("/assets/") else "index.html")).resolve()
                if not candidate.is_relative_to(static_root) or not candidate.is_file():
                    self._json(404, {"error": "NOT_FOUND"})
                else:
                    self._send(200, candidate.read_bytes(), mimetypes.guess_type(candidate)[0] or "text/html")

        def do_POST(self) -> None:
            self._json(405, {"error": "READ_ONLY_DIAGNOSTIC"})

    return ThreadingHTTPServer((host, port), Handler)


def serve(session: LiveSession, *, host: str, port: int, static_root: Path,
          token: str | None = None) -> None:
    server = build_server(session, host=host, port=port, static_root=static_root, token=token)
    session.start()
    try:
        server.serve_forever(poll_interval=.2)
    finally:
        server.server_close()
        session.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Disarmed AutoKart live Mission Control")
    parser.add_argument("--mode", choices=("live", "replay", "sim"), required=True)
    parser.add_argument("--camera", choices=("hp60c",), default="hp60c")
    parser.add_argument("--replay", type=Path)
    parser.add_argument("--record", type=Path)
    parser.add_argument("--max-record-frames", type=int, default=0)
    parser.add_argument("--log", type=Path)
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--detector", choices=("yolo26s", "conenet-v1", "conenet-v2", "coneslayer"), default="conenet-v1")
    parser.add_argument("--opencv-threads", type=int, default=4,
                        help="OpenCV inference threads for this diagnostic process (default: 4)")
    parser.add_argument("--model", type=Path, help="ONNX path; defaults to pinned detector model")
    parser.add_argument("--manifest", type=Path, help="Detector manifest; defaults to the selected model manifest")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--bridge-library", type=Path)
    parser.add_argument("--camera-configuration", type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8768)
    parser.add_argument("--token-file", type=Path)
    parser.add_argument("--static-root", type=Path,
                        default=ROOT.parent / "phase_sim/dist")
    args = parser.parse_args(argv)
    if args.mode == "replay" and args.replay is None:
        parser.error("--replay is required for replay mode")
    if not 1 <= args.opencv_threads <= 32:
        parser.error("--opencv-threads must be between 1 and 32")
    if args.host not in {"127.0.0.1", "localhost", "::1"} and args.token_file is None:
        parser.error("nonlocal binding requires --token-file")
    token = args.token_file.read_text().strip() if args.token_file else None
    if args.token_file and (not token or len(token) < 24):
        parser.error("token file must contain at least 24 characters")
    if not args.static_root.is_dir():
        parser.error("built phase_sim/dist missing; run npm run build")
    detector = None
    if args.mode == "live":
        cv2.setNumThreads(args.opencv_threads)
        detector = live_detector(args.detector, args.model, args.manifest)
    factory = (lambda: Replay(args.replay)) if args.mode == "replay" else None
    session = LiveSession(mode=args.mode, detector=detector, config_path=args.config,
                          record_path=args.record, max_record_frames=args.max_record_frames,
                          log_path=args.log, frame_limit=args.frames,
                          source_factory=factory, bridge_library=args.bridge_library,
                          camera_configuration=args.camera_configuration)
    logging.getLogger("autokart_perception.sdk").setLevel(logging.ERROR)
    print(f"AutoKart diagnostic dashboard: http://{args.host}:{args.port}/live", flush=True)
    print("Vehicle authorization: DISABLED", flush=True)
    try:
        serve(session, host=args.host, port=args.port, static_root=args.static_root, token=token)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
