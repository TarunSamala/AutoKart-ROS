"""Bounded, read-only preview of paired ROS images from the perception owner."""

from __future__ import annotations

from collections import deque
from math import isfinite
from threading import Lock
import time

import numpy as np

from autokart_perception.live_render import depth_preview, encode_jpeg


def image_array(message, kind: str) -> np.ndarray:
    """Copy one ROS image without assuming tightly packed rows or native endian."""
    width, height, step = int(message.width), int(message.height), int(message.step)
    if not 0 < width <= 1920 or not 0 < height <= 1080:
        raise ValueError("IMAGE_DIMENSIONS_INVALID")
    if kind == "rgb":
        if message.encoding not in {"bgr8", "rgb8"} or step < width * 3:
            raise ValueError("RGB_ENCODING_INVALID")
        if len(message.data) != step * height:
            raise ValueError("RGB_LENGTH_INVALID")
        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(height, step)
        image = rows[:, :width * 3].reshape(height, width, 3).copy()
        return image if message.encoding == "bgr8" else image[:, :, ::-1].copy()
    if kind == "depth":
        if message.encoding not in {"mono16", "16UC1"} or step < width * 2 or step % 2:
            raise ValueError("DEPTH_ENCODING_INVALID")
        if len(message.data) != step * height:
            raise ValueError("DEPTH_LENGTH_INVALID")
        endian = ">u2" if message.is_bigendian else "<u2"
        rows = np.frombuffer(message.data, dtype=endian).reshape(height, step // 2)
        return rows[:, :width].astype(np.uint16, copy=True)
    raise ValueError("IMAGE_KIND_INVALID")


PASSTHROUGH = ("rgb_timestamp_ms", "depth_timestamp_ms", "rgb_sequence", "depth_sequence", "frame_age_ms",
               "inference_ms", "model_version", "model_backend", "model_stage", "model_classes", "safety_state",
               "latched_fault", "calibration_reasons", "depth_scale_factor", "depth_scale_verified",
               "mount_verified", "calibration_artifact")


class CameraFeed:
    """Matches RGB, depth and status by capture stamp; never owns the camera."""

    def __init__(self, max_age_s: float = 1.0):
        self.max_age_s = max_age_s
        self._lock = Lock()
        self._frames: dict[int, dict] = {}
        self._statuses: dict[int, dict] = {}
        self._rgb_arrivals: deque[float] = deque(maxlen=32)
        self._last_fault: str | None = None
        self._last_sequence: int | None = None
        self._missing_frames = 0

    def update_image(self, kind: str, stamp_ns: int, message) -> None:
        try:
            array = image_array(message, kind)
            if kind == "rgb":
                jpeg, stats = encode_jpeg(array), {}
            else:
                preview, stats = depth_preview(array)
                jpeg = encode_jpeg(preview)
        except (ValueError, TypeError, RuntimeError) as exc:
            with self._lock:
                self._last_fault = str(exc)
            return
        now = time.monotonic()
        with self._lock:
            if self._frames and stamp_ns < max(self._frames) - 150_000_000:
                self._last_fault = "NONMONOTONE_IMAGE"
                return
            frame = self._frames.setdefault(stamp_ns, {"received_at": now})
            if stamp_ns in self._statuses and "status" not in frame:
                frame["status"] = self._statuses.pop(stamp_ns)
            if kind in frame:
                self._last_fault = "DUPLICATE_CAMERA_FRAME"
                return
            if ("width" in frame and (frame["width"], frame["height"]) !=
                    (int(message.width), int(message.height))):
                self._last_fault = "CAMERA_PAIR_DIMENSIONS_MISMATCH"
                self._frames.pop(stamp_ns)
                return
            image_frame = getattr(getattr(message, "header", None), "frame_id", "UNKNOWN")
            if "image_frame" in frame and frame["image_frame"] != image_frame:
                self._last_fault = "CAMERA_PAIR_FRAME_MISMATCH"
                self._frames.pop(stamp_ns)
                return
            frame[kind] = jpeg
            frame["image_frame"] = image_frame
            frame["width"], frame["height"] = int(message.width), int(message.height)
            frame["depth_stats"] = stats if kind == "depth" else frame.get("depth_stats")
            frame["received_at"] = now
            if "rgb" in frame and "depth" in frame:
                frame["paired_at"] = now
                # Vendor HP60C capture stamps use host monotonic time;
                # recorded/simulated ROS stamps may use wall time.
                wall_age_ns = time.time_ns() - stamp_ns
                monotonic_age_ns = time.monotonic_ns() - stamp_ns
                latency_ns = (wall_age_ns if 0 <= wall_age_ns <= 5_000_000_000
                              else monotonic_age_ns)
                frame["capture_to_gateway_ms"] = (latency_ns / 1e6
                                                   if 0 <= latency_ns <= 5_000_000_000 else None)
            if kind == "rgb":
                self._rgb_arrivals.append(now)
            for old in sorted(self._frames)[:-3]:
                del self._frames[old]
            self._last_fault = None

    def update_status(self, stamp_ns: int, payload: dict) -> None:
        if payload.get("sample_time_ns") != stamp_ns:
            with self._lock:
                self._last_fault = "CAMERA_STATUS_STAMP_MISMATCH"
            return
        with self._lock:
            frame = self._frames.get(stamp_ns)
            if (frame is not None and "status" in frame) or stamp_ns in self._statuses:
                return
            if frame is None:
                self._statuses[stamp_ns] = dict(payload)
            else:
                frame["status"] = dict(payload)
            sequence = payload.get("sequence")
            if isinstance(sequence, int) and sequence >= 0:
                if self._last_sequence is not None and sequence > self._last_sequence:
                    self._missing_frames += max(0, sequence - self._last_sequence - 1)
                elif self._last_sequence is not None and sequence < self._last_sequence:
                    self._missing_frames = 0
                self._last_sequence = sequence
            for old in list(self._statuses)[:-3]:
                del self._statuses[old]

    def snapshot(self) -> dict:
        now = time.monotonic()
        with self._lock:
            paired = [(stamp, frame) for stamp, frame in self._frames.items()
                      if "rgb" in frame and "depth" in frame]
            if not paired:
                return {"available": False, "frame_id": None,
                        "reason": self._last_fault or "CAMERA_PAIR_MISSING"}
            stamp, frame = max(paired)
            age_ms = (now - frame["paired_at"]) * 1000
            if age_ms > self.max_age_s * 1000:
                return {"available": False, "frame_id": None,
                        "reason": "CAMERA_PAIR_STALE", "age_ms": age_ms}
            status = frame.get("status") or {}
            arrivals = [item for item in self._rgb_arrivals if now - item <= 5]
            fps = ((len(arrivals) - 1) / (arrivals[-1] - arrivals[0])
                   if len(arrivals) >= 2 and arrivals[-1] > arrivals[0] else None)
            processing = status.get("processing_ms")
            if processing is not None and (not isinstance(processing, (int, float)) or
                                           not isfinite(processing) or processing < 0):
                processing = None
            return {"available": True, "frame_id": str(stamp),
                    "capture_time_ns": str(stamp), "age_ms": age_ms,
                    "width": frame["width"], "height": frame["height"],
                    "image_frame": frame["image_frame"],
                    "sequence": status.get("sequence"),
                    "capture_to_gateway_ms": frame.get("capture_to_gateway_ms"),
                    "missing_frames": self._missing_frames,
                    "source": status.get("source", "UNKNOWN"),
                    "valid": status.get("valid") is True,
                    "reason_codes": status.get("reason_codes", ["STATUS_MISSING"]),
                    "calibration_status": status.get("calibration_status", "UNKNOWN"),
                    "model_sha256": status.get("model_sha256", "UNKNOWN"),
                    "detections": status.get("detections", []),
                    "cone_details": status.get("cone_details", []),
                    "depth_stats": frame.get("depth_stats"),
                    "receive_fps": fps, "processing_ms": processing,
                    "depth_qualified": status.get("calibration_status") == "MOUNT_VERIFIED",
                    **{key: status.get(key) for key in PASSTHROUGH}}

    def image(self, frame_id: str, kind: str) -> bytes | None:
        if kind not in {"rgb", "depth"}:
            return None
        with self._lock:
            try:
                stamp = int(frame_id)
                frame = self._frames[stamp]
            except (ValueError, KeyError):
                return None
            if "paired_at" not in frame:
                return None
            if time.monotonic() - frame["paired_at"] > self.max_age_s:
                return None
            return frame.get(kind)
