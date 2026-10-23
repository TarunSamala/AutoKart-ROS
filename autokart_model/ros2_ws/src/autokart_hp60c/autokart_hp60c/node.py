"""autokart_hp60c: sole HP60C camera SDK owner.

Publishes stamped RGB, depth and CameraInfo for both streams plus a delivery
status message. Never opens the SDK from any other process. See CONTRACT.md.
"""
from __future__ import annotations

import threading
import json
import time
import fcntl
import tempfile
from pathlib import Path

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from autokart_msgs.msg import SensorDeliveryMetadata, MissionDebug

from .backends import build_backend, CameraBackendError, Frame


def frame_to_rgb_image(frame: Frame, frame_id: str, stamp) -> Image:
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = frame.rgb.shape[0], frame.rgb.shape[1]
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = frame.rgb.tobytes()
    return msg


def frame_to_depth_image(frame: Frame, frame_id: str, stamp) -> Image:
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = frame.depth.shape[0], frame.depth.shape[1]
    msg.encoding = "16UC1"
    msg.is_bigendian = 0
    msg.step = msg.width * 2
    msg.data = frame.depth.tobytes()
    return msg


def intrinsics_to_camera_info(frame: Frame, frame_id: str, stamp) -> CameraInfo:
    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = frame.depth.shape[0], frame.depth.shape[1]
    if not frame.aligned:
        msg.k = [0.0]*9
        msg.p = [0.0]*12
        return msg
    fx, fy, cx, cy = frame.intrinsics.fx, frame.intrinsics.fy, frame.intrinsics.cx, frame.intrinsics.cy
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


class HP60CNode(Node):
    def __init__(self, start_capture_thread: bool = True, **node_kwargs):
        super().__init__("autokart_hp60c", **node_kwargs)
        self.declare_parameter("backend", "synthetic")
        self.declare_parameter("algorithm_package_path", "")
        self.declare_parameter("hp60c_library_path", "")
        self.declare_parameter("hp60c_configuration_path", "")
        self.declare_parameter("max_age_ms", 250)
        self.declare_parameter("rgb_frame_id", "hp60c_rgb_optical_frame")
        self.declare_parameter("depth_frame_id", "hp60c_depth_optical_frame")
        self.declare_parameter("depth_scale_m_per_unit", 0.001)
        self.declare_parameter("synthetic_width", 640)
        self.declare_parameter("synthetic_height", 480)
        self.declare_parameter("synthetic_period_s", 0.08)

        self.rgb_frame_id = str(self.get_parameter("rgb_frame_id").value)
        self.depth_frame_id = str(self.get_parameter("depth_frame_id").value)

        self.rgb_pub = self.create_publisher(Image, "/hp60c/rgb/image_raw", qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, "/hp60c/depth/image_raw", qos_profile_sensor_data)
        self.rgb_info_pub = self.create_publisher(CameraInfo, "/hp60c/rgb/camera_info", qos_profile_sensor_data)
        self.depth_info_pub = self.create_publisher(CameraInfo, "/hp60c/depth/camera_info", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(SensorDeliveryMetadata, "/hp60c/status", 10)

        self.diagnostics_pub = self.create_publisher(MissionDebug, "/hp60c/diagnostics", 10)
        self.last_frame_stamp = None
        self.last_source_stamp = -1
        self._owner_lock = None
        self.sequence = 0
        self.cumulative_captured = 0
        self.cumulative_dropped = 0
        self._last_fault = None
        self._stop = threading.Event()

        backend_name = str(self.get_parameter("backend").value)
        try:
            if backend_name == "hp60c":
                self._owner_lock = open(Path(tempfile.gettempdir()) / "autokart-hp60c-sdk.lock", "a")
                try:
                    fcntl.flock(self._owner_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError as exc:
                    raise CameraBackendError("CAMERA_OWNER_ALREADY_ACTIVE") from exc
            self.backend = build_backend(
                backend_name,
                algorithm_package_path=str(self.get_parameter("algorithm_package_path").value),
                library=str(self.get_parameter("hp60c_library_path").value) or None,
                configuration=str(self.get_parameter("hp60c_configuration_path").value) or None,
                max_age_ms=int(self.get_parameter("max_age_ms").value),
                width=int(self.get_parameter("synthetic_width").value),
                height=int(self.get_parameter("synthetic_height").value),
                period_s=float(self.get_parameter("synthetic_period_s").value),
            )
            self.backend.start()
            self.startup_error = None
        except CameraBackendError as exc:
            self.backend = None
            self.startup_error = str(exc)
            self.get_logger().error(f"autokart_hp60c startup failed: {self.startup_error}")

        self._thread = None
        if start_capture_thread:
            self.start_capture()

    def start_capture(self):
        """Starts the background capture loop. Split out from __init__ so tests
        can construct the node, swap in a fault-injecting backend, then start
        capture deterministically instead of racing the constructor's own
        default backend."""
        self._thread = threading.Thread(target=self._capture_loop, name="hp60c_capture", daemon=True)
        self._thread.start()

    def _capture_loop(self):
        if self.backend is None:
            # Keep reporting the fault for as long as the node runs, rather
            # than publishing one status message and then going silent forever
            # (a silent, otherwise-alive node is indistinguishable from a slow
            # one to anything watching only process/node liveness).
            while not self._stop.is_set():
                self._publish_fault(self.startup_error or "BACKEND_UNAVAILABLE")
                self._stop.wait(1.0)
            return
        while not self._stop.is_set():
            try:
                frame = self.backend.read()
            except CameraBackendError as exc:
                self.cumulative_dropped += 1
                self._publish_fault(str(exc))
                self._stop.wait(0.1)
                continue
            except Exception as exc:  # noqa: BLE001 - deliberate: an unexpected
                # fault here (e.g. a backend bug) must surface as a reported
                # fault, never kill this thread silently while the node looks
                # alive and /hp60c/status looks frozen-but-fine.
                self.cumulative_dropped += 1
                self._publish_fault(f"UNEXPECTED_BACKEND_ERROR:{type(exc).__name__}:{exc}")
                self._stop.wait(0.1)
                continue
            try:
                import numpy as np
                if (frame.rgb.dtype != np.uint8 or frame.depth.dtype != np.uint16 or
                        frame.rgb.ndim != 3 or frame.rgb.shape[2] != 3 or
                        frame.depth.shape != frame.rgb.shape[:2] or not frame.intrinsics.valid()):
                    raise CameraBackendError("FRAME_FORMAT_OR_INTRINSICS_INVALID")
                age_ns = time.monotonic_ns() - frame.timestamp_ns
                if frame.timestamp_ns <= self.last_source_stamp or not 0 <= age_ns <= int(self.get_parameter("max_age_ms").value)*1_000_000:
                    raise CameraBackendError("STALE_OR_NONMONOTONE_FRAME")
            except CameraBackendError as exc:
                self.cumulative_dropped += 1
                self._publish_fault(str(exc))
                self._stop.wait(0.1)
                continue
            self.last_source_stamp = frame.timestamp_ns
            self.sequence += 1
            self.cumulative_captured += 1
            from builtin_interfaces.msg import Time
            stamp = Time()
            capture_ns = self.get_clock().now().nanoseconds - age_ns
            stamp.sec, stamp.nanosec = divmod(capture_ns, 1_000_000_000)
            self.last_frame_stamp = stamp
            try:
                self.rgb_pub.publish(frame_to_rgb_image(frame, self.rgb_frame_id, stamp))
                self.depth_pub.publish(frame_to_depth_image(frame, self.depth_frame_id, stamp))
                self.rgb_info_pub.publish(intrinsics_to_camera_info(frame, self.rgb_frame_id, stamp))
                self.depth_info_pub.publish(intrinsics_to_camera_info(frame, self.depth_frame_id, stamp))
            except RCLError:
                if self._stop.is_set() or not rclpy.ok():
                    break
                raise
            diagnostic = MissionDebug()
            diagnostic.measurement_time = stamp
            diagnostic.cycle = self.sequence
            diagnostic.frame_id = self.rgb_frame_id
            diagnostic.stage = "CAMERA"
            diagnostic.payload_json = json.dumps({"schema_version":1, "source":frame.source,
                "sample_time_ns":capture_ns, "source_monotonic_ns":frame.timestamp_ns,
                "rgb_source_timestamp_ms":frame.rgb_timestamp, "depth_source_timestamp_ms":frame.depth_timestamp,
                "sequence":self.sequence, "depth_scale_m_per_unit":frame.depth_scale,
                "aligned":bool(frame.aligned), "calibration":frame.calibration,
                "frame_age_ms":age_ns/1e6, "vehicle_authorized":False}, allow_nan=False)
            try:
                self.diagnostics_pub.publish(diagnostic)
            except RCLError:
                if self._stop.is_set() or not rclpy.ok():
                    break
                raise
            self._publish_status(quality="OK", fault_state="NONE")
            self._last_fault = None

    def _publish_fault(self, reason: str):
        if reason != self._last_fault:
            self.get_logger().warning(f"autokart_hp60c fault: {reason}")
        self._last_fault = reason
        self._publish_status(quality="INVALID", fault_state=reason)

    def _publish_status(self, quality: str, fault_state: str):
        msg = SensorDeliveryMetadata()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.rgb_frame_id
        msg.sensor_id = "hp60c"
        msg.capture_time = self.last_frame_stamp or msg.header.stamp
        msg.processing_completion_time = msg.header.stamp
        msg.delivery_time = msg.header.stamp
        msg.sequence = self.sequence
        msg.quality = quality
        msg.fault_state = fault_state
        msg.payload_present = quality == "OK"
        msg.cumulative_captured = self.cumulative_captured
        # Every captured frame is published synchronously with no queue, so
        # delivered == captured here (unlike a node with a queue, where the
        # two can diverge). Not a conditional: there is nothing to branch on.
        msg.cumulative_delivered = self.cumulative_captured
        msg.cumulative_dropped = self.cumulative_dropped
        try:
            self.status_pub.publish(msg)
        except RCLError:
            if not (self._stop.is_set() or not rclpy.ok()):
                raise

    def destroy_node(self):
        self._stop.set()
        if self._thread is not None:
            # Join before tearing down publishers/handles: the capture thread
            # calls self.*_pub.publish(...) and would otherwise race destroy
            # and hit rclpy's "Destroyable" teardown error.
            self._thread.join(timeout=2.0)
        if self.backend is not None:
            try:
                self.backend.close()
            except Exception:
                pass
        if self._owner_lock is not None:
            self._owner_lock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HP60CNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
