"""autokart_hp60c: sole HP60C camera SDK owner.

Publishes stamped RGB, depth and CameraInfo for both streams plus a delivery
status message. Never opens the SDK from any other process. See CONTRACT.md.
"""
from __future__ import annotations

import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from autokart_msgs.msg import SensorDeliveryMetadata

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
    msg.height, msg.width = frame.rgb.shape[0], frame.rgb.shape[1]
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

        self.rgb_frame_id = str(self.get_parameter("rgb_frame_id").value)
        self.depth_frame_id = str(self.get_parameter("depth_frame_id").value)

        self.rgb_pub = self.create_publisher(Image, "/hp60c/rgb/image_raw", qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, "/hp60c/depth/image_raw", qos_profile_sensor_data)
        self.rgb_info_pub = self.create_publisher(CameraInfo, "/hp60c/rgb/camera_info", qos_profile_sensor_data)
        self.depth_info_pub = self.create_publisher(CameraInfo, "/hp60c/depth/camera_info", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(SensorDeliveryMetadata, "/hp60c/status", 10)

        self.sequence = 0
        self.cumulative_captured = 0
        self.cumulative_dropped = 0
        self._last_fault = None
        self._stop = threading.Event()

        backend_name = str(self.get_parameter("backend").value)
        try:
            self.backend = build_backend(
                backend_name,
                algorithm_package_path=str(self.get_parameter("algorithm_package_path").value),
                library=str(self.get_parameter("hp60c_library_path").value) or None,
                configuration=str(self.get_parameter("hp60c_configuration_path").value) or None,
                max_age_ms=int(self.get_parameter("max_age_ms").value),
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
        try:
            self._capture_loop_body()
        except Exception:
            # On SIGINT rclpy invalidates the context before destroy_node() can
            # set _stop, so an in-flight publish raises. That is shutdown, not a
            # fault; anything raised while the context is still valid is real.
            if self.context.ok():
                raise

    def _capture_loop_body(self):
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
            self.sequence += 1
            self.cumulative_captured += 1
            stamp = self.get_clock().now().to_msg()
            self.rgb_pub.publish(frame_to_rgb_image(frame, self.rgb_frame_id, stamp))
            self.depth_pub.publish(frame_to_depth_image(frame, self.depth_frame_id, stamp))
            self.rgb_info_pub.publish(intrinsics_to_camera_info(frame, self.rgb_frame_id, stamp))
            self.depth_info_pub.publish(intrinsics_to_camera_info(frame, self.depth_frame_id, stamp))
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
        self.status_pub.publish(msg)

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
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HP60CNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # SIGINT may already have shut the context down; a second shutdown raises RCLError.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
