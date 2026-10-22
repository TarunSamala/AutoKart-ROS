"""autokart_perception_adapter: YOLO26s + RGB/depth fusion -> qualified 3D cones.

Consumes RGB/depth/CameraInfo -- by default from autokart_hp60c's topics
(node 01 in this library), never opens a camera itself. Publishes
autokart_msgs/ConeArray on /perception/cones (qualified cones only) and a
status message on /perception/status.

Scope note: /perception/detections and /perception/obstacles from the master
architecture doc's node registry are NOT published by this node this session
(see CONTRACT.md "Deferred"). Cones and status are implemented and tested.
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from autokart_msgs.msg import Cone as RosCone, ConeArray, SensorDeliveryMetadata

from .backends import build_backend, PerceptionBackendError

LABELS = {"blue": 1, "yellow": 2, "orange": 3, "large_orange": 4}


def image_to_rgb_array(msg: Image) -> np.ndarray:
    if msg.encoding != "rgb8":
        raise ValueError(f"UNSUPPORTED_RGB_ENCODING:{msg.encoding}")
    return np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)


def image_to_depth_array(msg: Image) -> np.ndarray:
    if msg.encoding != "16UC1":
        raise ValueError(f"UNSUPPORTED_DEPTH_ENCODING:{msg.encoding}")
    return np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(msg.height, msg.width)


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class PerceptionAdapterNode(Node):
    def __init__(self, **node_kwargs):
        super().__init__("autokart_perception_adapter", **node_kwargs)
        self.declare_parameter("backend", "synthetic")
        self.declare_parameter("algorithm_package_path", "")
        self.declare_parameter("model_path", "")
        self.declare_parameter("model_manifest", "")
        self.declare_parameter("calibration_artifact", "")
        self.declare_parameter("input_mode", "topics")             # "topics" | "standalone_timer"
        self.declare_parameter("standalone_period_s", 0.1)
        self.declare_parameter("rgb_topic", "/hp60c/rgb/image_raw")
        self.declare_parameter("depth_topic", "/hp60c/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/hp60c/rgb/camera_info")
        self.declare_parameter("source_label", "LIVE")             # provenance hint for gating; see CONTRACT.md

        self.cones_pub = self.create_publisher(ConeArray, "/perception/cones", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(SensorDeliveryMetadata, "/perception/status", 10)

        self.sequence = 0
        self._last_fault = None

        backend_name = str(self.get_parameter("backend").value)
        try:
            self.backend = build_backend(
                backend_name,
                algorithm_package_path=str(self.get_parameter("algorithm_package_path").value),
                model_path=str(self.get_parameter("model_path").value) or None,
                model_manifest=str(self.get_parameter("model_manifest").value) or None,
                calibration_artifact=str(self.get_parameter("calibration_artifact").value) or None,
            )
            self.startup_error = None
        except PerceptionBackendError as exc:
            self.backend = None
            self.startup_error = str(exc)
            self.get_logger().error(f"autokart_perception_adapter startup failed: {self.startup_error}")

        self._subs = []
        self._timer = None
        mode = str(self.get_parameter("input_mode").value)
        if mode == "standalone_timer":
            self._timer = self.create_timer(
                float(self.get_parameter("standalone_period_s").value), self._standalone_tick)
        elif mode == "topics":
            self._setup_topic_sync()
        else:
            raise ValueError(f"unsupported input_mode: {mode}")

        if self.backend is None:
            # Keep reporting, not a one-shot: a misconfigured dependency must
            # stay visible for as long as the node runs (same lesson learned
            # and fixed in autokart_hp60c's reviewer pass).
            self._fault_timer = self.create_timer(1.0, lambda: self._publish_fault(self.startup_error))

    def _setup_topic_sync(self):
        rgb_sub = message_filters.Subscriber(
            self, Image, str(self.get_parameter("rgb_topic").value), qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(
            self, Image, str(self.get_parameter("depth_topic").value), qos_profile=qos_profile_sensor_data)
        info_sub = message_filters.Subscriber(
            self, CameraInfo, str(self.get_parameter("camera_info_topic").value), qos_profile=qos_profile_sensor_data)
        self._subs = [rgb_sub, depth_sub, info_sub]
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub], queue_size=5, slop=0.05)
        self._sync.registerCallback(self._on_synced_frame)

    def _on_synced_frame(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        if self.backend is None:
            return
        try:
            rgb = image_to_rgb_array(rgb_msg)
            depth = image_to_depth_array(depth_msg)
        except ValueError as exc:
            self._publish_fault(str(exc))
            return
        fx, cx, fy, cy = info_msg.k[0], info_msg.k[2], info_msg.k[4], info_msg.k[5]
        timestamp_ns = stamp_to_ns(rgb_msg.header.stamp)
        self._run_backend(rgb, depth, fx, fy, cx, cy, timestamp_ns,
                           str(self.get_parameter("source_label").value))

    def _standalone_tick(self):
        if self.backend is None:
            return
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        depth = np.full((480, 640), 2000, dtype=np.uint16)
        timestamp_ns = self.get_clock().now().nanoseconds
        self._run_backend(rgb, depth, 500.0, 500.0, 320.0, 240.0, timestamp_ns, "SYNTHETIC")

    def _run_backend(self, rgb, depth, fx, fy, cx, cy, timestamp_ns, source_hint):
        try:
            outcome = self.backend.process(rgb, depth, fx, fy, cx, cy, timestamp_ns, source_hint)
        except PerceptionBackendError as exc:
            self._publish_fault(str(exc))
            return
        except Exception as exc:  # unexpected backend fault must not kill this node silently
            self._publish_fault(f"UNEXPECTED_BACKEND_ERROR:{type(exc).__name__}:{exc}")
            return
        self.sequence += 1
        self._publish_status(quality="OK" if outcome.valid else "DIAGNOSTIC_ONLY",
                              fault_state="NONE" if outcome.valid else ",".join(outcome.reasons) or "UNQUALIFIED",
                              timestamp_ns=timestamp_ns)
        self._last_fault = None
        if outcome.valid:
            msg = ConeArray()
            msg.measurement_time.sec, msg.measurement_time.nanosec = divmod(timestamp_ns, 1_000_000_000)
            msg.delivery_time = self.get_clock().now().to_msg()
            msg.frame_id = "base_link"
            msg.ground_truth = outcome.source == "SYNTHETIC"
            for cone in outcome.cones:
                if cone.label not in LABELS:
                    continue
                rc = RosCone()
                rc.id = cone.id
                rc.semantic_class = LABELS[cone.label]
                rc.position.x, rc.position.y = cone.x_m, cone.y_m
                rc.confidence = cone.confidence
                rc.position_std_m = cone.sigma_m
                msg.cones.append(rc)
            self.cones_pub.publish(msg)

    def _publish_fault(self, reason: str):
        if reason != self._last_fault:
            self.get_logger().warning(f"autokart_perception_adapter fault: {reason}")
        self._last_fault = reason
        self._publish_status(quality="INVALID", fault_state=reason,
                              timestamp_ns=self.get_clock().now().nanoseconds)

    def _publish_status(self, quality: str, fault_state: str, timestamp_ns: int):
        msg = SensorDeliveryMetadata()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.sensor_id = "autokart_perception_adapter"
        msg.sequence = self.sequence
        msg.quality = quality
        msg.fault_state = fault_state
        msg.payload_present = quality == "OK"
        self.status_pub.publish(msg)

    def destroy_node(self):
        # Cancel timers before tearing down publishers/handles: a timer
        # callback still in flight (e.g. the 1s fault-report timer) calling
        # self.*_pub.publish(...) during destroy otherwise races rclpy's
        # handle teardown (observed as an unretrieved "cannot use Destroyable"
        # exception during this node's own clean-workspace test teardown).
        if self._timer is not None:
            self._timer.cancel()
        if getattr(self, "_fault_timer", None) is not None:
            self._fault_timer.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionAdapterNode()
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
