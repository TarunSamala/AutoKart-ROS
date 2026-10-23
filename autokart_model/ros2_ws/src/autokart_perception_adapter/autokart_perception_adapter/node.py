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

import json
import time
from math import cos,sin,isfinite
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from autokart_msgs.msg import Cone as RosCone, ConeArray, SensorDeliveryMetadata, MissionDebug

from .backends import build_backend, PerceptionBackendError

LABELS = {"blue": 1, "yellow": 2, "orange": 3, "large_orange": 4}


def image_to_rgb_array(msg: Image) -> np.ndarray:
    if msg.encoding != "rgb8" or msg.width <= 0 or msg.height <= 0 or msg.step < msg.width*3:
        raise ValueError("UNSUPPORTED_RGB_LAYOUT")
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if raw.size != msg.height*msg.step:
        raise ValueError("RGB_PAYLOAD_SIZE_INVALID")
    return raw.reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3).copy()


def image_to_depth_array(msg: Image) -> np.ndarray:
    if msg.encoding != "16UC1" or msg.width <= 0 or msg.height <= 0 or msg.step < msg.width*2 or msg.step%2:
        raise ValueError("UNSUPPORTED_DEPTH_LAYOUT")
    raw = np.frombuffer(bytes(msg.data), dtype=np.dtype('>u2' if msg.is_bigendian else '<u2'))
    if raw.size*2 != msg.height*msg.step:
        raise ValueError("DEPTH_PAYLOAD_SIZE_INVALID")
    return raw.reshape(msg.height,msg.step//2)[:,:msg.width].astype(np.uint16)


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
        self.declare_parameter("obstacle_config_path", "")
        self.declare_parameter("input_mode", "topics")             # "topics" | "standalone_timer"
        self.declare_parameter("standalone_period_s", 0.1)
        self.declare_parameter("rgb_topic", "/hp60c/rgb/image_raw")
        self.declare_parameter("depth_topic", "/hp60c/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/hp60c/rgb/camera_info")
        self.declare_parameter("source_label", "LIVE")             # provenance hint for gating; see CONTRACT.md

        self.cones_pub = self.create_publisher(ConeArray, "/perception/cones", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(SensorDeliveryMetadata, "/perception/status", 10)

        self.diagnostics_pub = self.create_publisher(MissionDebug, "/perception/diagnostics", 10)
        self.detections_pub = self.create_publisher(MissionDebug, "/perception/detections", 10)
        self.obstacles_pub = self.create_publisher(ConeArray, "/perception/obstacles", qos_profile_sensor_data)
        self.obstacle_status_pub = self.create_publisher(MissionDebug, "/perception/obstacle_status_base", 10)
        self.projected_obstacles_pub = self.create_publisher(ConeArray, "/autokart_planner/obstacles", qos_profile_sensor_data)
        self.projected_coverage_pub = self.create_publisher(MissionDebug, "/perception/obstacle_status", 10)
        self.pending_obstacles = {}
        self.camera_metadata = {}
        self.pending_frames = {}
        self.last_frame_ns = -1
        self.last_input = time.monotonic()
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
                obstacle_config_path=str(self.get_parameter("obstacle_config_path").value) or None,
            )
            self.startup_error = None
        except PerceptionBackendError as exc:
            self.backend = None
            self.startup_error = str(exc)
            self.get_logger().error(f"autokart_perception_adapter startup failed: {self.startup_error}")

        self.create_subscription(MissionDebug, "/mapping/status", self._on_mapping_pose, 10)
        self.create_subscription(MissionDebug, "/hp60c/diagnostics", self._on_camera_metadata, 10)
        self._watchdog = self.create_timer(0.1, self._check_input)
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
            [rgb_sub, depth_sub, info_sub], queue_size=1, slop=0.05)
        self._sync.registerCallback(self._on_synced_frame)

    def _on_mapping_pose(self,message):
        try:
            mapped=json.loads(message.payload_json)
            if not isinstance(mapped,dict): return
            stamp=mapped.get("sample_time_ns")
            pending=self.pending_obstacles.pop(stamp,None)
            if pending is None: return
            obstacles,coverage=pending
            pose=mapped.get("capture_pose")
            now=self.get_clock().now().nanoseconds
            if (mapped.get("valid") is not True or mapped.get("source") != coverage["source"] or
                    not isinstance(pose,dict) or pose.get("sample_time_ns") != stamp or
                    type(mapped.get("expires_ns")) is not int or mapped["expires_ns"] <= now or
                    not 0 <= now-stamp <= 250_000_000 or
                    not all(type(pose.get(k)) in (int,float) and isfinite(pose[k]) for k in ("x_m","y_m","yaw_rad"))):
                return
            c,s=cos(pose["yaw_rad"]),sin(pose["yaw_rad"])
            for item in obstacles.cones:
                x,y=item.position.x,item.position.y
                item.position.x=pose["x_m"]+c*x-s*y
                item.position.y=pose["y_m"]+s*x+c*y
            obstacles.frame_id="odom"
            self.projected_obstacles_pub.publish(obstacles)
            coverage={**coverage,"frame_id":"odom","expires_ns":min(coverage["expires_ns"],mapped["expires_ns"])}
            self._publish_debug(self.projected_coverage_pub,coverage,"OBSTACLES")
        except (ValueError,TypeError,KeyError,OverflowError):
            return

    def _check_input(self):
        if self.backend is None:
            return
        if time.monotonic() - self.last_input > 0.5:
            self.pending_frames.clear()
            self._publish_fault("CAMERA_INPUT_TIMEOUT")

    def _on_camera_metadata(self, message):
        try:
            payload = json.loads(message.payload_json)
            stamp = stamp_to_ns(message.measurement_time)
            if not isinstance(payload, dict) or payload.get("schema_version") != 1 or payload.get("sample_time_ns") != stamp:
                raise ValueError("CAMERA_METADATA_INVALID")
            self.camera_metadata[stamp] = payload
            for old in sorted(self.camera_metadata)[:-5]:
                del self.camera_metadata[old]
            self._try_frame(stamp)
        except (ValueError, TypeError):
            self._publish_fault("CAMERA_METADATA_MALFORMED")

    def _on_synced_frame(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        stamp = stamp_to_ns(rgb_msg.header.stamp)
        self.pending_frames[stamp] = (rgb_msg, depth_msg, info_msg)
        for old in sorted(self.pending_frames)[:-3]:
            del self.pending_frames[old]
        self._try_frame(stamp)

    def _try_frame(self, stamp):
        metadata = self.camera_metadata.get(stamp)
        if metadata is None or stamp not in self.pending_frames:
            return
        rgb_msg, depth_msg, info_msg = self.pending_frames.pop(stamp)
        self.camera_metadata.pop(stamp, None)
        if stamp <= self.last_frame_ns or not 0 <= self.get_clock().now().nanoseconds - stamp <= 250_000_000:
            self._publish_fault("STALE_OR_NONMONOTONE_CAMERA_FRAME")
            return
        self.last_frame_ns = stamp
        self.last_input = time.monotonic()
        if (metadata.get("aligned") is not True or
                type(metadata.get("depth_scale_m_per_unit")) not in (float,int) or
                not np.isfinite(metadata["depth_scale_m_per_unit"]) or metadata["depth_scale_m_per_unit"] <= 0):
            self._publish_fault("DEPTH_REGISTRATION_OR_SCALE_UNAVAILABLE")
            return
        if hasattr(self.backend, "depth_scale"):
            self.backend.depth_scale = metadata["depth_scale_m_per_unit"]
            self.backend.aligned = metadata["aligned"]
            self.backend.source_timestamp_ns = metadata.get("source_monotonic_ns", stamp)
            self.backend.capture_sequence = metadata.get("sequence", 0)
        self._process_synced_frame(rgb_msg, depth_msg, info_msg, metadata.get("source", "UNVERIFIED"))

    def _process_synced_frame(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo, source_hint):
        if self.backend is None:
            return
        try:
            stamps = {stamp_to_ns(m.header.stamp) for m in (rgb_msg, depth_msg, info_msg)}
            if len(stamps) != 1 or info_msg.header.frame_id != rgb_msg.header.frame_id:
                raise ValueError("RGB_DEPTH_INFO_TIMESTAMP_OR_FRAME_MISMATCH")
            if (rgb_msg.height != depth_msg.height or rgb_msg.width != depth_msg.width or
                    info_msg.height != rgb_msg.height or info_msg.width != rgb_msg.width or
                    not all(np.isfinite(v) for v in info_msg.k) or info_msg.k[0] <= 0 or info_msg.k[4] <= 0 or
                    not 0 <= info_msg.k[2] < info_msg.width or not 0 <= info_msg.k[5] < info_msg.height):
                raise ValueError("CAMERA_INFO_OR_REGISTERED_DIMENSIONS_INVALID")
            rgb = image_to_rgb_array(rgb_msg)
            depth = image_to_depth_array(depth_msg)
        except ValueError as exc:
            self._publish_fault(str(exc))
            return
        fx, cx, fy, cy = info_msg.k[0], info_msg.k[2], info_msg.k[4], info_msg.k[5]
        timestamp_ns = stamp_to_ns(rgb_msg.header.stamp)
        self._run_backend(rgb, depth, fx, fy, cx, cy, timestamp_ns,
                           source_hint)

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
        self.last_input = time.monotonic()
        source = {"SYNTHETIC":"SIMULATED", "LIVE_PHYSICAL":"LIVE"}.get(outcome.source, outcome.source)
        payload = {"schema_version":1, "source":source, "valid":bool(outcome.valid),
                   "sample_time_ns":timestamp_ns, "expires_ns":timestamp_ns+250_000_000,
                   "sequence":self.sequence, "frame_id":"base_link", "reason_codes":list(outcome.reasons),
                   "model_sha256":outcome.model_sha256, "inference_ms":outcome.processing_ms,
                   "detections":list(outcome.detections), "diagnostics":outcome.diagnostics,
                   "obstacle_coverage_valid":outcome.obstacle_coverage_valid,
                   "vehicle_authorized":False}
        self._publish_debug(self.diagnostics_pub, payload, "PERCEPTION")
        self._publish_debug(self.detections_pub, payload, "DETECTIONS")
        obstacles = ConeArray()
        obstacles.measurement_time.sec, obstacles.measurement_time.nanosec = divmod(timestamp_ns, 1_000_000_000)
        obstacles.delivery_time = self.get_clock().now().to_msg()
        obstacles.frame_id = "base_link"
        obstacles.ground_truth = source == "SIMULATED"
        for index,item in enumerate(outcome.obstacles):
            obstacle=RosCone()
            obstacle.id=index
            obstacle.semantic_class=0
            obstacle.position.x,obstacle.position.y=float(item["x_m"]),float(item["y_m"])
            obstacle.position_std_m=float(item["radius_m"])
            obstacle.confidence=1.0
            obstacles.cones.append(obstacle)
        self.obstacles_pub.publish(obstacles)
        coverage={**payload,"valid":outcome.obstacle_coverage_valid,"coverage_complete":outcome.obstacle_coverage_valid}
        self.pending_obstacles[timestamp_ns]=(obstacles,coverage)
        for old in sorted(self.pending_obstacles)[:-5]: del self.pending_obstacles[old]
        self._publish_debug(self.obstacle_status_pub,
            {**payload, "valid":outcome.obstacle_coverage_valid, "coverage_complete":outcome.obstacle_coverage_valid,
             "reason_codes":[] if outcome.obstacle_coverage_valid else ["ARBITRARY_OBSTACLE_COVERAGE_UNAVAILABLE"]}, "OBSTACLES")
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

    def _publish_debug(self, publisher, payload, stage):
        msg = MissionDebug()
        msg.measurement_time.sec, msg.measurement_time.nanosec = divmod(payload["sample_time_ns"],1_000_000_000)
        msg.cycle, msg.frame_id, msg.stage = self.sequence, payload.get("frame_id","base_link"), stage
        msg.payload_json = json.dumps(payload, allow_nan=False,
            default=lambda x: x.item() if hasattr(x,"item") else x.tolist())
        publisher.publish(msg)

    def _publish_fault(self, reason: str):
        if reason != self._last_fault:
            self.get_logger().warning(f"autokart_perception_adapter fault: {reason}")
        self._last_fault = reason
        stamp = self.get_clock().now().nanoseconds
        self._publish_debug(self.diagnostics_pub, {"schema_version":1,"source":"UNVERIFIED", "valid":False,
            "sample_time_ns":stamp,"expires_ns":stamp,"reason_codes":[reason],"vehicle_authorized":False}, "PERCEPTION")
        self._publish_status(quality="INVALID", fault_state=reason,
                              timestamp_ns=self.get_clock().now().nanoseconds)

    def _publish_status(self, quality: str, fault_state: str, timestamp_ns: int):
        msg = SensorDeliveryMetadata()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.sensor_id = "autokart_perception_adapter"
        msg.capture_time.sec, msg.capture_time.nanosec = divmod(timestamp_ns, 1_000_000_000)
        msg.processing_completion_time = msg.header.stamp
        msg.delivery_time = msg.header.stamp
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
        self._watchdog.cancel()
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
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
