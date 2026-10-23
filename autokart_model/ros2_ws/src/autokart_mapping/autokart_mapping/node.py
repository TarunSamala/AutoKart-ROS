"""autokart_mapping: standalone cone-track mapping node (node 06 of the 12-node architecture).

Subscribes validated metric cone observations (``autokart_msgs/ConeArray``, ``base_link``) and pose
(``nav_msgs/Odometry``), publishes an observed cone map (``autokart_msgs/ConeArray``, ``odom``) plus a
diagnostic status (``autokart_msgs/MissionDebug``) carrying the Delaunay/Gabriel/gate/centerline
geometry and the observed skidpad ring topology. See CONTRACT.md. All outputs are diagnostic
candidates; ``vehicle_authorized`` is always false.
"""
from __future__ import annotations

import json
import time
from math import atan2, cos, sin, sqrt, pi

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from autokart_msgs.msg import Cone as RosCone, ConeArray, MissionDebug

from .algorithm import MappingAlgorithm, MappingAlgorithmError
from .engine import CLASSES, ConeObservation, MappingEngine


def ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def ros_time(value: int):
    from builtin_interfaces.msg import Time
    stamp = Time()
    stamp.sec, stamp.nanosec = divmod(int(value), 1_000_000_000)
    return stamp


def pose_from_odom(message: Odometry, pose_estimate_cls):
    """Build the reused ``shared_core.PoseEstimate`` from a ``nav_msgs/Odometry`` message. Same
    quaternion-to-yaw and covariance-to-sigma conversion as the existing, reused mapping adapter."""
    q = message.pose.pose.orientation
    siny = 2 * (q.w * q.z + q.x * q.y)
    cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
    covariance = message.pose.covariance
    uncertainty = sqrt(max(0.0, float(covariance[0]), float(covariance[7])))
    speed = message.twist.twist.linear
    return pose_estimate_cls(float(message.pose.pose.position.x),
                             float(message.pose.pose.position.y), atan2(siny, cosy),
                             sqrt(speed.x * speed.x + speed.y * speed.y),
                             ns(message.header.stamp), uncertainty,
                             frame_id=message.header.frame_id,
                             valid=message.child_frame_id == "base_link")


class MappingNode(Node):
    def __init__(self, **node_kwargs):
        super().__init__("autokart_mapping", **node_kwargs)
        self.declare_parameter("algorithm_package_path", "")
        self.declare_parameter("source_mode", "SIMULATED")
        self.declare_parameter("live_odom_verified", False)
        self.declare_parameter("cones_topic", "/perception/cones")
        self.declare_parameter("perception_status_topic", "/perception/diagnostics")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("map_topic", "/autokart_mapping/cones")
        self.declare_parameter("status_topic", "/autokart_mapping/status")
        self.declare_parameter("cone_input_max_age_s", 0.3)
        self.declare_parameter("input_timeout_s", 0.5)

        self.source_mode = str(self.get_parameter("source_mode").value)
        self.live_odom_verified = bool(self.get_parameter("live_odom_verified").value)
        if self.source_mode not in {"SIMULATED", "REPLAY", "LIVE"}:
            raise ValueError("unsupported mapping source_mode")
        self.cone_input_max_age_s = float(self.get_parameter("cone_input_max_age_s").value)
        self.input_timeout_s = float(self.get_parameter("input_timeout_s").value)

        self.algorithm = None
        self.engine = None
        self.startup_error = None
        try:
            self.algorithm = MappingAlgorithm(str(self.get_parameter("algorithm_package_path").value))
            self.engine = MappingEngine(self.algorithm)
        except MappingAlgorithmError as exc:
            self.startup_error = str(exc)
            self.get_logger().error(f"autokart_mapping startup failed: {self.startup_error}")

        self.perception_statuses = {}
        self.pending_cones = {}
        self.poses = {}
        self.odom_statuses = {}
        self.cycle = 0
        self.last_receive = None

        map_topic = str(self.get_parameter("map_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        self.map_pub = self.create_publisher(ConeArray, map_topic, qos_profile_sensor_data)
        self.status_pub = self.create_publisher(MissionDebug, status_topic, 10)
        self.canonical_pub = self.create_publisher(MissionDebug, "/mapping/status", 10)
        self.create_subscription(MissionDebug, str(self.get_parameter("perception_status_topic").value),
                                 self.on_perception_status, 10)
        self.create_subscription(String, "/vehicle/odometry_status", self.on_odom_status, 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value),
                                 self.on_odom, qos_profile_sensor_data)
        self.create_subscription(ConeArray, str(self.get_parameter("cones_topic").value),
                                 self.on_cones, qos_profile_sensor_data)
        self.create_timer(0.1, self.on_timer)

    def on_odom(self, message):
        if self.algorithm is None:
            return
        try:
            pose = pose_from_odom(message, self.algorithm.PoseEstimate)
            self.poses[pose.timestamp_ns] = pose
            if len(self.poses) > 256:
                del self.poses[min(self.poses)]
        except (ValueError, OverflowError):
            self.poses.clear()
            self.engine.clear()
        for stamp in sorted(self.pending_cones):
            self._maybe_process(stamp)

    def on_cones(self, message: ConeArray):
        if self.algorithm is None:
            self._publish_fault("BACKEND_UNAVAILABLE", ns(message.measurement_time))
            return
        stamp = ns(message.measurement_time)
        observations = [ConeObservation(id=int(item.id), semantic_class=int(item.semantic_class),
                                        x_m=float(item.position.x), y_m=float(item.position.y),
                                        confidence=float(item.confidence),
                                        sigma_m=float(item.position_std_m))
                        for item in message.cones]
        status = {"sample_time_ns": stamp, "valid": False, "source": self.source_mode,
                  "frame_id": message.frame_id, "ground_truth": bool(message.ground_truth)}
        self.pending_cones[stamp] = (observations, status, time.monotonic())
        for old in sorted(self.pending_cones)[:-16]:
            del self.pending_cones[old]
        self._maybe_process(stamp)

    def on_perception_status(self, message):
        try:
            payload = json.loads(message.payload_json)
            stamp = ns(message.measurement_time)
            if not isinstance(payload, dict) or payload.get("schema_version") != 1 or payload.get("sample_time_ns") != stamp:
                raise ValueError("PERCEPTION_STATUS_MISMATCH")
            self.perception_statuses[stamp] = payload
            for old in sorted(self.perception_statuses)[:-16]:
                del self.perception_statuses[old]
            self._maybe_process(stamp)
        except (ValueError, TypeError):
            self._publish_fault("PERCEPTION_STATUS_MALFORMED", self.get_clock().now().nanoseconds)

    def on_odom_status(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict) or payload.get("schema_version") != 1 or type(payload.get("sample_time_ns")) is not int:
                raise ValueError()
            self.odom_statuses[payload["sample_time_ns"]] = payload
            for old in sorted(self.odom_statuses)[:-32]:
                del self.odom_statuses[old]
            for stamp in sorted(self.pending_cones):
                self._maybe_process(stamp)
        except (ValueError, TypeError):
            self.odom_statuses.clear()

    def _pose_qualified(self, pose):
        status = self.odom_statuses.get(pose.timestamp_ns, {})
        source = {"SYNTHETIC":"SIMULATED", "LIVE_PHYSICAL":"LIVE"}.get(status.get("source"),status.get("source"))
        now = self.get_clock().now().nanoseconds
        return (status.get("valid") is True and status.get("continuity_lost") is False and
                source == self.source_mode and type(status.get("expires_ns")) is int and
                status["expires_ns"] > now and 0 <= now-pose.timestamp_ns <= 250_000_000 and
                (source != "LIVE" or status.get("physical_qualified") is True))

    def pose_at_capture(self, stamp):
        if stamp in self.poses:
            return self.poses[stamp] if self._pose_qualified(self.poses[stamp]) else None
        before = [x for x in self.poses if 0 < stamp-x <= 150_000_000]
        after = [x for x in self.poses if 0 < x-stamp <= 150_000_000]
        if not before or not after:
            return None
        a,b = self.poses[max(before)],self.poses[min(after)]
        if not self._pose_qualified(a) or not self._pose_qualified(b) or not a.valid or not b.valid or a.frame_id != b.frame_id:
            return None
        t=(stamp-a.timestamp_ns)/(b.timestamp_ns-a.timestamp_ns)
        yaw_delta=(b.yaw_rad-a.yaw_rad+pi)%(2*pi)-pi
        if abs(yaw_delta)>pi/2:
            return None
        return self.algorithm.PoseEstimate(a.x_m+(b.x_m-a.x_m)*t,a.y_m+(b.y_m-a.y_m)*t,
            a.yaw_rad+yaw_delta*t,a.speed_mps+(b.speed_mps-a.speed_mps)*t,stamp,
            max(a.position_std_m,b.position_std_m),frame_id=a.frame_id,valid=True)

    def _maybe_process(self, stamp):
        pending = self.pending_cones.get(stamp)
        pose = self.pose_at_capture(stamp)
        companion = self.perception_statuses.get(stamp)
        if pending is None or pose is None or companion is None:
            return
        observations, status, _ = self.pending_cones.pop(stamp)
        self.perception_statuses.pop(stamp, None)
        actual_source = {"SYNTHETIC":"SIMULATED", "LIVE_PHYSICAL":"LIVE"}.get(companion.get("source"),companion.get("source"))
        now_ns = self.get_clock().now().nanoseconds
        status["valid"] = (0 <= now_ns - stamp <= 250_000_000 and
                           type(companion.get("expires_ns")) is int and companion["expires_ns"] > now_ns and
                           companion.get("valid") is True and actual_source == self.source_mode and
                           companion.get("frame_id") == status["frame_id"] and
                           (not status["ground_truth"] or self.source_mode == "SIMULATED"))
        status["source"] = actual_source
        status["synthetic_fixture_validated"] = status["valid"] and self.source_mode == "SIMULATED"
        if self.source_mode == "LIVE" and not self.live_odom_verified:
            self.engine.clear()
            result = {"valid": False, "source": status.get("source", "UNKNOWN"),
                      "sample_time_ns": stamp, "reason_codes": ["LIVE_ODOM_UNVERIFIED"],
                      "tracked_cones": [], "geometry": {}}
        elif self.source_mode == "LIVE" and pose.position_std_m <= 0:
            self.engine.clear()
            result = {"valid": False, "source": status.get("source", "UNKNOWN"),
                      "sample_time_ns": stamp, "reason_codes": ["LIVE_ODOM_COVARIANCE_UNQUALIFIED"],
                      "tracked_cones": [], "geometry": {}}
        else:
            result = self.engine.update(observations, status, pose)
        self.last_receive = time.monotonic()
        self.publish(result)

    def on_timer(self):
        expired = [stamp for stamp, (_, _, received) in self.pending_cones.items()
                   if time.monotonic() - received > self.cone_input_max_age_s]
        for stamp in expired:
            self.pending_cones.pop(stamp, None)
            if self.engine is not None:
                self.engine.clear()
            self.publish({"valid": False, "source": "UNKNOWN", "sample_time_ns": stamp,
                          "reason_codes": ["STATUS_OR_ODOM_TIMEOUT"],
                          "tracked_cones": [], "geometry": {}})
        if (self.last_receive is not None and
                time.monotonic() - self.last_receive > self.input_timeout_s):
            self.last_receive = None
            if self.engine is not None:
                self.engine.clear()
            self.publish({"valid": False, "source": "UNKNOWN",
                          "sample_time_ns": self.get_clock().now().nanoseconds,
                          "reason_codes": ["MAP_INPUT_TIMEOUT"],
                          "tracked_cones": [], "geometry": {}})
        if self.algorithm is None:
            self._publish_fault("BACKEND_UNAVAILABLE", self.get_clock().now().nanoseconds)

    def _publish_fault(self, reason: str, stamp: int):
        self.publish({"valid": False, "source": "UNKNOWN", "sample_time_ns": stamp,
                      "reason_codes": [self.startup_error or reason],
                      "tracked_cones": [], "geometry": {}})

    def publish(self, result):
        self.cycle += 1
        stamp = result["sample_time_ns"]
        map_message = ConeArray()
        map_message.measurement_time = ros_time(max(0, stamp))
        map_message.delivery_time = ros_time(self.get_clock().now().nanoseconds)
        map_message.frame_id = "odom"
        map_message.ground_truth = result["source"] in {"SIMULATED", "SYNTHETIC"}
        if result["valid"]:
            for item in result["tracked_cones"]:
                cone = RosCone()
                cone.id = int(item["id"])
                cone.semantic_class = CLASSES[item["label"]]
                cone.position.x = float(item["x_m"])
                cone.position.y = float(item["y_m"])
                cone.confidence = float(item["confidence"])
                cone.position_std_m = float(item["sigma_m"])
                map_message.cones.append(cone)
        self.map_pub.publish(map_message)
        payload = {"schema_version": 1, "source": result["source"],
                   "processed_at_monotonic_ns": time.monotonic_ns(),
                   "valid": result["valid"], "reason_codes": result["reason_codes"],
                   "frame_id": "odom", "sample_time_ns": stamp,
                   "expires_ns": stamp + 150_000_000, "sequence": self.cycle,
                   "tracked_count": len(map_message.cones),
                   "pose_time_ns":result.get("pose_time_ns"),
                   "capture_pose":result.get("capture_pose"),
                   "geometry": result["geometry"], "vehicle_authorized": False,
                   "published_to_vcu": False}
        status_message = MissionDebug()
        status_message.measurement_time = ros_time(max(0, stamp))
        status_message.cycle = self.cycle
        status_message.frame_id = "odom"
        status_message.stage = "MAPPING"
        status_message.payload_json = json.dumps(payload, sort_keys=True, allow_nan=False)
        self.status_pub.publish(status_message)
        self.canonical_pub.publish(status_message)


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
