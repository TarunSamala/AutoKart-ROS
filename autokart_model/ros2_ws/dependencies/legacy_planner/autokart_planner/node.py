"""ROS transport for read-only RC-05 mission and path candidates."""

from __future__ import annotations

import json
from math import atan2, cos, sin
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from autokart_msgs.msg import ActuatorFeedback, ConeArray, MissionDebug

from autokart_perception.shared_core import BrakeCapability
from autokart_vehicle_geometry.geometry import GEOMETRY
from autokart_mapping.node import ns, pose_from_odom, ros_time
from .mission import PlannerEngine


class PlannerNode(Node):
    def __init__(self):
        super().__init__("autokart_planner")
        self.declare_parameter("mission", "SKIDPAD")
        self.declare_parameter("source", "SIMULATED")
        self.declare_parameter("brake_min_mps2", 0.0)
        self.declare_parameter("brake_delay_s", 0.0)
        self.declare_parameter("t_latency", -1.0)  # measured p95 + 50 ms overrides legacy brake_delay_s
        self.declare_parameter("brake_provenance", "UNKNOWN")
        # Commissioning default: 1.0 m, well short of the BrakeCapability dataclass default (1.5 m). The
        # eventual 15 cm stop_gap target is a config value to tighten only after the depth/odometry error is
        # measured and written to the calibration file -- do not lower this blind.
        self.declare_parameter("brake_margin_m", 1.0)
        self.declare_parameter("speed_limit_mps", 1.0)
        # Bumper-referenced stop parameters: default from the single geometry source, not a second literal.
        self.declare_parameter("camera_to_bumper_m", GEOMETRY.camera_to_front_bumper_horizontal_m)
        self.declare_parameter("stop_gap_from_bumper_m", 1.0)
        self.declare_parameter("lateral_margin_m", GEOMETRY.lateral_margin_m)
        self.declare_parameter("footprint_width_m", GEOMETRY.bumper_width_m + 2.0 * GEOMETRY.lateral_margin_m)
        brake = None
        minimum = float(self.get_parameter("brake_min_mps2").value)
        if minimum > 0:
            latency = float(self.get_parameter("t_latency").value)
            if latency < 0:
                latency = float(self.get_parameter("brake_delay_s").value)
            if str(self.get_parameter("source").value) != "LIVE" or latency > 0:
                brake = BrakeCapability(minimum, latency,
                                        str(self.get_parameter("brake_provenance").value),
                                        margin_m=float(self.get_parameter("brake_margin_m").value))
        self.engine = PlannerEngine(str(self.get_parameter("mission").value),
                                    str(self.get_parameter("source").value), brake,
                                    speed_limit_mps=float(self.get_parameter("speed_limit_mps").value),
                                    footprint_width_m=float(self.get_parameter("footprint_width_m").value),
                                    camera_to_bumper_m=float(self.get_parameter("camera_to_bumper_m").value),
                                    stop_gap_from_bumper_m=float(self.get_parameter("stop_gap_from_bumper_m").value),
                                    lateral_margin_m=float(self.get_parameter("lateral_margin_m").value))
        self.poses = {}
        self.feedback = None
        self.maps = {}
        self.statuses = {}
        self.obstacles = {}
        self.cycle = 0
        self.last_update = None
        self.create_subscription(Odometry, "/autokart/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(ConeArray, "/autokart/mapping/cones", self.on_map, qos_profile_sensor_data)
        self.create_subscription(MissionDebug, "/autokart/mapping/status", self.on_status, 10)
        self.create_subscription(ConeArray, "/autokart/planner/obstacles", self.on_obstacles, qos_profile_sensor_data)
        self.create_subscription(ActuatorFeedback, "/vehicle/actuator_feedback", self.on_feedback, 10)
        self.path_pub = self.create_publisher(Path, "/autokart/planner/path_candidate", 10)
        self.dashboard_path_pub = self.create_publisher(Path, "/planning/path", 10)
        self.status_pub = self.create_publisher(MissionDebug, "/autokart/planner/status", 10)
        self.dashboard_pub = self.create_publisher(MissionDebug, "/dashboard/debug", 10)
        self.create_timer(0.1, self.on_timer)

    def on_odom(self, message):
        pose = pose_from_odom(message)
        self.poses[pose.timestamp_ns] = pose
        if len(self.poses) > 16:
            del self.poses[min(self.poses)]
        for stamp in sorted(self.statuses):
            self._maybe_process(stamp)

    def on_feedback(self, message):
        self.feedback = message

    def on_map(self, message):
        stamp = ns(message.measurement_time)
        self.maps[stamp] = message
        if len(self.maps) > 8:
            del self.maps[min(self.maps)]
        self._maybe_process(stamp)

    def on_obstacles(self, message):
        stamp = ns(message.measurement_time)
        if message.frame_id != "base_link" or message.ground_truth:
            return
        self.obstacles[stamp] = [
            {"x_m": cone.position.x, "y_m": cone.position.y,
             "radius_m": max(0.0, cone.position_std_m)}
            for cone in message.cones if cone.semantic_class == 0
        ]
        if len(self.obstacles) > 8:
            del self.obstacles[min(self.obstacles)]
        self._maybe_process(stamp)

    def on_status(self, message):
        try:
            payload = json.loads(message.payload_json)
        except (ValueError, TypeError):
            self.publish(self.engine.reject("MAPPING_STATUS_MALFORMED", ns(message.measurement_time)))
            return
        stamp = ns(message.measurement_time)
        if payload.get("schema_version") != 1 or payload.get("sample_time_ns") != stamp:
            self.publish(self.engine.reject("MAPPING_STATUS_INVALID", stamp))
            return
        self.statuses[stamp] = (payload, time.monotonic())
        self._maybe_process(stamp)

    def _maybe_process(self, stamp):
        mapping = self.maps.get(stamp)
        pair = self.statuses.get(stamp)
        pose_stamps = [value for value in self.poses if 0 <= stamp - value <= 150_000_000]
        if pair is None or mapping is None or not pose_stamps:
            return
        pose = self.poses[max(pose_stamps)]
        payload, _ = pair
        if self.engine.mission == "BRAKING_TEST" and stamp not in self.obstacles:
            return
        self.maps.pop(stamp, None)
        self.statuses.pop(stamp, None)
        if mapping.frame_id != "odom":
            result = self.engine.reject("MAP_STATUS_OR_POSE_MISSING", stamp)
        elif not payload.get("valid") or not mapping.cones:
            result = self.engine.reject("NO_VALID_MAP_CONES" if not mapping.cones else "MAPPING_INVALID", stamp)
        else:
            feedback_valid = (self.feedback is not None and self.feedback.valid and
                              0 <= stamp - ns(self.feedback.measurement_time) <= 150_000_000)
            result = self.engine.evaluate(payload, pose, self.obstacles.pop(stamp, []),
                                          stop_feedback_valid=feedback_valid)
        self.last_update = time.monotonic()
        self.publish(result)

    def on_timer(self):
        expired = [stamp for stamp, (_, received) in self.statuses.items()
                   if time.monotonic() - received > 0.3]
        for stamp in expired:
            self.statuses.pop(stamp, None)
            self.maps.pop(stamp, None)
            self.publish(self.engine.reject("MAP_PAIR_OR_OBSTACLE_TIMEOUT", stamp))
        if self.last_update is not None and time.monotonic() - self.last_update > 0.5:
            self.last_update = None
            self.publish(self.engine.reject("PLANNER_INPUT_TIMEOUT", self.get_clock().now().nanoseconds))

    def publish(self, result):
        self.cycle += 1
        stamp = result["sample_time_ns"]
        path = Path()
        path.header.stamp = ros_time(max(0, stamp))
        path.header.frame_id = "odom"
        if result["valid"]:
            points = result["path_odom"]
            for index, (x, y) in enumerate(points):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                if index + 1 < len(points):
                    dx, dy = points[index + 1][0] - x, points[index + 1][1] - y
                    heading = atan2(dy, dx)
                    pose.pose.orientation.z = sin(heading / 2)
                    pose.pose.orientation.w = cos(heading / 2)
                else:
                    pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        self.path_pub.publish(path)
        self.dashboard_path_pub.publish(path)
        payload = {"schema_version": 1, "source": self.engine.source,
                   "processed_at_monotonic_ns": time.monotonic_ns(),
                   "valid": bool(result["valid"]), "reason_codes": result["reason_codes"],
                   "frame_id": "odom", "sample_time_ns": stamp,
                   "expires_ns": stamp + 100_000_000 if result["valid"] else stamp,
                   "sequence": self.cycle, "decision": result["decision"],
                   "target_speed_mps": result["target_speed_mps"],
                   "mission_state": result["mission_state"],
                   "stop_point_m": result.get("stop_point_m"),
                   "stop_point_frame": result.get("stop_point_frame", "base_link") if result.get("stop_point_m") is not None else None,
                   "stopping_distance_m": result.get("stopping_distance_m"),
                   "cone_forward_distance_m": result.get("cone_forward_distance_m"),
                   "bumper_to_cone_m": result.get("bumper_to_cone_m"),
                   "dead_reckoned": bool(result.get("dead_reckoned", False)),
                   "stop_gap_from_bumper_m": self.engine.stop_gap_from_bumper_m,
                   "red_zone_m": GEOMETRY.stop_gap_from_bumper_m,
                   "vehicle_authorized": False, "published_to_vcu": False}
        message = MissionDebug()
        message.measurement_time = ros_time(max(0, stamp))
        message.cycle = self.cycle
        message.frame_id = "odom"
        message.stage = "PLANNING"
        message.payload_json = json.dumps(payload, sort_keys=True, allow_nan=False)
        self.status_pub.publish(message)
        message.stage = "planning"
        self.dashboard_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
