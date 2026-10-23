"""autokart_controller: standalone ROS 2 node wrapping the existing Pure Pursuit + longitudinal
PID controller as a bounded, expiring command CANDIDATE. This node has explicitly NO authority to
command the vehicle while disarmed: its output topic is not subscribed by any actuator/VCU
transport in this library, and `valid`/`vehicle_authorized` never claim otherwise. See CONTRACT.md.
"""
from __future__ import annotations

import json
import time
from math import hypot, isfinite

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from autokart_msgs.msg import ActuatorFeedback, MotionCommand, SafetyEnvelope, MissionDebug

from .pursuit_backends import build_pursuit_backend, PursuitBackendError
from .controller_core import ControllerCore

TIMER_PERIOD_S = 0.02


def ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def ros_time(stamp_ns: int):
    from builtin_interfaces.msg import Time
    stamp_ns = max(0, int(stamp_ns))
    return Time(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)


class ControllerNode(Node):
    def __init__(self, **node_kwargs):
        # **node_kwargs (e.g. parameter_overrides=[...]) threaded through to rclpy.Node, same
        # pattern as autokart_hp60c/autokart_planner use for deterministic test construction.
        super().__init__("autokart_controller", **node_kwargs)
        self.declare_parameter("pursuit_backend", "builtin")
        self.declare_parameter("kart_sim_package_path", "")
        # UNVERIFIED mechanical/physical defaults -- see SENIOR_REVIEWER_REPORT.md. This repository
        # has not measured this kart's wheelbase or its mechanical steering travel limit (see
        # PROJECT_INDEX.md: "as-built wiring ... remain unknown or unverified"). These defaults
        # come from the existing kart/ros2_ws autokart_controller's own code defaults, which carry
        # the identical caveat there -- not independently re-guessed here, but not elevated to
        # "measured" either.
        self.declare_parameter("wheelbase_m", 1.0)  # UNVERIFIED nominal, see CONTRACT.md
        self.declare_parameter("maximum_steering_rad", 0.6)  # UNVERIFIED nominal mechanical limit
        self.declare_parameter("maximum_steering_rate_radps", 3.0)  # UNVERIFIED nominal
        self.declare_parameter("lookahead_m", 1.5)
        self.declare_parameter("speed_lookahead_gain_s", 0.15)
        self.declare_parameter("maximum_speed_mps", 5.0)
        self.declare_parameter("path_max_age_s", 0.5)
        self.declare_parameter("pose_max_age_s", 0.15)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("path_topic", "/autokart_planner/path")
        self.declare_parameter("safety_envelope_topic", "/autokart_planner/safety_envelope")
        self.declare_parameter("speed_profile_topic", "/autokart_planner/speed_profile")
        self.declare_parameter("actuator_feedback_topic", "/vehicle/actuator_feedback")
        self.declare_parameter("source", "UNVERIFIED")

        self.declare_parameter("candidate_ttl_s", 0.1)
        self.declare_parameter("feedback_max_age_s", 0.15)
        self.declare_parameter("physical_limits_verified", False)
        self.latest_path = None
        self.planning_expiry_ns = 0
        self.feedback_stamp_ns = 0
        self.actual_steering_rad = None
        self.odom_status = None
        self.odom_stamp_ns = 0
        self.source = str(self.get_parameter("source").value)
        self.pose_max_age_ns = int(float(self.get_parameter("pose_max_age_s").value) * 1e9)

        backend_name = str(self.get_parameter("pursuit_backend").value)
        self.engine = None
        self.startup_error = None
        try:
            backend = build_pursuit_backend(
                backend_name,
                wheelbase_m=float(self.get_parameter("wheelbase_m").value),
                lookahead_m=float(self.get_parameter("lookahead_m").value),
                speed_lookahead_gain_s=float(self.get_parameter("speed_lookahead_gain_s").value),
                maximum_steering_rad=float(self.get_parameter("maximum_steering_rad").value),
                maximum_steering_rate_radps=float(self.get_parameter("maximum_steering_rate_radps").value),
                kart_sim_package_path=str(self.get_parameter("kart_sim_package_path").value),
            )
            self.engine = ControllerCore(
                backend=backend,
                maximum_speed_mps=float(self.get_parameter("maximum_speed_mps").value),
                max_pose_age_ns=self.pose_max_age_ns,
                max_path_age_ns=int(float(self.get_parameter("path_max_age_s").value) * 1e9),
            )
        except PursuitBackendError as exc:
            self.startup_error = str(exc)
            self.get_logger().error(f"autokart_controller startup failed: {self.startup_error}")

        self.pose = None  # (x, y, yaw_rad, speed_mps, received_wall_s)
        self.envelope = None  # (stamp_ns, valid, maximum_speed_mps)
        self.speed_profile = None  # (stamp_ns, [speeds])
        self.feedback_valid = False  # missing measured feedback fails closed
        self.cycle = 0
        self.last_compute_wall_s = None
        # See autokart_planner's identical reasoning: start the staleness clock at construction,
        # not at the first received path, so a planner that never starts still gets reported.
        self.last_path_wall_s = time.monotonic()

        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value),
                                 self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Path, str(self.get_parameter("path_topic").value), self.on_path, 10)
        self.create_subscription(SafetyEnvelope, str(self.get_parameter("safety_envelope_topic").value),
                                 self.on_envelope, 10)
        self.create_subscription(String, str(self.get_parameter("speed_profile_topic").value),
                                 self.on_speed_profile, 10)
        self.create_subscription(ActuatorFeedback, str(self.get_parameter("actuator_feedback_topic").value),
                                 self.on_feedback, 10)

        self.command_pub = self.create_publisher(MotionCommand, "/autokart_controller/motion_command_candidate", 10)
        self.diagnostics_pub = self.create_publisher(String, "/autokart_controller/diagnostics", 10)

        self.request_pub = self.create_publisher(MissionDebug, "/control/request", 10)
        self.status_pub = self.create_publisher(MissionDebug, "/control/status", 10)
        self.create_subscription(MissionDebug, "/planning/status", self.on_planning_status, 10)
        self.create_subscription(String, "/vehicle/odometry_status", self.on_odometry_status, 10)
        self.create_timer(TIMER_PERIOD_S, self.on_timer)

    def on_planning_status(self, message):
        try:
            p = json.loads(message.payload_json)
            stamp = ns(message.measurement_time)
            if not isinstance(p, dict) or p.get("sample_time_ns") != stamp or p.get("frame_id") != "odom":
                raise ValueError("invalid planning schema")
            if type(p.get("valid")) is not bool or type(p.get("expires_ns")) is not int:
                raise ValueError("invalid planning boolean or expiry")
            values = p.get("speed_profile_mps", [])
            if not isinstance(values, list) or not all(isinstance(v, (int, float)) and isfinite(v) and v >= 0 for v in values):
                raise ValueError("invalid profile")
            points = p.get("path_odom", [])
            if not isinstance(points, list) or any(not isinstance(point, (list, tuple)) or len(point) != 2 or
                    not all(isinstance(v, (int, float)) and isfinite(v) for v in point) for point in points):
                raise ValueError("invalid path")
            if p.get("valid") and len(points) != len(values):
                raise ValueError("profile/path mismatch")
            if p.get("source") != self.source:
                raise ValueError("source mismatch")
            self.envelope = (stamp, p["valid"] is True, max(p.get("speed_profile_mps", [0.0]) or [0.0]))
            self.speed_profile = (stamp, p.get("speed_profile_mps", []))
            self.planning_expiry_ns = int(p["expires_ns"])
            self.latest_path = (p.get("path_odom", []), stamp)
            self.last_path_wall_s = time.monotonic()
        except (ValueError, TypeError, KeyError):
            self.latest_path = None
            self._publish_fault("PLANNING_STATUS_MALFORMED")

    def on_odometry_status(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("invalid odometry status")
            self.odom_status = payload
        except (ValueError, TypeError):
            self.odom_status = None

    def odometry_qualified(self, now_ns):
        status = self.odom_status
        expected_source = "LIVE" if self.source == "LIVE_PHYSICAL" else self.source
        observed_source = status.get("source") if status else None
        if expected_source in ("SIMULATED", "SYNTHETIC") and observed_source in ("SIMULATED", "SYNTHETIC"):
            observed_source = expected_source
        return bool(status and status.get("valid") is True and
            observed_source == expected_source and
            status.get("sample_time_ns") == self.odom_stamp_ns and
            isinstance(status.get("expires_ns"), int) and
            self.odom_stamp_ns <= now_ns <= status["expires_ns"] and
            0 <= now_ns-self.odom_stamp_ns <= self.pose_max_age_ns and
            not status.get("continuity_lost", False) and
            (self.source not in ("LIVE_PHYSICAL", "LIVE") or status.get("physical_qualified") is True))

    def on_odom(self, message: Odometry) -> None:
        from math import atan2
        now = self.get_clock().now().nanoseconds
        if message.header.frame_id != "odom" or message.child_frame_id != "base_link" or not 0 <= now-ns(message.header.stamp) <= self.pose_max_age_ns:
            self.pose = None
            return
        q = message.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = atan2(siny, cosy)
        self.odom_stamp_ns = ns(message.header.stamp)
        linear = message.twist.twist.linear
        speed = hypot(linear.x, linear.y)
        self.pose = (float(message.pose.pose.position.x), float(message.pose.pose.position.y),
                     float(yaw), float(speed), time.monotonic())

    def on_envelope(self, message: SafetyEnvelope) -> None:
        self.envelope = (ns(message.timestamp), bool(message.valid), float(message.maximum_speed_mps))

    def on_speed_profile(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self.speed_profile = (int(payload["sample_time_ns"]), list(payload.get("speed_profile_mps", [])))
        except (ValueError, TypeError, KeyError):
            # Malformed diagnostic side-channel: drop it, do not propagate a parse exception out of
            # a subscription callback. ControllerCore falls back to the envelope's flat speed cap
            # when no matching profile is available.
            self.get_logger().warning("autokart_controller: ignoring malformed speed_profile payload")

    def on_feedback(self, message: ActuatorFeedback) -> None:
        self.feedback_valid = bool(message.valid) and isfinite(message.road_wheel_angle_rad)
        self.feedback_stamp_ns = ns(message.measurement_time)
        self.actual_steering_rad = float(message.road_wheel_angle_rad) if self.feedback_valid else None

    def on_path(self, message: Path) -> None:
        self.last_path_wall_s = time.monotonic()
        stamp_ns = ns(message.header.stamp)
        points = [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]
        self.latest_path = (points, stamp_ns)

    def on_timer(self) -> None:
        if self.engine is None:
            self._publish_fault(self.startup_error or "BACKEND_UNAVAILABLE")
            return
        now = time.monotonic()
        if self.last_path_wall_s is not None and now - self.last_path_wall_s > \
                float(self.get_parameter("path_max_age_s").value):
            self.publish(self.engine.reject("PATH_MISSING_OR_STALE", self.get_clock().now().nanoseconds))
        elif self.latest_path is not None:
            self._process(*self.latest_path)

    def _process(self, points: list, stamp_ns: int) -> None:
        if self.engine is None:
            self._publish_fault(self.startup_error or "BACKEND_UNAVAILABLE")
            return
        now_ns = self.get_clock().now().nanoseconds
        if self.source not in ("LIVE_PHYSICAL", "LIVE", "SYNTHETIC", "SIMULATED", "REPLAY", "UNVERIFIED"):
            self._publish_fault("SOURCE_UNRECOGNIZED")
            return
        if self.source in ("LIVE", "LIVE_PHYSICAL") and str(self.get_parameter("pursuit_backend").value) != "reused":
            self._publish_fault("LIVE_REUSED_BACKEND_REQUIRED")
            return
        if not self.odometry_qualified(now_ns):
            self._publish_fault("ODOMETRY_STATUS_UNQUALIFIED")
            return
        if not 0 < float(self.get_parameter("candidate_ttl_s").value) <= 0.1:
            self._publish_fault("CANDIDATE_TTL_INVALID")
            return
        if now_ns > self.planning_expiry_ns:
            self._publish_fault("PLANNING_EXPIRED")
            return
        if self.source in ("LIVE_PHYSICAL", "LIVE") and self.get_parameter("physical_limits_verified").value is not True:
            self._publish_fault("PHYSICAL_LIMITS_UNQUALIFIED")
            return
        if not self.feedback_valid or not 0 <= now_ns-self.feedback_stamp_ns <= float(self.get_parameter("feedback_max_age_s").value)*1e9:
            self._publish_fault("ACTUATOR_FEEDBACK_MISSING_INVALID_OR_STALE")
            return
        wall_now = time.monotonic()
        dt_s = 0.1 if self.last_compute_wall_s is None else max(1e-3, wall_now - self.last_compute_wall_s)
        self.last_compute_wall_s = wall_now

        pose_fresh = self.pose is not None and wall_now - self.pose[4] <= self.pose_max_age_ns / 1e9
        pose_x, pose_y, pose_yaw, pose_speed = self.pose[:4] if self.pose is not None else (0.0, 0.0, 0.0, 0.0)

        if self.envelope is None:
            envelope_valid, envelope_max_speed = False, 0.0
        else:
            envelope_stamp_ns, envelope_valid_raw, envelope_max_speed = self.envelope
            envelope_valid = envelope_valid_raw and envelope_stamp_ns == stamp_ns

        speed_profile: list = []
        if self.speed_profile is not None:
            profile_stamp_ns, profile_values = self.speed_profile
            if profile_stamp_ns == stamp_ns:
                speed_profile = profile_values

        result = self.engine.evaluate(
            path_points=points, path_stamp_ns=stamp_ns, now_ns=now_ns,
            envelope_valid=envelope_valid, envelope_max_speed_mps=envelope_max_speed,
            speed_profile=speed_profile, pose_x=pose_x, pose_y=pose_y, pose_yaw_rad=pose_yaw,
            pose_speed_mps=pose_speed, pose_fresh=pose_fresh, dt_s=dt_s,
            brake_stuck=not self.feedback_valid,
        )
        self.publish(result)

    def _publish_fault(self, reason: str) -> None:
        self.cycle += 1
        stamp_ns = self.get_clock().now().nanoseconds
        result = {
            "valid": False, "reason_codes": [reason], "sample_time_ns": stamp_ns,
            "sequence": self.cycle, "steering_rad": 0.0, "speed_mps": 0.0, "brake": 0.0,
            "drive": 0.0, "cross_track_error_m": None, "heading_error_rad": None,
            "speed_tracking_error_mps": None,
        }
        self.publish(result)

    def publish(self, result: dict) -> None:
        self.cycle += 1
        stamp_ns = max(0, result["sample_time_ns"])
        command = MotionCommand()
        issued_ns = self.get_clock().now().nanoseconds
        expiry_ns = min(issued_ns + int(float(self.get_parameter("candidate_ttl_s").value)*1e9), self.planning_expiry_ns) if result["valid"] else issued_ns
        command.timestamp = ros_time(issued_ns)
        command.expiry = ros_time(max(issued_ns, expiry_ns))
        command.source = f"autokart_controller/{self.source}"
        command.sequence = result["sequence"] & 0xFFFFFFFF
        command.mode = "speed_steering" if result["valid"] else "STOP"
        command.target_speed_mps = float(result["speed_mps"])
        command.steering_angle_rad = float(result["steering_rad"])
        command.curvature_1pm = 0.0
        command.use_curvature = False
        command.acceleration_limit_mps2 = 0.0
        command.deceleration_limit_mps2 = 0.0
        command.valid = bool(result["valid"])
        command.integrity = 0  # no VCU integrity signing: this candidate is never sent to a VCU bridge
        self.command_pub.publish(command)

        payload = {
            "schema_version": 1, "source": self.source,
            "processed_at_monotonic_ns": time.monotonic_ns(),
            "valid": bool(result["valid"]), "reason_codes": result["reason_codes"],
            "sample_time_ns": stamp_ns, "sequence": result["sequence"],
            "brake": result["brake"], "drive": result["drive"],
            "cross_track_error_m": result["cross_track_error_m"],
            "heading_error_rad": result["heading_error_rad"],
            "speed_tracking_error_mps": result["speed_tracking_error_mps"],
            "frame_id": "base_link", "issued_ns": issued_ns, "expires_ns": expiry_ns,
            "target_speed_mps": float(result["speed_mps"]),
            "target_steering_rad": float(result["steering_rad"]),
            "stop_requested": not result["valid"], "output_inhibited": True,
            "steering_tracking_error_rad": float(result["steering_rad"])-self.actual_steering_rad if self.actual_steering_rad is not None else None,
            "path_index": result.get("path_index"), "path_arc_m": result.get("path_arc_m"),
            "yaw_rate_error_radps": None, "yaw_rate_error_reason": "MEASURED_YAW_RATE_NOT_AVAILABLE",
            "vehicle_authorized": False, "published_to_vcu": False,
        }
        diagnostics = String()
        diagnostics.data = json.dumps(payload, sort_keys=True, allow_nan=False)
        self.diagnostics_pub.publish(diagnostics)
        status = MissionDebug()
        status.measurement_time = ros_time(stamp_ns)
        status.frame_id = "base_link"
        status.stage = "CONTROL"
        status.cycle = self.cycle
        status.payload_json = diagnostics.data
        self.request_pub.publish(status)
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
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
