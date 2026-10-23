"""autokart_controller: standalone ROS 2 node wrapping the existing Pure Pursuit + longitudinal
PID controller as a bounded, expiring command CANDIDATE. This node has explicitly NO authority to
command the vehicle while disarmed: its output topic is not subscribed by any actuator/VCU
transport in this library, and `valid`/`vehicle_authorized` never claim otherwise. See CONTRACT.md.
"""
from __future__ import annotations

import json
import time
from math import hypot

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from autokart_msgs.msg import ActuatorFeedback, MotionCommand, SafetyEnvelope

from .pursuit_backends import build_pursuit_backend, PursuitBackendError
from .controller_core import ControllerCore

TIMER_PERIOD_S = 0.1


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
        self.declare_parameter("odom_topic", "/autokart/odom")
        self.declare_parameter("path_topic", "/autokart_planner/path")
        self.declare_parameter("safety_envelope_topic", "/autokart_planner/safety_envelope")
        self.declare_parameter("speed_profile_topic", "/autokart_planner/speed_profile")
        self.declare_parameter("actuator_feedback_topic", "/vehicle/actuator_feedback")
        self.declare_parameter("source", "SIMULATED")
        # How long a received path waits for its same-stamp envelope (and, if the envelope is
        # valid, its speed profile) before being processed anyway / rejected. See _maybe_process.
        self.declare_parameter("pair_timeout_s", 0.1)
        # MotionCommand.expiry = timestamp + this, for valid candidates (0.1 s mirrors
        # kart_sim.autonomy.ControllerManager.validity_s). Invalid candidates expire at their stamp.
        self.declare_parameter("command_validity_s", 0.1)

        self.source = str(self.get_parameter("source").value)
        self.pair_timeout_s = float(self.get_parameter("pair_timeout_s").value)
        path_max_age_s = float(self.get_parameter("path_max_age_s").value)
        if self.pair_timeout_s + TIMER_PERIOD_S >= path_max_age_s:
            # Fails safe (such paths are rejected PATH_EXPIRED), but would do so on every cycle in
            # which the speed profile is late or missing -- say so instead of silently stopping.
            self.get_logger().warning(
                f"autokart_controller: pair_timeout_s ({self.pair_timeout_s}) + timer period "
                f"({TIMER_PERIOD_S}) >= path_max_age_s ({path_max_age_s}); paths whose speed profile "
                "is late will expire before processing and be rejected")
        self.command_validity_ns = int(float(self.get_parameter("command_validity_s").value) * 1e9)
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
        # Stamp-keyed pairing buffers (bounded). The planner publishes path, envelope and speed
        # profile for the same sample time on three topics, with no cross-topic ordering guarantee.
        self.pending_paths: dict = {}  # stamp_ns -> (points, received_wall_s)
        self.envelopes: dict = {}  # stamp_ns -> (valid, maximum_speed_mps)
        self.speed_profiles: dict = {}  # stamp_ns -> [speeds]
        self.feedback_valid = True  # no feedback received yet => assume not brake-stuck
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

        self.create_timer(TIMER_PERIOD_S, self.on_timer)

    def on_odom(self, message: Odometry) -> None:
        from math import atan2
        q = message.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = atan2(siny, cosy)
        linear = message.twist.twist.linear
        speed = hypot(linear.x, linear.y)
        self.pose = (float(message.pose.pose.position.x), float(message.pose.pose.position.y),
                     float(yaw), float(speed), time.monotonic())

    @staticmethod
    def _bounded_put(store: dict, key, value, limit: int = 8) -> None:
        store[key] = value
        while len(store) > limit:
            del store[min(store)]

    def on_envelope(self, message: SafetyEnvelope) -> None:
        stamp_ns = ns(message.timestamp)
        self._bounded_put(self.envelopes, stamp_ns, (bool(message.valid), float(message.maximum_speed_mps)))
        self._maybe_process(stamp_ns)

    def on_speed_profile(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            stamp_ns = int(payload["sample_time_ns"])
            values = [float(v) for v in payload.get("speed_profile_mps", [])]
        except (ValueError, TypeError, KeyError):
            # Malformed diagnostic side-channel: drop it, do not propagate a parse exception out of
            # a subscription callback. The pair timeout then processes the path with the
            # envelope's flat speed cap instead.
            self.get_logger().warning("autokart_controller: ignoring malformed speed_profile payload")
            return
        self._bounded_put(self.speed_profiles, stamp_ns, values)
        self._maybe_process(stamp_ns)

    def on_feedback(self, message: ActuatorFeedback) -> None:
        self.feedback_valid = bool(message.valid)

    def on_path(self, message: Path) -> None:
        self.last_path_wall_s = time.monotonic()
        stamp_ns = ns(message.header.stamp)
        points = [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]
        self._bounded_put(self.pending_paths, stamp_ns, (points, time.monotonic()))
        self._maybe_process(stamp_ns)

    def on_timer(self) -> None:
        if self.engine is None:
            self._publish_fault(self.startup_error or "BACKEND_UNAVAILABLE")
            return
        now = time.monotonic()
        for stamp_ns in sorted(self.pending_paths):
            _, received = self.pending_paths[stamp_ns]
            if now - received > self.pair_timeout_s:
                if stamp_ns in self.envelopes:
                    # Envelope arrived, profile did not: process with the envelope's flat cap.
                    self._maybe_process(stamp_ns, force=True)
                else:
                    self.pending_paths.pop(stamp_ns, None)
                    self.publish(self.engine.reject("PLANNER_STATUS_MISSING", stamp_ns))
        if now - self.last_path_wall_s > float(self.get_parameter("path_max_age_s").value):
            self.publish(self.engine.reject("PATH_MISSING_OR_STALE", self.get_clock().now().nanoseconds))

    def _maybe_process(self, stamp_ns: int, force: bool = False) -> None:
        if self.engine is None:
            self._publish_fault(self.startup_error or "BACKEND_UNAVAILABLE")
            return
        if stamp_ns not in self.pending_paths or stamp_ns not in self.envelopes:
            return
        envelope_valid, envelope_max_speed = self.envelopes[stamp_ns]
        if not force and envelope_valid and stamp_ns not in self.speed_profiles:
            return  # wait (up to pair_timeout_s) for the per-point profile
        points, _ = self.pending_paths.pop(stamp_ns)
        self.envelopes.pop(stamp_ns, None)
        speed_profile = self.speed_profiles.pop(stamp_ns, [])
        self._process(points, stamp_ns, envelope_valid, envelope_max_speed, speed_profile)

    def _process(self, points: list, stamp_ns: int, envelope_valid: bool, envelope_max_speed: float,
                 speed_profile: list) -> None:
        now_ns = self.get_clock().now().nanoseconds
        wall_now = time.monotonic()
        dt_s = 0.1 if self.last_compute_wall_s is None else max(1e-3, wall_now - self.last_compute_wall_s)
        self.last_compute_wall_s = wall_now

        pose_fresh = self.pose is not None and wall_now - self.pose[4] <= self.pose_max_age_ns / 1e9
        pose_x, pose_y, pose_yaw, pose_speed = self.pose[:4] if self.pose is not None else (0.0, 0.0, 0.0, 0.0)

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
        if not result["valid"]:
            # After a STOP, the next valid cycle must not compute dt across the whole fault gap: a
            # multi-second dt would scale the steering rate limit (rate*dt) enough to disable it and
            # dump error*dt into the PID integral. Restart from the nominal first-cycle dt instead.
            self.last_compute_wall_s = None
        stamp_ns = max(0, result["sample_time_ns"])
        command = MotionCommand()
        command.timestamp = ros_time(stamp_ns)
        # Valid candidates expire command_validity_s after their sample time; invalid ones at the
        # sample time itself (already expired -- never actionable). See CONTRACT.md.
        command.expiry = ros_time(stamp_ns + (self.command_validity_ns if result["valid"] else 0))
        command.source = f"autokart_controller/{self.source}"
        command.sequence = result["sequence"] & 0xFFFFFFFF
        command.mode = "speed_steering"
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
            "vehicle_authorized": False, "published_to_vcu": False,
        }
        diagnostics = String()
        diagnostics.data = json.dumps(payload, sort_keys=True, allow_nan=False)
        self.diagnostics_pub.publish(diagnostics)


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
