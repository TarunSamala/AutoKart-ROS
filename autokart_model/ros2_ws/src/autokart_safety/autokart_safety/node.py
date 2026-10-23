"""Integration safety adapter: reuses VCU safety logic with permanent output inhibit.

Every profile stays DISARMED. ARM/manual-power-confirm/demo-drive requests
are excluded and any underlying core state remains diagnostic only. The
published authorized-command topic contains stop candidates with no authority.
Independent physical stop and measured hardware release remain external gates.
"""
from __future__ import annotations

import json
import time
import math
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from autokart_msgs.msg import ActuatorFeedback, MissionDebug, VcuState

from .adapter import SafetyCoreError, load_safety_core

ALLOWED_REQUESTS = {
    "stop", "reset_fault", "demo_drive_stop",
}
STATUS_TOPICS = {
    "perception": "/perception/diagnostics",
    "mapping": "/mapping/status",
    "planner": "/planning/status",
    "controller": "/control/status",
    "odom": "/autokart/localization/status",
}


class AutokartSafetyNode(Node):
    def __init__(self, **node_kwargs):
        super().__init__("autokart_safety", **node_kwargs)
        p = self.declare_parameter
        self.declare_parameter("vcu_safety_package_path", "")
        self.declare_parameter("overspeed_mps", 2.5)
        self.declare_parameter("stop_ineffective_s", 4.0)
        # Read-only: a person sets these true only after completing
        # docs/BENCH_TESTS.md. This node never writes them, and their compiled
        # default is false -- never auto-verified, never auto-armed.
        self.declare_parameter("calibration_verified", False)
        self.declare_parameter("vehicle_config_verified", False)
        self.declare_parameter("braking_verified", False)

        package_path = str(self.get_parameter("vcu_safety_package_path").value)
        self.core = None
        self.logic = None
        self.load_error: str | None = None
        try:
            self.core = load_safety_core(package_path)
            self.logic = self.core.SafetyLogic(self.core.Config(
                overspeed_mps=float(self.get_parameter("overspeed_mps").value),
                stop_ineffective_s=float(self.get_parameter("stop_ineffective_s").value),
            ))
        except SafetyCoreError as exc:
            self.load_error = str(exc)
            self.get_logger().error(
                f"autokart_safety startup failed, core unavailable: {self.load_error}. "
                "This node will never authorize ARM while in this state.")

        self.calibration_verified = bool(self.get_parameter("calibration_verified").value)
        self.vehicle_config_verified = bool(self.get_parameter("vehicle_config_verified").value)
        self.braking_verified = bool(self.get_parameter("braking_verified").value)

        self.last_seen: dict[str, float] = {}
        self.vcu = None
        self.odom_status = None
        self.last_odom = None
        self.speed_mps: float | None = None
        self.speed_valid = False
        self.planner_decision: str | None = None
        self.pending_requests = deque(maxlen=32)
        self.cycle = 0

        self.canonical_pub = self.create_publisher(MissionDebug, "/safety/status", 10)
        self.authorized_pub = self.create_publisher(MissionDebug, "/vehicle/authorized_command", 10)
        self.status_pub = self.create_publisher(MissionDebug, "/autokart/safety/status", 10)
        self.dashboard_pub = self.create_publisher(MissionDebug, "/dashboard/debug", 10)
        self.request_pub = self.create_publisher(String, "/autokart/vcu/request", 10)

        self.create_subscription(Odometry, "/odom", self._on_odometry, 10)
        self.create_subscription(String, "/vehicle/odometry_status",
            self._on_odometry_status, 10)
        self.create_subscription(MissionDebug, "/control/request", self._on_control_request, 10)
        self.create_subscription(VcuState, "/vcu/state", self._on_vcu, 10)
        self.create_subscription(ActuatorFeedback, "/vehicle/actuator_feedback", self._on_feedback, 10)
        self.create_subscription(String, "/autokart/safety/request", self._on_request, 10)
        for key, topic in STATUS_TOPICS.items():
            self.create_subscription(MissionDebug, topic, self._make_status_handler(key), 10)

        self.create_timer(0.02, self._tick)

    def _touch(self, key: str) -> None:
        self.last_seen[key] = time.monotonic()

    def _on_vcu(self, message: VcuState) -> None:
        if self.core is None:
            return
        self._touch("vcu")
        self.vcu = self.core.VcuView(
            state=str(message.state), estop_active=bool(message.emergency_stop_active),
            estop_input_fitted=bool(message.estop_input_fitted),
            active_faults=tuple(message.active_faults))

    def _on_feedback(self, message: ActuatorFeedback) -> None:
        # motor_speed_radps is angular velocity, never translational m/s.
        # Measured wheel geometry belongs in odometry; do not guess it here.
        self._touch("feedback")

    def _on_odometry_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict) or type(payload.get("sample_time_ns")) is not int:
                raise ValueError("ODOMETRY_STATUS_INVALID")
            msg = MissionDebug(payload_json=message.data)
            stamp = payload["sample_time_ns"]
            msg.measurement_time.sec, msg.measurement_time.nanosec = divmod(max(0, stamp), 1_000_000_000)
            self._make_status_handler("odom")(msg)
            self.odom_status = payload if self._age("odom") is not None else None
        except (ValueError, TypeError):
            self.odom_status = None
            self.last_seen.pop("odom", None)
        if self.last_odom is not None:
            self._on_odometry(self.last_odom)

    def _on_odometry(self, message: Odometry) -> None:
        self.last_odom = message
        value = float(message.twist.twist.linear.x)
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        now = self.get_clock().now().nanoseconds
        status = self.odom_status or {}
        self.speed_valid = (message.header.frame_id == "odom" and
                            message.child_frame_id == "base_link" and math.isfinite(value) and
                            0 <= now - stamp <= 200_000_000 and
                            status.get("sample_time_ns") == stamp and status.get("valid") is True and
                            status.get("source") in {"LIVE", "LIVE_PHYSICAL"} and
                            type(status.get("expires_ns")) is int and status["expires_ns"] > now)
        self.speed_mps = value if self.speed_valid else None
        if self.speed_valid:
            self._touch("speed")
        else:
            self.last_seen.pop("speed", None)

    def _on_control_request(self, message: MissionDebug) -> None:
        # Observe requests for diagnostics only. Integration cannot arm.
        self._make_status_handler("control_request")(message)

    def _make_status_handler(self, key: str):
        def handler(message: MissionDebug) -> None:
            try:
                payload = json.loads(message.payload_json)
            except (ValueError, TypeError):
                self.last_seen.pop(key, None)
                if key == "planner":
                    self.planner_decision = None
                return
            now = self.get_clock().now().nanoseconds
            measurement_stamp = message.measurement_time.sec * 1_000_000_000 + message.measurement_time.nanosec
            qualified = (isinstance(payload, dict) and payload.get("schema_version") == 1 and
                payload.get("source") in {"LIVE_PHYSICAL", "LIVE"} and payload.get("valid") is True and
                type(payload.get("sample_time_ns")) is int and payload["sample_time_ns"] == measurement_stamp and
                measurement_stamp > 0 and 0 <= now - measurement_stamp <= 300_000_000 and
                type(payload.get("expires_ns")) is int and payload["expires_ns"] > now)
            if qualified:
                self._touch(key)
            else:
                self.last_seen.pop(key, None)
            if key == "planner":
                self.planner_decision = payload.get("decision") if isinstance(payload, dict) else None
        return handler

    def _on_request(self, message: String) -> None:
        value = str(message.data).strip().lower()
        if value in ALLOWED_REQUESTS:
            self.pending_requests.append(value)
        else:
            self.get_logger().warning(f"Ignoring request outside the allowed set: {message.data!r}")

    def _age(self, key: str) -> float | None:
        seen = self.last_seen.get(key)
        return None if seen is None else time.monotonic() - seen

    def _publish_unavailable(self) -> None:
        """Published every tick while the real safety core could not be
        loaded. Never one-shot: a node watching only process/node liveness
        must not see a frozen-looking-fine /autokart/safety/status (same
        lesson the hp60c reviewer's fix #3 encoded for that node)."""
        self.cycle += 1
        now_stamp = self.get_clock().now().to_msg()
        payload = {
            "schema_version": 1, "source": "UNVERIFIED", "valid": False,
            "reason_codes": [f"SAFETY_CORE_UNAVAILABLE:{self.load_error}"],
            "frame_id": "base_link",
            "sample_time_ns": self.get_clock().now().nanoseconds,
            "sequence": self.cycle, "state": "UNAVAILABLE", "arm_allowed": False,
            "output_inhibited": True,
            "vcu_heartbeat_ok": False, "startup_ready": False,
            "stop_latched": False, "stages": [],
            "manual_power_cut_confirmed": False,
            "demo_drive_active": False,
            "vehicle_authorized": False,
        }
        message = MissionDebug()
        message.measurement_time = now_stamp
        message.cycle = self.cycle
        message.frame_id = "base_link"
        message.stage = "SAFETY"
        message.payload_json = json.dumps(payload, sort_keys=True, allow_nan=False)
        self.status_pub.publish(message)
        self.canonical_pub.publish(message)
        self._publish_inhibited_command(now_stamp)
        message.stage = "safety"
        self.dashboard_pub.publish(message)

    def _tick(self) -> None:
        if self.logic is None or self.core is None:
            self._publish_unavailable()
            return

        inputs = self.core.Inputs(
            ages={k: self._age(k) for k in ("vcu", "odom", "perception", "mapping", "planner", "controller")},
            vcu=self.vcu, speed_mps=self.speed_mps, speed_valid=self.speed_valid and self._age("speed") is not None and self._age("speed") < 0.2,
            planner_decision=self.planner_decision,
            calibration_verified=self.calibration_verified,
            vehicle_config_verified=self.vehicle_config_verified,
            braking_verified=self.braking_verified,
        )
        requests = list(self.pending_requests)
        self.pending_requests.clear()
        out = self.logic.step(time.monotonic(), inputs, requests)
        for event in out.events:
            self.get_logger().info(event)
        if out.vcu_request in {"stop", "disarm", "STOP", "DIS"}:
            self.request_pub.publish(String(data=out.vcu_request))

        self.cycle += 1
        now_stamp = self.get_clock().now().to_msg()
        payload = {
            "schema_version": 1, "source": "UNVERIFIED", "valid": False,
            "reason_codes": list(out.reasons) + ["INTEGRATION_OUTPUT_INHIBITED"], "frame_id": "base_link",
            "sample_time_ns": self.get_clock().now().nanoseconds,
            "sequence": self.cycle, "state": "DISARMED", "arm_allowed": False,
            "core_state": out.state, "output_inhibited": True,
            "vcu_heartbeat_ok": out.vcu_heartbeat_ok, "startup_ready": False, "core_startup_ready": out.startup_ready,
            "stop_latched": out.stop_latched, "stages": out.stages,
            "manual_power_cut_confirmed": out.manual_power_cut_confirmed,
            "demo_drive_active": out.demo_drive_active,
            "vehicle_authorized": False,
        }
        message = MissionDebug()
        message.measurement_time = now_stamp
        message.cycle = self.cycle
        message.frame_id = "base_link"
        message.stage = "SAFETY"
        message.payload_json = json.dumps(payload, sort_keys=True, allow_nan=False)
        self.status_pub.publish(message)
        self.canonical_pub.publish(message)
        self._publish_inhibited_command(now_stamp)
        message.stage = "safety"
        self.dashboard_pub.publish(message)

    def _publish_inhibited_command(self, stamp) -> None:
        message = MissionDebug()
        message.measurement_time = stamp
        message.cycle = self.cycle
        message.frame_id = "base_link"
        message.stage = "SAFETY"
        message.payload_json = json.dumps({
            "schema_version": 1, "source": "UNVERIFIED", "valid": False,
            "sequence": self.cycle, "sample_time_ns": self.get_clock().now().nanoseconds,
            "frame_id": "base_link", "vehicle_authorized": False,
            "output_inhibited": True, "stop_requested": True, "state": "DISARMED",
            "target_speed_mps": 0.0, "target_steering_rad": 0.0,
            "reason_codes": ["INTEGRATION_OUTPUT_INHIBITED"],
        }, allow_nan=False)
        self.authorized_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = AutokartSafetyNode()
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
