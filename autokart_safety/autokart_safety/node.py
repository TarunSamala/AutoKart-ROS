"""autokart_safety: standalone ROS wrapper around the real VCU arming safety
supervisor (reused via ``adapter.load_safety_core``, never reimplemented).

This is a SOFTWARE authority only. It is the only node in this library that
may request ARM of the vehicle bridge, and it never auto-arms: default state
is DISARMED, and arming requires every upstream input to be measurably fresh
and valid, all three ``*_verified`` flags to be true (read-only parameters a
person sets after the bench checklist -- this node never writes them), and an
operator to have freshly confirmed the manual power/traction disconnect is
within reach for this specific arm attempt.

THIS NODE IS NOT THE PHYSICAL E-STOP AND MUST NEVER BE TREATED AS ONE. Per
this repository's PROJECT_INDEX.md/README.txt, GPIO7's E-stop input has no
status wire fitted; the manual battery/supply disconnect is the only proven
emergency stop. A software STOP/DISARM published by this node depends on the
laptop, the USB link and the ESP32 all continuing to function -- it provides
no independent, hardware-enforced guarantee. See CONTRACT.md.

Distinct, separate, NOT replaced by this node: ``autokart_safety_bridge``'s
``front_safety_envelope_monitor`` is a diagnostic-only front-obstacle envelope
classifier (``vehicle_authority`` permanently OFF). See adapter.py and
CONTRACT.md.
"""
from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from autokart_msgs.msg import ActuatorFeedback, MissionDebug, VcuState

from .adapter import SafetyCoreError, load_safety_core

ALLOWED_REQUESTS = {
    "arm", "stop", "reset_fault", "confirm_manual_power_cut",
    "demo_drive_start", "demo_drive_stop",
}
STATUS_TOPICS = {
    "perception": "/autokart/perception/status",
    "mapping": "/autokart/mapping/status",
    "planner": "/autokart/planner/status",
    "controller": "/autokart/controller/status",
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
        self.speed_mps: float | None = None
        self.speed_valid = False
        self.planner_decision: str | None = None
        self.pending_requests: list[str] = []
        self.cycle = 0

        self.status_pub = self.create_publisher(MissionDebug, "/autokart/safety/status", 10)
        self.dashboard_pub = self.create_publisher(MissionDebug, "/dashboard/debug", 10)
        self.request_pub = self.create_publisher(String, "/autokart/vcu/request", 10)

        self.create_subscription(VcuState, "/vcu/state", self._on_vcu, 10)
        self.create_subscription(ActuatorFeedback, "/vehicle/actuator_feedback", self._on_feedback, 10)
        self.create_subscription(String, "/autokart/safety/request", self._on_request, 10)
        for key, topic in STATUS_TOPICS.items():
            self.create_subscription(MissionDebug, topic, self._make_status_handler(key), 10)

        self.create_timer(0.1, self._tick)

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
        self.speed_mps = float(message.motor_speed_radps)
        self.speed_valid = bool(message.valid)

    def _make_status_handler(self, key: str):
        def handler(message: MissionDebug) -> None:
            try:
                payload = json.loads(message.payload_json)
            except (ValueError, TypeError):
                self.last_seen.pop(key, None)
                if key == "planner":
                    self.planner_decision = None
                return
            if payload.get("source") != "LIVE" or payload.get("valid") is not True:
                self.last_seen.pop(key, None)
            else:
                self._touch(key)
            if key == "planner":
                self.planner_decision = payload.get("decision")
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
            "schema_version": 1, "source": "LIVE", "valid": True,
            "reason_codes": [f"SAFETY_CORE_UNAVAILABLE:{self.load_error}"],
            "frame_id": "base_link",
            "sample_time_ns": self.get_clock().now().nanoseconds,
            "sequence": self.cycle, "state": "UNAVAILABLE", "arm_allowed": False,
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
        message.stage = "safety"
        self.dashboard_pub.publish(message)

    def _tick(self) -> None:
        if self.logic is None or self.core is None:
            self._publish_unavailable()
            return

        inputs = self.core.Inputs(
            ages={k: self._age(k) for k in ("vcu", "odom", "perception", "mapping", "planner", "controller")},
            vcu=self.vcu, speed_mps=self.speed_mps, speed_valid=self.speed_valid,
            planner_decision=self.planner_decision,
            calibration_verified=self.calibration_verified,
            vehicle_config_verified=self.vehicle_config_verified,
            braking_verified=self.braking_verified,
        )
        requests, self.pending_requests = self.pending_requests, []
        out = self.logic.step(time.monotonic(), inputs, requests)
        for event in out.events:
            self.get_logger().info(event)
        if out.vcu_request is not None:
            self.request_pub.publish(String(data=out.vcu_request))

        self.cycle += 1
        now_stamp = self.get_clock().now().to_msg()
        payload = {
            "schema_version": 1, "source": "LIVE", "valid": True,
            "reason_codes": out.reasons, "frame_id": "base_link",
            "sample_time_ns": self.get_clock().now().nanoseconds,
            "sequence": self.cycle, "state": out.state, "arm_allowed": out.arm_allowed,
            "vcu_heartbeat_ok": out.vcu_heartbeat_ok, "startup_ready": out.startup_ready,
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
        message.stage = "safety"
        self.dashboard_pub.publish(message)


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
