"""ROS transport for the VCU arming safety supervisor.

The only node that may request ARM of autokart_vehicle_bridge. Subscribes to real freshness/validity signals
across the pipeline (VCU telemetry, odometry, perception, mapping, planner, controller) plus the three
*_verified configuration flags (never set by this node -- read-only parameters a person sets after the bench
checklist) and a fresh manual power cut confirmation. Publishes its decision as a MissionDebug status
(/autokart/safety/status, consumed by autokart_vehicle_bridge to gate throttle) and forwards ARM/DIS/RST
requests to the bridge on /autokart/vcu/request. Receives bounded arm/stop/reset_fault/confirm_manual_power_cut
requests on /autokart/safety/request -- the ONLY channel through which the dashboard can influence the vehicle,
and even that only as a request this node evaluates, never a command it passes through.
"""
from __future__ import annotations

import json
import time


def main(args=None) -> None:
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from std_msgs.msg import String
        from autokart_msgs.msg import ActuatorFeedback, MissionDebug, VcuState
    except ImportError as exc:
        raise SystemExit(f"NOT_EXECUTED_EXTERNAL_DEPENDENCY: {exc}") from exc

    from .core import Config, Inputs, SafetyLogic, VcuView

    ALLOWED_REQUESTS = {"arm", "stop", "reset_fault", "confirm_manual_power_cut",
                       "demo_drive_start", "demo_drive_stop"}
    STATUS_TOPICS = {
        "perception": "/autokart/perception/status",
        "mapping": "/autokart/mapping/status",
        "planner": "/autokart/planner/status",
        "controller": "/autokart/controller/status",
        "odom": "/autokart/localization/status",
    }

    class VcuSafetyNode(Node):
        def __init__(self) -> None:
            super().__init__("autokart_vcu_safety")
            p = self.declare_parameter
            self.cfg = Config(
                overspeed_mps=float(p("overspeed_mps", 2.5).value),
                stop_ineffective_s=float(p("stop_ineffective_s", 4.0).value),
            )
            # Read-only: a person sets these (true) only after reviewing docs/BENCH_TESTS.md. This node never
            # writes them.
            self.calibration_verified = bool(p("calibration_verified", False).value)
            self.vehicle_config_verified = bool(p("vehicle_config_verified", False).value)
            self.braking_verified = bool(p("braking_verified", False).value)

            self.logic = SafetyLogic(self.cfg)
            self.last_seen: dict[str, float] = {}
            self.vcu = VcuView()
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
            self._touch("vcu")
            self.vcu = VcuView(state=str(message.state), estop_active=bool(message.emergency_stop_active),
                               estop_input_fitted=bool(message.estop_input_fitted),
                               active_faults=tuple(message.active_faults))

        def _on_feedback(self, message: ActuatorFeedback) -> None:
            self.speed_mps = float(message.motor_speed_radps)  # axle-derived linear speed, see the bridge
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

        def _tick(self) -> None:
            inputs = Inputs(
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

    rclpy.init(args=args)
    node = VcuSafetyNode()
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
