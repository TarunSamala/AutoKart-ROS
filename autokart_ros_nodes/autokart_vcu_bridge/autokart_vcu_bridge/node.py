"""autokart_vcu_bridge: sole USB/serial VCU endpoint owner (standalone library node).

Relays an ALREADY-DECIDED candidate motion command (autokart_msgs/MotionCommand)
into a throttle voltage / brake percentage over the AutoKart laptop<->VCU wire
protocol, and publishes ActuatorFeedback/VcuState from the VCU's telemetry.

This node does NOT decide motion authority: it requires an external
`safety_armed` input (a placeholder; node 10 / autokart_safety, built
concurrently in this library, publishes MissionDebug JSON that this node does
not yet consume -- see CONTRACT.md "Relationship to node 10") and
never fabricates that authorization itself. It never actuates
steering_angle_rad (observed only, per docs/VCU_PROTOCOL.md). No raw PWM/DAC/
stepper control and no E-stop override happen here -- the VCU firmware's own
SAFE/ARMED/FAULT state machine remains the actual safety authority.

Reuses, lazily from the `algorithm_package_path` ROS parameter (never
hardcoded, same pattern as autokart_hp60c's HP60CBackend):
- `autokart_vehicle_bridge.protocol` (CRC16 framing, CMD/ARM/DIS/RST wire
  encoding, TEL/HELLO/NAK decoding) -- inside backends.py's SerialVcuBackend.
- `autokart_vehicle_bridge.core` (ThrottleController, CommandGate,
  raw_to_rad, brake_pct_for_candidate) -- here in node.py, since this is pure
  control-law/gating logic with no hardware dependency, used identically
  regardless of which backend (synthetic or serial) is selected.

If `algorithm_package_path` does not resolve (e.g. this package exported and
run standalone with no access to the source repository), the node still
starts and still relays telemetry, but falls back to the only safe
degenerate policy it is willing to invent itself: throttle always 0 mV, brake
always 100% -- never a silent default, always reported via `/vcu_bridge/status`
as `CORE_LOGIC_UNAVAILABLE`. It never reimplements the real control law.
"""
from __future__ import annotations

import importlib
import sys
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from builtin_interfaces.msg import Time as RosTime
from autokart_msgs.msg import ActuatorFeedback, MotionCommand, SensorDeliveryMetadata, VcuState

from .backends import build_backend, VcuBackendError

NAK_TEXT = {1: "wrong state", 2: "E-stop active", 3: "DAC not ready", 4: "command link not fresh",
            5: "throttle not zero", 6: "fault cause still present", 7: "output wiring unverified"}


def ros_time(value: float) -> RosTime:
    stamp = RosTime()
    sec, nanosec = divmod(int(value * 1e9), 1_000_000_000)
    stamp.sec, stamp.nanosec = sec, nanosec
    return stamp


class VcuBridgeNode(Node):
    def __init__(self, start_timer: bool = True, **node_kwargs):
        super().__init__("autokart_vcu_bridge", **node_kwargs)
        p = self.declare_parameter
        p("backend", "synthetic")
        p("algorithm_package_path", "")
        p("serial_port", "/dev/ttyACM0")
        p("serial_baud", 115200)
        p("tick_hz", 20.0)
        p("throttle_v_max", 2.0)
        p("throttle_run_v", 2.0)
        p("volts_per_mps", 1.0)
        p("cmd_timeout_s", 0.25)
        p("safety_timeout_s", 0.3)
        p("steer_zero_raw", 2048)
        p("steer_rad_per_raw", 0.001534)
        p("brake_zero_raw", 2048)
        p("brake_rad_per_raw", 0.001534)
        p("candidate_topic", "/autokart/controller/motion_command_candidate")
        p("safety_armed_topic", "/vcu_bridge/safety_armed")
        p("request_topic", "/vcu_bridge/request")

        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)
        self.safety_timeout_s = float(self.get_parameter("safety_timeout_s").value)
        self.steer_zero_raw = int(self.get_parameter("steer_zero_raw").value)
        self.steer_rad_per_raw = float(self.get_parameter("steer_rad_per_raw").value)
        self.brake_zero_raw = int(self.get_parameter("brake_zero_raw").value)
        self.brake_rad_per_raw = float(self.get_parameter("brake_rad_per_raw").value)

        self.feedback_pub = self.create_publisher(ActuatorFeedback, "/vehicle/actuator_feedback", 10)
        self.vcu_pub = self.create_publisher(VcuState, "/vcu/state", 10)
        self.status_pub = self.create_publisher(SensorDeliveryMetadata, "/vcu_bridge/status", 10)
        self.create_subscription(MotionCommand, str(self.get_parameter("candidate_topic").value),
                                  self._on_candidate, 10)
        self.create_subscription(Bool, str(self.get_parameter("safety_armed_topic").value),
                                  self._on_safety_armed, 10)
        self.create_subscription(String, str(self.get_parameter("request_topic").value),
                                  self._on_request, 10)

        self.seq = 0
        self.cumulative_captured = 0
        self.cumulative_delivered = 0
        self.cumulative_dropped = 0
        self._last_fault = None
        self.last_tel = None
        self.last_tel_wall = None
        self.last_hello = None
        self.last_hello_wall = None
        self.candidate = None            # (target_speed_mps, valid, wall_stamp, expiry_ns)
        self.safety_armed_value = False
        self.last_safety_wall = None
        self.pending_request = None

        algorithm_package_path = str(self.get_parameter("algorithm_package_path").value)
        self._core, self._core_error = self._try_import_core(algorithm_package_path)
        if self._core is not None:
            self.throttle_cfg = self._core.ThrottleConfig(
                throttle_v_max=float(self.get_parameter("throttle_v_max").value),
                throttle_run_v=float(self.get_parameter("throttle_run_v").value),
                volts_per_mps=float(self.get_parameter("volts_per_mps").value),
            )
            self.throttle_ctl = self._core.ThrottleController(self.throttle_cfg)
            self.gate = self._core.CommandGate(self.throttle_cfg.throttle_v_max, self.cmd_timeout_s)
        else:
            self.get_logger().warning(
                f"autokart_vcu_bridge: real control law unavailable ({self._core_error}); "
                "falling back to the only safe degenerate policy this node invents itself: "
                "throttle always 0 mV, brake always 100%. See CONTRACT.md.")

        backend_name = str(self.get_parameter("backend").value)
        try:
            self.backend = build_backend(
                backend_name,
                algorithm_package_path=algorithm_package_path,
                port=str(self.get_parameter("serial_port").value),
                baud=int(self.get_parameter("serial_baud").value),
            )
            self.backend.start()
            self.startup_error = None
        except VcuBackendError as exc:
            self.backend = None
            self.startup_error = str(exc)
            self.get_logger().error(f"autokart_vcu_bridge startup failed: {self.startup_error}")

        self._timer = None
        if start_timer:
            self.start_timer()

    def _try_import_core(self, algorithm_package_path: str):
        if not algorithm_package_path:
            return None, "CORE_LOGIC_PATH_NOT_CONFIGURED"
        src = Path(algorithm_package_path).expanduser()
        if not src.is_dir():
            return None, f"DEPENDENCY_PATH_NOT_FOUND:{src}"
        src_str = str(src)
        if src_str not in sys.path:
            sys.path.insert(0, src_str)
        try:
            return importlib.import_module("autokart_vehicle_bridge.core"), None
        except Exception as exc:  # noqa: BLE001 - exercised via test_backends' fault case
            return None, f"SDK_ADAPTER_IMPORT_FAILED:{type(exc).__name__}:{exc}"

    def start_timer(self):
        """Split out from __init__, same reasoning as the other nodes in this
        library: lets tests construct the node, swap in a fault-injecting
        backend, then start ticking deterministically."""
        tick_hz = max(1.0, float(self.get_parameter("tick_hz").value))
        self._timer = self.create_timer(1.0 / tick_hz, self._tick)

    def _on_candidate(self, message: MotionCommand) -> None:
        expiry_ns = int(message.expiry.sec) * 1_000_000_000 + int(message.expiry.nanosec)
        self.candidate = (float(message.target_speed_mps), bool(message.valid), time.monotonic(), expiry_ns)

    def _on_safety_armed(self, message: Bool) -> None:
        self.safety_armed_value = bool(message.data)
        self.last_safety_wall = time.monotonic()

    def _on_request(self, message: String) -> None:
        value = str(message.data).strip().upper()
        if value in ("ARM", "DIS", "RST"):
            self.pending_request = value
        else:
            self.get_logger().warning(f"Ignoring unknown VCU request: {message.data!r}")

    def _tick(self) -> None:
        self.seq += 1
        now_wall = time.monotonic()

        if self.backend is None:
            self._publish_fault(self.startup_error or "BACKEND_UNAVAILABLE")
            self._publish_feedback(None, False)
            self._publish_vcu_state(None, False)
            return

        backend_fault = None
        try:
            events = self.backend.poll()
        except VcuBackendError as exc:
            backend_fault = str(exc)
            events = []
        except Exception as exc:  # noqa: BLE001 - must surface as a reported fault, never kill the timer
            backend_fault = f"UNEXPECTED_BACKEND_ERROR:{type(exc).__name__}:{exc}"
            events = []

        for kind, value in events:
            if kind == "TEL":
                self.last_tel = value
                self.last_tel_wall = now_wall
                # ack_seq/link_age_ms have no home in any typed message this node publishes (see
                # CONTRACT.md "Known, documented interface gaps") -- logged for a human/bag recorder
                # rather than silently dropped or shoehorned into an unrelated field.
                self.get_logger().debug(
                    f"VCU TEL: ack_seq={value.ack_seq} link_age_ms={value.link_age_ms}")
            elif kind == "HELLO":
                self.last_hello = value
                self.last_hello_wall = now_wall
            elif kind == "NAK":
                _, code = value
                self.get_logger().warning(f"VCU NAK: {NAK_TEXT.get(code, code)}")

        if self.pending_request is not None:
            try:
                self.backend.send_request(self.seq, self.pending_request)
            except VcuBackendError as exc:
                backend_fault = backend_fault or str(exc)
            self.pending_request = None

        tel = self.last_tel
        tel_fresh = bool(tel is not None and self.last_tel_wall is not None
                          and now_wall - self.last_tel_wall <= 0.3)
        vcu_armed = tel_fresh and tel is not None and tel.state == 1
        safety_armed = bool(self.safety_armed_value and self.last_safety_wall is not None
                             and now_wall - self.last_safety_wall <= self.safety_timeout_s)

        target_mps, cmd_wall = 0.0, None
        candidate_valid = False
        if self.candidate is not None:
            target_mps, raw_valid, cmd_wall, expiry_ns = self.candidate
            now_ros_ns = self.get_clock().now().nanoseconds
            expiry_ok = (expiry_ns == 0) or (now_ros_ns <= expiry_ns)
            candidate_valid = raw_valid and expiry_ok

        if self._core is not None:
            speed_measured = bool(tel_fresh and tel is not None and tel.speed_mmps >= 0)
            measured_mps = tel.speed_mmps / 1000.0 if speed_measured else 0.0
            valid_for_throttle = candidate_valid and speed_measured
            throttle_v = self.throttle_ctl.step(now_wall, target_mps if valid_for_throttle else 0.0,
                                                 measured_mps)
            brake_pct = self._core.brake_pct_for_candidate(valid_for_throttle, target_mps)
            gated = self.gate.compute(now_wall, safety_armed, vcu_armed, throttle_v, brake_pct, cmd_wall)
            if gated.reason != "pass":
                self.throttle_ctl.reset()
            throttle_mv, brake_pct_out, gate_reason = gated.throttle_mv, gated.brake_pct, gated.reason
        else:
            throttle_mv, brake_pct_out, gate_reason = 0, 100, "CORE_LOGIC_UNAVAILABLE"

        try:
            self.backend.send_command(self.seq, throttle_mv, brake_pct_out)
        except VcuBackendError as exc:
            backend_fault = backend_fault or str(exc)

        self._publish_feedback(tel, tel_fresh)
        self._publish_vcu_state(tel, tel_fresh)

        if backend_fault is not None:
            self.cumulative_dropped += 1
            self._publish_fault(backend_fault)
        else:
            self.cumulative_captured += 1
            self.cumulative_delivered += 1
            if self._core is not None:
                self._publish_status("OK", "NONE", True)
            else:
                self._publish_status("INVALID", f"CORE_LOGIC_UNAVAILABLE:{self._core_error}", True)
            self._last_fault = None

    def _publish_fault(self, reason: str) -> None:
        if reason != self._last_fault:
            self.get_logger().warning(f"autokart_vcu_bridge fault: {reason}")
        self._last_fault = reason
        self._publish_status("INVALID", reason, False)

    def _publish_status(self, quality: str, fault_state: str, payload_present: bool) -> None:
        msg = SensorDeliveryMetadata()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "vcu_bridge"
        msg.sensor_id = "vcu_bridge"
        msg.sequence = self.seq
        msg.quality = quality
        msg.fault_state = fault_state
        msg.payload_present = payload_present
        msg.cumulative_captured = self.cumulative_captured
        msg.cumulative_delivered = self.cumulative_delivered
        msg.cumulative_dropped = self.cumulative_dropped
        self.status_pub.publish(msg)

    def _publish_feedback(self, tel, tel_fresh: bool) -> None:
        message = ActuatorFeedback()
        stamp = ros_time(time.monotonic())
        message.measurement_time = stamp
        message.delivery_time = stamp
        if tel_fresh and tel is not None and self._core is not None:
            steer_rad = self._core.raw_to_rad(tel.steer_raw, self.steer_zero_raw, self.steer_rad_per_raw)
            message.road_wheel_angle_rad = float(steer_rad) if steer_rad is not None else 0.0
            message.steering_actuator_position_rad = message.road_wheel_angle_rad
            # No measured brake-travel scale (MEASURE, see config/vcu_bridge_live.yaml): raw AS5600
            # angle, not metres -- same caveat the real autokart_vehicle_bridge node documents.
            brake_rad = self._core.raw_to_rad(tel.brake_raw, self.brake_zero_raw, self.brake_rad_per_raw)
            message.brake_actuator_position_m = float(brake_rad) if brake_rad is not None else 0.0
            # No separate motor-shaft encoder on this hardware: rear axle encoder's linear speed
            # (m/s), reused in "motor_speed_radps" because no closer-fitting field exists in
            # ActuatorFeedback -- NOT a motor-shaft angular velocity. Same documented reuse as the
            # real autokart_vehicle_bridge node (see docs/VCU_PROTOCOL.md).
            message.motor_speed_radps = float(tel.speed_mmps) / 1000.0 if tel.speed_mmps >= 0 else float("nan")
            message.valid = bool(tel.dac_ok) and steer_rad is not None and tel.speed_mmps >= 0
        else:
            # Either no fresh telemetry, or the real conversion law (core.raw_to_rad) is unavailable:
            # never reimplement it locally, report invalid instead.
            message.valid = False
        self.feedback_pub.publish(message)

    def _publish_vcu_state(self, tel, tel_fresh: bool) -> None:
        message = VcuState()
        message.timestamp = ros_time(time.monotonic())
        if not tel_fresh or tel is None:
            message.state = "UNKNOWN"
            message.motor_enable = False
            message.contactor_open = True
            message.emergency_stop_active = True
            message.estop_input_fitted = False
            message.active_speed_limit_mps = 0.0
            message.limiting_reason = "NO_VCU_TELEMETRY"
            message.active_faults = []
        else:
            message.state = tel.state_name
            message.motor_enable = bool(tel.en)
            message.contactor_open = not bool(tel.en)
            message.estop_input_fitted = bool(tel.estop_input_fitted)
            message.emergency_stop_active = bool(tel.estop_input_fitted and not tel.estop_ok)
            if self._core is not None and tel.speed_mmps >= 0:
                message.active_speed_limit_mps = self.throttle_cfg.throttle_v_max / max(
                    self.throttle_cfg.volts_per_mps, 1e-6)
            else:
                message.active_speed_limit_mps = 0.0
            faults = tel.fault_names()
            message.limiting_reason = ",".join(faults) if faults else "NONE"
            message.active_faults = faults
        message.reset_epoch = 0
        self.vcu_pub.publish(message)

    def destroy_node(self):
        if self._timer is not None:
            self._timer.cancel()
        if self.backend is not None:
            # Best-effort disarm on shutdown, same as the reference
            # autokart_vehicle_bridge node's main() (found missing in this
            # node's senior-review pass). The firmware's own 300 ms
            # LINK_TIMEOUT is the independent backstop if this write fails.
            try:
                self.backend.send_request(self.seq + 1, "DIS")
            except Exception as exc:  # noqa: BLE001 - shutdown must not raise
                self.get_logger().warning(f"autokart_vcu_bridge: best-effort DIS on shutdown failed: {exc}")
            try:
                self.backend.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VcuBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Humble's default SIGINT handler shuts the context down itself and
        # spin() then raises ExternalShutdownException; calling
        # rclpy.shutdown() again unguarded raised RCLError on every Ctrl-C
        # (found during this node's live clean-workspace validation run).
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
