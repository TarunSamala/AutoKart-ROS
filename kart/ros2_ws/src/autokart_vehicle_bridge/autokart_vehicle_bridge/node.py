"""ROS entry point for the real VCU USB-serial bridge.

Single owner of the serial link and the single writer of CMD/ARM/DIS/RST -- no other node opens this port.
Subscribes to the existing disarmed Pure Pursuit candidate (``MotionCommand`` on
``/autokart/controller/motion_command_candidate``, see docs/NODE_CONTRACTS.md) and to a bounded arm/stop/reset
request topic written only by the VCU safety supervisor, never directly by the dashboard (hard rule). Publishes
``ActuatorFeedback`` and ``VcuState`` from real telemetry. No GPIO, no direct actuator topic, no Wi-Fi: every
actuation this node causes passes through a single CMD frame on this one serial link.
"""
from __future__ import annotations

import json
import time


def main(args=None) -> None:
    try:
        import rclpy
        import serial
        from builtin_interfaces.msg import Time as RosTime
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from std_msgs.msg import String
        from autokart_msgs.msg import ActuatorFeedback, MissionDebug, MotionCommand as RosMotionCommand, VcuState
    except ImportError as exc:
        raise SystemExit(f"NOT_EXECUTED_EXTERNAL_DEPENDENCY: {exc}") from exc

    from .core import (CommandGate, ThrottleConfig, ThrottleController, arm_request_approved,
                       brake_pct_for_candidate, raw_to_rad)
    from .demo_drive import DemoDriveController
    from .protocol import NAK_TEXT, PROTO_VERSION, build_cmd, build_simple, parse_frame, parse_message

    def ros_time(value: float) -> RosTime:
        stamp = RosTime()
        sec, nanosec = divmod(int(value * 1e9), 1_000_000_000)
        stamp.sec, stamp.nanosec = sec, nanosec
        return stamp

    class VehicleBridgeNode(Node):
        def __init__(self) -> None:
            super().__init__("autokart_vehicle_bridge")
            p = self.declare_parameter
            port = str(p("port", "/dev/ttyACM0").value)
            baud = int(p("baud", 115200).value)
            self.throttle_cfg = ThrottleConfig(
                throttle_v_max=float(p("throttle_v_max", 2.0).value),
                throttle_run_v=float(p("throttle_run_v", 2.0).value),
                volts_per_mps=float(p("volts_per_mps", 1.0).value),
            )
            self.throttle_ctl = ThrottleController(self.throttle_cfg)
            self.gate = CommandGate(self.throttle_cfg.throttle_v_max, float(p("cmd_timeout_s", 0.25).value))
            # MEASURE: AS5600 zero offset / scale for steering and brake-position readout.
            self.steer_zero_raw = int(p("steer_zero_raw", 2048).value)
            self.steer_rad_per_raw = float(p("steer_rad_per_raw", 0.001534).value)
            self.brake_zero_raw = int(p("brake_zero_raw", 2048).value)
            self.brake_rad_per_raw = float(p("brake_rad_per_raw", 0.001534).value)

            self.ser = None
            try:
                self.ser = serial.Serial(port, baud, timeout=0)
            except Exception as exc:  # noqa: BLE001 - any serial-open failure must stay non-fatal and fail-safe
                self.get_logger().error(f"VCU serial port {port} unavailable: {exc}")

            self.rx_buf = b""
            self.seq = 0
            self.last_tel = None
            self.last_tel_wall = None
            self.last_published_tel_seq = None
            self.safety_armed = False
            self.safety_arm_pending = False
            self.last_safety_wall = None
            self.last_hello_wall = None
            self.last_hello = None
            self.candidate = None          # (target_speed_mps, valid, wall_stamp, frame_stamp_ns)
            self.pending_request = None    # "ARM" | "DIS" | "RST"
            self.pending_request_wall = None
            self.demo_drive = DemoDriveController()  # DEMO DRIVE, not autonomy; gated by vcu_safety

            self.feedback_pub = self.create_publisher(ActuatorFeedback, "/vehicle/actuator_feedback", 10)
            self.vcu_pub = self.create_publisher(VcuState, "/vcu/state", 10)
            self.hello_pub = self.create_publisher(String, "/autokart/vcu/hello", 10)
            self.health_pub = self.create_publisher(String, "/autokart/vcu/health", 10)
            self.axle_pub = self.create_publisher(String, "/autokart/vcu/axle_counts", 10)
            self.trace_pub = self.create_publisher(String, "/autokart/latency/trace", 50)
            self.create_subscription(RosMotionCommand, "/autokart/controller/motion_command_candidate",
                                     self._on_candidate, 10)
            self.create_subscription(String, "/autokart/vcu/request", self._on_request, 10)
            self.create_subscription(MissionDebug, "/autokart/safety/status", self._on_safety_status, 10)
            self.create_subscription(String, "/autokart/demo_drive/control", self._on_demo_drive_control, 10)
            self.create_timer(0.05, self._tick)  # 20 Hz: send CMD, read telemetry, publish feedback/state

        def _on_demo_drive_control(self, message) -> None:
            """Published only by the dashboard gateway (backend), never the browser directly --
            same bounded-channel pattern as /autokart/vcu/request. Any message here, including a
            plain ping, counts as the backend-owned heartbeat (see demo_drive.py's module
            docstring for why this is deliberately NOT the only dead-man channel: firmware's own
            300 ms CMD_TIMEOUT_MS is independent and stricter)."""
            now = time.monotonic()
            self.demo_drive.heartbeat(now)
            try:
                payload = json.loads(message.data)
            except (ValueError, TypeError):
                return
            action = payload.get("action")
            if action == "drive":
                self.demo_drive.request_drive(now, str(payload.get("direction", "")),
                                              float(payload.get("duration_s", 0.0)))
            elif action == "stop":
                self.demo_drive.request_stop()

        def _on_candidate(self, message) -> None:
            stamp_ns = int(message.timestamp.sec) * 1_000_000_000 + int(message.timestamp.nanosec)
            self.candidate = (float(message.target_speed_mps), bool(message.valid), time.monotonic(), stamp_ns)

        def _on_request(self, message) -> None:
            value = str(message.data).strip().upper()
            if value in ("ARM", "DIS", "RST"):
                self.pending_request = value
                self.pending_request_wall = time.monotonic()
            else:
                self.get_logger().warning(f"Ignoring unknown VCU request: {message.data!r}")

        def _on_safety_status(self, message) -> None:
            try:
                payload = json.loads(message.payload_json)
            except (ValueError, TypeError):
                self.safety_armed = False
                self.safety_arm_pending = False
                self.demo_drive.set_active(False)
                return
            valid = (payload.get("schema_version") == 1 and payload.get("source") == "LIVE"
                     and bool(payload.get("startup_ready")) and not bool(payload.get("stop_latched")))
            self.last_safety_wall = time.monotonic()
            self.safety_armed = valid and str(payload.get("state")) == "ARMED"
            self.safety_arm_pending = valid and str(payload.get("state")) == "ARMING"
            self.demo_drive.set_active(self.safety_armed and bool(payload.get("demo_drive_active")))

        def _poll_serial(self) -> None:
            if self.ser is None:
                return
            try:
                chunk = self.ser.read(4096)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"VCU serial read failed: {exc}")
                self.ser = None
                return
            if not chunk:
                return
            self.rx_buf += chunk
            if len(self.rx_buf) > 8192:
                self.rx_buf = self.rx_buf[-8192:]  # drop stale backlog rather than growing unbounded
            while b"\n" in self.rx_buf:
                line, self.rx_buf = self.rx_buf.split(b"\n", 1)
                payload = parse_frame(line)
                if payload is None:
                    continue
                parsed = parse_message(payload)
                if parsed is None:
                    continue
                kind, value = parsed
                if kind == "TEL":
                    self.last_tel = value
                    self.last_tel_wall = time.monotonic()
                    if value.fwseq != self.last_published_tel_seq:
                        self.last_published_tel_seq = value.fwseq
                        stamp_ns = time.monotonic_ns()
                        self.axle_pub.publish(String(data=json.dumps({"stamp_ns": stamp_ns,
                            "fwseq": value.fwseq, "counts": value.counts if value.counts >= 0 else None,
                            "dac_mv": value.dac_mv, "ack_seq": value.ack_seq,
                            "speed_mmps": value.speed_mmps if value.speed_mmps >= 0 else None}, sort_keys=True)))
                        self.trace_pub.publish(String(data=json.dumps({"event": "vcu_tel",
                            "time_ns": stamp_ns, "ack_seq": value.ack_seq, "dac_mv": value.dac_mv,
                            "state": value.state_name, "en": bool(value.en), "brk": bool(value.brk),
                            "fault": value.fault}, sort_keys=True)))
                elif kind == "HELLO":
                    self.last_hello = value
                    self.last_hello_wall = time.monotonic()
                elif kind == "NAK":
                    _, code = value
                    self.get_logger().warning(f"VCU NAK: {NAK_TEXT.get(code, code)}")

        def _send(self, data: bytes) -> None:
            if self.ser is None:
                return
            try:
                self.ser.write(data)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"VCU serial write failed: {exc}")
                self.ser = None

        def _tick(self) -> None:
            self._poll_serial()
            now = time.monotonic()
            if self.last_safety_wall is None or now - self.last_safety_wall > 0.3:
                self.safety_armed = False
                self.safety_arm_pending = False
                self.demo_drive.set_active(False)
            if self.last_hello is not None and self.last_hello_wall is not None and now - self.last_hello_wall <= 6.0:
                self.hello_pub.publish(String(data=f"{self.last_hello[0]},{self.last_hello[1]}"))
            tel = self.last_tel
            protocol_ok = (self.last_hello is not None and self.last_hello[1] == PROTO_VERSION and
                           self.last_hello_wall is not None and now - self.last_hello_wall <= 6.0)
            tel_fresh = bool(protocol_ok and self.last_tel_wall is not None and now - self.last_tel_wall <= 0.3)
            vcu_armed = tel_fresh and tel is not None and tel.state == 1
            health = {"telemetry_fresh": tel_fresh, "state": tel.state_name if tel_fresh and tel else "UNKNOWN",
                      "dac_ok": bool(tel.dac_ok) if tel_fresh and tel else False,
                      "estop_ok": bool(tel.estop_ok) if tel_fresh and tel else False,
                      "estop_input_fitted": bool(tel.estop_input_fitted) if tel_fresh and tel else False,
                      "estop_status": ("OK" if tel.estop_ok else "PRESSED") if tel_fresh and tel and tel.estop_input_fitted else "NOT FITTED - MANUAL POWER CUT ONLY",
                      "protocol_ok": protocol_ok,
                      "hello_fresh": bool(self.last_hello_wall is not None and now - self.last_hello_wall <= 6.0),
                      "speed_measured": bool(tel_fresh and tel is not None and tel.speed_mmps >= 0)}
            self.health_pub.publish(String(data=json.dumps(health, sort_keys=True)))

            self.seq += 1
            if self.pending_request is not None:
                if self.pending_request == "ARM" and (not protocol_ok or not arm_request_approved(
                        now, self.safety_arm_pending, self.last_safety_wall, self.pending_request_wall)):
                    if self.pending_request_wall is None or now - self.pending_request_wall > 0.3:
                        self.get_logger().warning("Dropping ARM without fresh supervisor ARMING approval")
                        self.pending_request = None
                        self.pending_request_wall = None
                else:
                    self._send(build_simple(self.pending_request, self.seq))
                    self.pending_request = None
                    self.pending_request_wall = None

            # DEMO DRIVE and autonomy are mutually exclusive sources into the SAME CommandGate below
            # (safety_armed + vcu_armed + fresh command, unchanged either way). self.demo_drive.active
            # mirrors autokart_vcu_safety's demo_drive_active, which is only ever true already-ARMED.
            if self.demo_drive.active:
                demo_out = self.demo_drive.compute(now)
                throttle_v = demo_out.throttle_mv / 1000.0
                brake_pct = demo_out.brake_pct
                cmd_wall = now if demo_out.throttle_mv > 0 else None
                self.throttle_ctl.reset()
            else:
                target_mps, valid, cmd_wall = (0.0, False, None)
                if self.candidate is not None:
                    target_mps, valid, cmd_wall, _ = self.candidate
                speed_measured = bool(tel_fresh and tel is not None and tel.speed_mmps >= 0)
                measured_mps = tel.speed_mmps / 1000.0 if speed_measured else 0.0
                valid = valid and speed_measured
                throttle_v = self.throttle_ctl.step(now, target_mps if valid else 0.0, measured_mps)
                brake_pct = brake_pct_for_candidate(valid, target_mps)
            gated = self.gate.compute(now, self.safety_armed, vcu_armed, throttle_v, brake_pct, cmd_wall)
            if gated.reason != "pass":
                self.throttle_ctl.reset()
            self._send(build_cmd(self.seq, gated.throttle_mv, 0, gated.brake_pct))
            self.trace_pub.publish(String(data=json.dumps({"event": "bridge_tx", "time_ns": time.monotonic_ns(),
                "seq": self.seq & 0xFFFFFFFF, "requested_dac_mv": gated.throttle_mv,
                "frame_stamp_ns": self.candidate[3] if self.candidate is not None else None}, sort_keys=True)))

            self._publish_feedback(tel, tel_fresh)
            self._publish_vcu_state(tel, tel_fresh)

        def _publish_feedback(self, tel, tel_fresh: bool) -> None:
            message = ActuatorFeedback()
            stamp = ros_time(time.monotonic())
            message.measurement_time = stamp
            message.delivery_time = stamp
            if tel_fresh and tel is not None:
                steer_rad = raw_to_rad(tel.steer_raw, self.steer_zero_raw, self.steer_rad_per_raw)
                message.road_wheel_angle_rad = float(steer_rad) if steer_rad is not None else 0.0
                message.steering_actuator_position_rad = message.road_wheel_angle_rad
                # No measured brake-travel scale yet (MEASURE): this is the raw AS5600 angle, not metres, and
                # is only meaningful once brake_rad_per_raw is calibrated against the actual linkage.
                brake_rad = raw_to_rad(tel.brake_raw, self.brake_zero_raw, self.brake_rad_per_raw)
                message.brake_actuator_position_m = float(brake_rad) if brake_rad is not None else 0.0
                # No separate motor-shaft encoder on this hardware: this is the rear axle encoder's linear
                # speed (m/s), reused in the "motor_speed_radps" field name because no closer-fitting field
                # exists in ActuatorFeedback -- it is NOT a motor-shaft angular velocity.
                message.motor_speed_radps = float(tel.speed_mmps) / 1000.0 if tel.speed_mmps >= 0 else float("nan")
                message.valid = bool(tel.dac_ok) and steer_rad is not None and tel.speed_mmps >= 0
            else:
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
                # A 2 V DAC ceiling is not a measured ground-speed ceiling.
                message.active_speed_limit_mps = 0.0 if tel.speed_mmps < 0 else self.throttle_cfg.throttle_v_max / max(
                    self.throttle_cfg.volts_per_mps, 1e-6)
                faults = tel.fault_names()
                message.limiting_reason = ",".join(faults) if faults else "NONE"
                message.active_faults = faults
            message.reset_epoch = 0
            self.vcu_pub.publish(message)

    rclpy.init(args=args)
    node = VehicleBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node.ser is not None:
                node._send(build_simple("DIS", node.seq + 1))  # best-effort disarm on shutdown
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
