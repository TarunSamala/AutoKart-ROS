"""ROS 2 node: USB serial <-> VCU. Publishes telemetry + odometry, gates commands."""
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from autokart_msgs.msg import SafetyState, VehicleCommand, VehicleOdom, VehicleTelemetry

from .core import CommandGate, Odometry, raw_to_deg
from .protocol import (NAK_TEXT, PROTO_VERSION, build_cmd, build_simple, parse_frame, parse_message)

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover
    serial = None


class VehicleBridge(Node):
    def __init__(self):
        super().__init__("autokart_vehicle_bridge")
        p = self.declare_parameter
        self.port = p("port", "/dev/ttyACM0").value
        self.baud = int(p("baud", 115200).value)
        self.wheelbase = float(p("wheelbase_m", 1.0).value)
        self.steer_zero_raw = int(p("steer_zero_raw", 2048).value)
        self.steer_deg_per_raw = float(p("steer_deg_per_raw", 0.0879).value)  # 360/4096 until geared/calibrated
        self.brake_zero_raw = int(p("brake_zero_raw", 0).value)
        self.brake_deg_per_raw = float(p("brake_deg_per_raw", 0.0879).value)
        self.throttle_v_max = float(p("throttle_v_max", 2.0).value)
        self.mode = str(p("data_mode", "LIVE").value)

        self.gate = CommandGate(self.throttle_v_max)
        self.odom = Odometry(self.wheelbase)
        self.seq = 1
        self.lock = threading.Lock()
        self.ser = None
        self.fw_version = ""
        self.protocol_ok = False
        self.last_tel = None
        self.last_tel_t = None
        self.prev_tel_ms = None
        self.cmd = None
        self.cmd_stamp = None
        self.safety_state = "DISARMED"
        self.request = None          # "ARM" | "DISARM" | "RESET" (one-shot, retried briefly)
        self.request_until = 0.0

        self.pub_tel = self.create_publisher(VehicleTelemetry, "/autokart/vehicle_telemetry", 10)
        self.pub_odom = self.create_publisher(VehicleOdom, "/autokart/odom", 10)
        self.create_subscription(VehicleCommand, "/autokart/vehicle_command", self.on_cmd, 10)
        self.create_subscription(SafetyState, "/autokart/safety/state", self.on_safety, 10)
        self.create_subscription(String, "/autokart/bridge/vcu_request", self.on_request, 10)
        self.create_timer(0.05, self.tx_tick)  # 20 Hz command / heartbeat

        self.open_port()
        threading.Thread(target=self.rx_loop, daemon=True).start()

    # ---------- serial ----------
    def open_port(self):
        if serial is None:
            self.get_logger().error("pyserial not installed")
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05, write_timeout=0.1)
            self.get_logger().info(f"opened {self.port}")
        except Exception as e:  # noqa: BLE001
            self.ser = None
            self.get_logger().error(f"cannot open {self.port}: {e}")

    def write(self, data: bytes):
        if self.ser is None:
            return
        try:
            self.ser.write(data)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"serial write failed: {e}")
            self.ser = None

    def rx_loop(self):
        buf = b""
        while rclpy.ok():
            if self.ser is None:
                time.sleep(1.0)
                self.open_port()
                continue
            try:
                chunk = self.ser.read(256)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"serial read failed: {e}")
                self.ser = None
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                payload = parse_frame(line)
                if payload is None:
                    continue
                msg = parse_message(payload)
                if msg:
                    self.handle(*msg)
            if len(buf) > 1024:
                buf = b""

    def handle(self, kind, val):
        if kind == "HELLO":
            self.fw_version, proto = val
            self.protocol_ok = proto == PROTO_VERSION
            if not self.protocol_ok:
                self.get_logger().error(f"protocol mismatch: firmware {proto}, bridge {PROTO_VERSION}")
        elif kind == "NAK":
            seq, code = val
            self.get_logger().warn(f"VCU refused request {seq}: {NAK_TEXT.get(code, code)}")
        elif kind == "TEL":
            self.on_tel(val)

    def on_tel(self, t):
        now = self.get_clock().now()
        steer = raw_to_deg(t.steer_raw, self.steer_zero_raw, self.steer_deg_per_raw)
        brake = raw_to_deg(t.brake_raw, self.brake_zero_raw, self.brake_deg_per_raw)
        speed = t.speed_mmps / 1000.0
        if self.prev_tel_ms is not None:
            self.odom.update(((t.t_ms - self.prev_tel_ms) & 0xFFFFFFFF) / 1000.0, speed, steer)
        self.prev_tel_ms = t.t_ms
        with self.lock:
            self.last_tel = t
            self.last_tel_t = time.monotonic()

        m = VehicleTelemetry()
        m.header.stamp = now.to_msg()
        m.header.frame_id = "vcu"
        m.state, m.state_name, m.fault = t.state, t.state_name, t.fault
        m.estop_ok, m.steer_pwm_ok, m.brake_pwm_ok = t.estop_ok, t.steer_pwm_ok, t.brake_pwm_ok
        m.axle_motion_seen, m.dac_ok, m.en, m.brk = t.axle_motion_seen, t.dac_ok, t.en, t.brk
        m.dac_v = t.dac_mv / 1000.0
        m.axle_counts = int(t.counts)
        m.speed_mps = speed
        m.steer_valid = steer is not None
        m.steer_deg = float(steer) if steer is not None else 0.0
        m.brake_valid = brake is not None
        m.brake_deg = float(brake) if brake is not None else 0.0
        m.link_age_ms = float(t.link_age_ms)
        m.fw_version = self.fw_version
        m.protocol_ok = self.protocol_ok
        self.pub_tel.publish(m)

        o = VehicleOdom()
        o.header.stamp = m.header.stamp
        o.header.frame_id = "odom"
        o.x, o.y, o.yaw_rad = self.odom.x, self.odom.y, self.odom.yaw
        o.speed_mps = speed
        o.yaw_rate_dps = float(self.odom.yaw_rate * 57.29578)
        o.axle_counts = int(t.counts)
        o.status = self.odom.status
        self.pub_odom.publish(o)

    # ---------- inputs ----------
    def on_cmd(self, m: VehicleCommand):
        self.cmd = (m.throttle_v, m.steer_deg, m.brake_pct)
        self.cmd_stamp = time.monotonic()

    def on_safety(self, m: SafetyState):
        self.safety_state = m.state

    def on_request(self, m: String):
        if m.data in ("ARM", "DISARM", "RESET"):
            self.request = m.data
            self.request_until = time.monotonic() + 1.5

    # ---------- 20 Hz transmit ----------
    def tx_tick(self):
        now = time.monotonic()
        with self.lock:
            tel, tel_t = self.last_tel, self.last_tel_t
        vcu_armed = bool(tel and tel.state == 1 and tel_t and now - tel_t < 0.3)
        out = self.gate.compute(now, self.safety_state == "ARMED", vcu_armed, self.cmd, self.cmd_stamp)
        self.write(build_cmd(self.seq, out.throttle_mv, out.steer_cdeg, out.brake_pct))
        self.seq += 1

        # Safety fault on the laptop side always disarms the VCU.
        if self.safety_state == "FAULT" and tel and tel.state == 1:
            self.write(build_simple("DIS", self.seq)); self.seq += 1

        if self.request:
            if now > self.request_until:
                self.request = None
            else:
                kind = {"ARM": "ARM", "DISARM": "DIS", "RESET": "RST"}[self.request]
                if self.request == "ARM" and vcu_armed:
                    self.request = None
                else:
                    self.write(build_simple(kind, self.seq)); self.seq += 1
                    if self.request in ("DISARM", "RESET") and tel and (
                            (self.request == "DISARM" and tel.state != 1) or (self.request == "RESET" and tel.state == 0)):
                        self.request = None


def main():
    rclpy.init()
    node = VehicleBridge()
    try:
        rclpy.spin(node)
    finally:
        # Best effort: tell the VCU to disarm on shutdown.
        node.write(build_simple("DIS", node.seq))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
