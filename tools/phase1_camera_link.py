#!/usr/bin/env python3
"""Send qualified stop-cone observations from ROS to the ESP32 over USB.

This Phase 1 link sends OBS frames only. It never sends ARM, CMD, RST or a
throttle value. The matching firmware ACKs observations and stays arming-locked.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "kart", "ros2_ws", "src", "autokart_vehicle_bridge"))
from autokart_vehicle_bridge.protocol import build_observation, parse_frame, parse_message  # noqa: E402
from cone_gap_probe import evaluate  # noqa: E402

EXPECTED_FIRMWARE = "0.2.2-no-estop-input-locked"


def decision_code(row: dict) -> int:
    return {"STOP": 0, "OBSERVE_ONLY": 1, "AT_OR_INSIDE_TARGET": 2}[row["decision"]]


def fields(row: dict) -> tuple[int, int, int, int, int]:
    stamp_ms = max(0, int(row["stamp_ns"]) // 1_000_000) % 0x7FFFFFFF
    forward = row["cone_forward_m"]
    gap = row["bumper_to_center_m"]
    return (stamp_ms, int(row["cone_id"]) if forward is not None else -1,
            round(forward * 1000) if forward is not None else -1,
            round(gap * 1000) if gap is not None else -1, decision_code(row))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    transport = parser.add_mutually_exclusive_group(required=True)
    transport.add_argument("--port", help="ESP32 USB serial port")
    transport.add_argument("--dry-run", action="store_true", help="print/log OBS without opening serial")
    parser.add_argument("--cone-id", type=int, required=True)
    parser.add_argument("--output", default=os.path.join(ROOT, "results", "phase1_camera_link.csv"))
    args = parser.parse_args()
    if args.cone_id < 0:
        parser.error("--cone-id must be nonnegative")
    try:
        import serial
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from autokart_msgs.msg import ConeArray, MissionDebug
    except ImportError as exc:
        parser.error(f"serial or ROS overlay unavailable: {exc}")

    def ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    port = None
    try:
        if not args.dry_run:
            port = serial.Serial()
            port.port = args.port
            port.baudrate = 115200
            port.timeout = 0.1
            port.write_timeout = 0.1
            port.dtr = False
            port.rts = False
            port.open()
            print("Waiting for locked VCU HELLO; no observations sent yet.")
            deadline = time.monotonic() + 8.0
            firmware_ok = False
            while time.monotonic() < deadline:
                payload = parse_frame(port.readline())
                parsed = parse_message(payload) if payload is not None else None
                if parsed and parsed[0] == "HELLO":
                    fw, proto = parsed[1]
                    if (fw, proto) != (EXPECTED_FIRMWARE, 2):
                        print(f"STOP: unexpected firmware {fw!r} protocol {proto}; expected {EXPECTED_FIRMWARE!r}")
                        return 2
                    firmware_ok = True
                    break
            if not firmware_ok:
                print("STOP: no expected VCU HELLO within 8 s")
                return 2

        class CameraLink(Node):
            def __init__(self, writer, output):
                super().__init__("autokart_phase1_camera_link")
                self.writer, self.output = writer, output
                self.statuses = {}
                self.cones = {}
                self.seq = 0
                self.last_frame_wall = None
                self.last_stop_wall = 0.0
                self.pending_ack = {}
                self.create_subscription(MissionDebug, "/autokart/perception/status", self.on_status, 10)
                self.create_subscription(ConeArray, "/perception/cones", self.on_cones, qos_profile_sensor_data)
                self.create_timer(0.1, self.tick)

            def send(self, row):
                self.seq += 1
                stamp_ms, cone_id, forward_mm, gap_mm, code = fields(row)
                frame = build_observation(self.seq, stamp_ms, cone_id, forward_mm, gap_mm, code)
                if port is not None:
                    port.write(frame)
                    self.pending_ack[self.seq] = (code, time.monotonic(), row)
                self.writer.writerow({"wall_time_s": time.time(), "seq": self.seq, **row,
                                      "ack": "PENDING" if port is not None else "DRY_RUN"})
                self.output.flush()
                if code == 0:
                    self.last_stop_wall = time.monotonic()
                print(f"OBS {self.seq}: {row['decision']} {row['reason']} gap={row['bumper_to_center_m']}")

            def stop(self, reason):
                row = evaluate({}, [], stamp_ns=0, cone_id=args.cone_id)
                row["reason"] = reason
                self.send(row)

            def on_status(self, message):
                stamp = ns(message.measurement_time)
                try:
                    status = json.loads(message.payload_json)
                except (TypeError, ValueError):
                    status = {}
                self.last_frame_wall = time.monotonic()
                if not status.get("valid"):
                    self.statuses.clear()
                    self.cones.clear()
                    self.send(evaluate(status, [], stamp_ns=stamp, cone_id=args.cone_id))
                else:
                    self.statuses[stamp] = (time.monotonic(), status)
                    self.pair(stamp)

            def on_cones(self, message):
                stamp = ns(message.measurement_time)
                if message.frame_id != "base_link" or message.ground_truth:
                    self.stop("UNQUALIFIED_CONE_FRAME")
                    return
                self.cones[stamp] = (time.monotonic(), [
                    {"id": int(c.id), "x_m": float(c.position.x), "y_m": float(c.position.y),
                     "position_std_m": float(c.position_std_m), "confidence": float(c.confidence)}
                    for c in message.cones])
                self.pair(stamp)

            def pair(self, stamp):
                if stamp in self.statuses and stamp in self.cones:
                    _, status = self.statuses.pop(stamp)
                    _, cones = self.cones.pop(stamp)
                    self.send(evaluate(status, cones, stamp_ns=stamp, cone_id=args.cone_id))

            def read_acks(self):
                if port is None:
                    return
                for _ in range(100):
                    if not port.in_waiting:
                        break
                    payload = parse_frame(port.readline())
                    parsed = parse_message(payload) if payload is not None else None
                    if parsed and parsed[0] == "OBSACK":
                        seq, code = parsed[1]
                        expected = self.pending_ack.pop(seq, None)
                        if expected is None or expected[0] != code:
                            self.get_logger().error("unexpected OBSACK")
                        else:
                            self.writer.writerow({"wall_time_s": time.time(), "seq": seq, **expected[2],
                                                  "ack": "OK"})
                            self.output.flush()
                            print(f"OBSACK {seq} decision={code}")

            def tick(self):
                now = time.monotonic()
                for stamp, (received, _) in list(self.cones.items()):
                    if now - received > 0.3:
                        self.cones.pop(stamp, None)
                        self.statuses.pop(stamp, None)
                        self.stop("STATUS_PAIR_TIMEOUT")
                for stamp, (received, _) in list(self.statuses.items()):
                    if now - received > 0.3:
                        self.statuses.pop(stamp, None)
                        self.cones.pop(stamp, None)
                        self.stop("CONE_PAIR_TIMEOUT")
                if (self.last_frame_wall is None or now - self.last_frame_wall > 0.5) and now - self.last_stop_wall > 0.2:
                    self.stop("PERCEPTION_TIMEOUT")
                self.read_acks()
                for seq, (_, sent, row) in list(self.pending_ack.items()):
                    if now - sent > 0.5:
                        self.pending_ack.pop(seq, None)
                        self.writer.writerow({"wall_time_s": time.time(), "seq": seq, **row,
                                              "ack": "TIMEOUT"})
                        self.output.flush()
                        self.get_logger().error(f"OBSACK timeout seq={seq}")

        os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
        with open(args.output, "w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=["wall_time_s", "seq", "stamp_ns", "cone_id",
                "cone_forward_m", "bumper_to_center_m", "target_gap_m", "decision", "reason", "ack"])
            writer.writeheader()
            rclpy.init()
            node = CameraLink(writer, output)
            print("Camera observations only: ARM/CMD/RST are never transmitted; firmware arming lock remains active.")
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
            finally:
                try:
                    node.stop("LINK_SHUTDOWN")
                    deadline = time.monotonic() + 0.75
                    while port is not None and node.pending_ack and time.monotonic() < deadline:
                        node.read_acks()
                        time.sleep(0.01)
                    for seq, (_, _, row) in list(node.pending_ack.items()):
                        writer.writerow({"wall_time_s": time.time(), "seq": seq, **row,
                                         "ack": "TIMEOUT"})
                    output.flush()
                except (OSError, serial.SerialException):
                    pass
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
        return 0
    finally:
        if port is not None:
            port.close()


if __name__ == "__main__":
    raise SystemExit(main())
