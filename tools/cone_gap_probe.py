#!/usr/bin/env python3
"""Read-only, camera-only stop-cone gap diagnostic. Never sends motion commands."""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "kart", "ros2_ws", "src", "autokart_vehicle_geometry"))
from autokart_vehicle_geometry.geometry import GEOMETRY  # noqa: E402


def evaluate(status: dict, cones: list[dict], *, stamp_ns: int, cone_id: int,
             bumper_offset_m: float = GEOMETRY.camera_to_front_bumper_horizontal_m,
             target_gap_m: float = GEOMETRY.stop_gap_from_bumper_m) -> dict:
    """Pair a qualified frame with one explicit ground-contact cone ID."""
    result = {"stamp_ns": stamp_ns, "cone_id": cone_id, "cone_forward_m": None,
              "bumper_to_center_m": None, "target_gap_m": target_gap_m,
              "decision": "STOP", "reason": "UNQUALIFIED_FRAME"}
    if (status.get("schema_version") != 1 or status.get("source") != "LIVE" or
            status.get("sample_time_ns") != stamp_ns or not status.get("valid") or
            status.get("frame_id") != "base_link"):
        return result
    matches = [cone for cone in cones if cone["id"] == cone_id]
    if len(matches) != 1:
        result["reason"] = "STOP_CONE_MISSING_OR_AMBIGUOUS"
        return result
    cone = matches[0]
    x, y, sigma = cone["x_m"], cone["y_m"], cone["position_std_m"]
    if (not all(math.isfinite(v) for v in (x, y, sigma)) or x <= 0 or sigma < 0 or
            sigma > 0.5 or not 0 <= cone["confidence"] <= 1):
        result["reason"] = "INVALID_CONE_GEOMETRY"
        return result
    gap = x - bumper_offset_m
    result.update(cone_forward_m=x, bumper_to_center_m=gap,
                  decision="AT_OR_INSIDE_TARGET" if gap <= target_gap_m + 1e-9 else "OBSERVE_ONLY",
                  reason="QUALIFIED_CONE")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cone-id", type=int, required=True,
                        help="ID of the stop cone; identify it from a stationary observation")
    parser.add_argument("--output", default=os.path.join(ROOT, "results", "cone_gap_probe.csv"))
    args = parser.parse_args()
    if args.cone_id < 0:
        parser.error("--cone-id must be nonnegative")
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from autokart_msgs.msg import ConeArray, MissionDebug
    except ImportError as exc:
        parser.error(f"ROS overlay unavailable: {exc}")

    def ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    class Probe(Node):
        def __init__(self):
            super().__init__("autokart_cone_gap_probe")
            self.statuses = {}
            self.pending = {}
            self.last_output = time.monotonic()
            self.create_subscription(MissionDebug, "/autokart/perception/status", self.on_status, 10)
            self.create_subscription(ConeArray, "/perception/cones", self.on_cones, qos_profile_sensor_data)
            self.create_timer(0.1, self.on_timer)

        def write(self, row):
            row = {"time_monotonic_s": round(time.monotonic(), 6), **row}
            writer.writerow(row)
            output.flush()
            self.last_output = time.monotonic()
            print(f"{row['decision']}: {row['reason']}; cone x={row['cone_forward_m']} m; "
                  f"bumper-to-center={row['bumper_to_center_m']} m; target={row['target_gap_m']} m")

        def on_status(self, message):
            stamp = ns(message.measurement_time)
            try:
                payload = json.loads(message.payload_json)
            except (ValueError, TypeError):
                payload = {}
            if not payload.get("valid"):
                self.pending.clear()
                self.write(evaluate(payload, [], stamp_ns=stamp, cone_id=args.cone_id))
            else:
                self.statuses[stamp] = payload
                self.pair(stamp)

        def on_cones(self, message):
            stamp = ns(message.measurement_time)
            if message.frame_id != "base_link" or message.ground_truth:
                self.write(evaluate({}, [], stamp_ns=stamp, cone_id=args.cone_id))
                return
            cones = [{"id": int(c.id), "x_m": float(c.position.x), "y_m": float(c.position.y),
                      "position_std_m": float(c.position_std_m), "confidence": float(c.confidence)}
                     for c in message.cones]
            self.pending[stamp] = (cones, time.monotonic())
            self.pair(stamp)

        def pair(self, stamp):
            if stamp in self.statuses and stamp in self.pending:
                status = self.statuses.pop(stamp)
                cones, _ = self.pending.pop(stamp)
                self.write(evaluate(status, cones, stamp_ns=stamp, cone_id=args.cone_id))

        def on_timer(self):
            now = time.monotonic()
            for stamp, (_, received) in list(self.pending.items()):
                if now - received > 0.3:
                    self.pending.pop(stamp)
                    self.statuses.pop(stamp, None)
                    self.write(evaluate({}, [], stamp_ns=stamp, cone_id=args.cone_id))
            if now - self.last_output > 0.5:
                self.write(evaluate({}, [], stamp_ns=0, cone_id=args.cone_id))

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=["time_monotonic_s", "stamp_ns", "cone_id",
            "cone_forward_m", "bumper_to_center_m", "target_gap_m", "decision", "reason"])
        writer.writeheader()
        print("READ-ONLY DIAGNOSTIC. No ARM, CMD, or VCU serial access. Stop target is bumper to cone center.")
        rclpy.init()
        node = Probe()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
