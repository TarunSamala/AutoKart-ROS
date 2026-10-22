#!/usr/bin/env python3
"""Observe a wheels-off stop trigger through planner, bridge TX and VCU telemetry.

Read-only: this tool never sends ARM, CMD, DIS, or RST. The operator independently
arms via the dashboard after preflight and places the cone. Telemetry cannot prove
physical SV voltage or BLD-750 BRK terminal state; measure those separately.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


class StopTriggerObserver:
    def __init__(self, timeout_s: float = 5.0):
        if timeout_s <= 0:
            raise ValueError("timeout must be positive")
        self.timeout_ns = int(timeout_s * 1e9)
        self.active_tel_at_ns: int | None = None
        self.trigger: dict | None = None
        self.tx: dict | None = None
        self.result: dict | None = None

    def add_trace(self, event: dict) -> None:
        try:
            at = int(event["time_ns"])
            if event.get("event") == "vcu_tel":
                if (event.get("state") == "ARMED" and int(event["dac_mv"]) > 0 and
                        event.get("en") is True and event.get("brk") is False and int(event.get("fault", 0)) == 0):
                    self.active_tel_at_ns = at
                if (self.trigger is not None and self.tx is not None and
                        at >= int(self.tx["time_ns"]) and int(event["ack_seq"]) == int(self.tx["seq"]) and
                        at - int(self.trigger["processed_at_monotonic_ns"]) <= self.timeout_ns and
                        int(event["dac_mv"]) == 0 and event.get("en") is False and
                        event.get("brk") is True and event.get("state") in ("SAFE", "ARMED") and
                        int(event.get("fault", 0)) == 0):
                    self.result = {"frame_stamp_ns": self.trigger["sample_time_ns"],
                                   "bridge_seq": self.tx["seq"], "trigger_to_tel_ms":
                                   (at - int(self.trigger["processed_at_monotonic_ns"])) / 1e6,
                                   "result": "PASS_TELEMETRY_ONLY"}
            elif event.get("event") == "bridge_tx" and self.trigger is not None:
                if (int(event.get("frame_stamp_ns", -1)) == int(self.trigger["sample_time_ns"]) and
                        int(event["requested_dac_mv"]) == 0 and
                        at >= int(self.trigger["processed_at_monotonic_ns"])):
                    self.tx = event
        except (KeyError, TypeError, ValueError):
            return

    def add_planner(self, payload: dict) -> None:
        try:
            at = int(payload["processed_at_monotonic_ns"])
            gap = float(payload["bumper_to_cone_m"])
            stop_gap = float(payload["stop_gap_from_bumper_m"])
            stopping = float(payload["stopping_distance_m"])
            if (self.active_tel_at_ns is None or self.active_tel_at_ns > at or
                    at - self.active_tel_at_ns > 500_000_000 or
                    payload.get("source") != "LIVE" or payload.get("valid") is not True or
                    payload.get("decision") != "STOP_CANDIDATE" or
                    payload.get("dead_reckoned") is True or
                    float(payload["target_speed_mps"]) != 0.0 or
                    not all(math.isfinite(v) for v in (gap, stop_gap, stopping)) or
                    stop_gap < 0 or stopping < 0 or gap > stop_gap + stopping):
                return
            self.trigger = payload
        except (KeyError, TypeError, ValueError):
            return

    def timed_out(self, now_ns: int) -> bool:
        return self.trigger is not None and self.result is None and \
            now_ns - int(self.trigger["processed_at_monotonic_ns"]) > self.timeout_ns


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=5.0, help="seconds after a qualified stop trigger")
    parser.add_argument("--wait", type=float, default=60.0, help="seconds to wait for an armed trigger")
    args = parser.parse_args()
    if args.timeout <= 0 or args.wait <= 0:
        parser.error("timeouts must be positive")
    gate = subprocess.run([str(ROOT / "scripts/system_check.sh")], text=True, capture_output=True, check=False)
    print(gate.stdout, end="")
    if gate.returncode != 0 or "FINAL: GO" not in gate.stdout:
        print("NO-GO: preflight did not pass; no stop-trigger observation started.")
        return 2
    confirmation = input("Wheels OFF ground, physical battery/supply cut within reach, second observer present, SV/BRK meter ready? Type WHEELS OFF: ")
    if confirmation != "WHEELS OFF":
        print("NOT EXECUTED: operator confirmation absent.")
        return 2
    try:
        import rclpy
        from rclpy.node import Node
        from autokart_msgs.msg import MissionDebug
        from std_msgs.msg import String
    except ImportError as exc:
        print(f"NOT EXECUTED: ROS unavailable: {exc}")
        return 2
    observer = StopTriggerObserver(args.timeout)

    class ObserverNode(Node):
        def __init__(self):
            super().__init__("autokart_stop_trigger_observer")
            self.create_subscription(MissionDebug, "/autokart/planner/status", self.on_planner, 10)
            self.create_subscription(String, "/autokart/latency/trace", self.on_trace, 50)

        def on_planner(self, message):
            try:
                observer.add_planner(json.loads(message.payload_json))
            except (TypeError, ValueError):
                pass

        def on_trace(self, message):
            try:
                observer.add_trace(json.loads(message.data))
            except (TypeError, ValueError):
                pass

    rclpy.init()
    node = ObserverNode()
    deadline = time.monotonic() + args.wait
    try:
        print("READ-ONLY. Arm independently via approved dashboard flow; place cone inside measured stop envelope.")
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if observer.result is not None:
                print(json.dumps(observer.result, sort_keys=True))
                print("Verify actual SV=0 V and BRK terminal open with meters; telemetry alone is not electrical proof.")
                return 0
            if observer.timed_out(time.monotonic_ns()):
                print("FAIL: stop trigger was not followed by matching zero bridge TX and braked VCU TEL in time.")
                return 1
        print("NOT EXECUTED: no complete qualified armed stop trigger before wait timeout.")
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
