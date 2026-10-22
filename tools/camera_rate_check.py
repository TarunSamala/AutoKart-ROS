#!/usr/bin/env python3
"""Read-only: subscribe to live /autokart/perception/rgb and /depth for a fixed window and report
observed rate, mean/max interval and timestamp monotonicity. Requires the phase-1 graph already
running (scripts/start_phase1.sh). Sends nothing to the VCU; sets no verified flag."""
from __future__ import annotations

import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class Stats:
    def __init__(self, name: str):
        self.name = name
        self.count = 0
        self.recv_times: list[float] = []
        self.nonmonotonic = 0
        self.last_stamp_ns: int | None = None

    def on_msg(self, msg) -> None:
        now = time.monotonic()
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.count += 1
        self.recv_times.append(now)
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            self.nonmonotonic += 1
        self.last_stamp_ns = stamp_ns

    def summary(self) -> dict:
        if len(self.recv_times) >= 2:
            span = self.recv_times[-1] - self.recv_times[0]
            rate_hz = (self.count - 1) / span if span > 0 else 0.0
            gaps = [b - a for a, b in zip(self.recv_times, self.recv_times[1:])]
            mean_gap, max_gap = sum(gaps) / len(gaps), max(gaps)
        else:
            rate_hz, mean_gap, max_gap = 0.0, None, None
        return {"topic": self.name, "frames_received": self.count, "observed_rate_hz": round(rate_hz, 3),
               "mean_interval_s": round(mean_gap, 4) if mean_gap is not None else None,
               "max_interval_s": round(max_gap, 4) if max_gap is not None else None,
               "nonmonotonic_stamp_count": self.nonmonotonic}


def run(duration_s: float) -> dict:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node("camera_rate_check")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    rgb, depth = Stats("/autokart/perception/rgb"), Stats("/autokart/perception/depth")
    node.create_subscription(Image, rgb.name, rgb.on_msg, qos)
    node.create_subscription(Image, depth.name, depth.on_msg, qos)
    t_end = time.monotonic() + duration_s
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    paired_hz = min(rgb.summary()["observed_rate_hz"], depth.summary()["observed_rate_hz"])
    return {"created_utc": datetime.now(timezone.utc).isoformat(), "window_s": duration_s,
            "rgb": rgb.summary(), "depth": depth.summary(), "paired_rgb_depth_hz": paired_hz}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--out-dir", default=str(ROOT / "results"))
    args = parser.parse_args()
    try:
        result = run(args.duration)
    except ImportError as exc:
        print(f"NOT EXECUTED: ROS unavailable: {exc}")
        return 2
    print(json.dumps(result, indent=2))
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"camera_rate_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.json"
    path.write_text(json.dumps(result, indent=2, sort_keys=True))
    print(f"Saved {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
