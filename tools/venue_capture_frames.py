#!/usr/bin/env python3
"""Read-only: subscribe to live /autokart/perception/rgb and /depth, save N stamp-matched
RGB+depth pairs to disk for one venue-capture station. Requires the phase-1 graph already running
(scripts/start_phase1.sh; tools/venue_capture.sh drives this per station). Sends nothing to the
VCU and sets no verified flag; writes only image/array files and its own meta.json under
--station-dir."""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path


def stamp_key(msg) -> int:
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class FramePairer:
    """Matches RGB and depth messages by identical header stamp -- both are published from the
    same captured Frame with the same stamp (see autokart_perception_ros/node.py's _images()), so
    an exact-stamp match is the paired-frame rule already enforced upstream, not a new one. Calls
    save(index, rgb_msg, depth_msg) once per matched pair, in arrival order, and stops accepting
    new frames once target_pairs is reached."""

    def __init__(self, save, target_pairs: int):
        self._rgb: dict[int, object] = {}
        self._depth: dict[int, object] = {}
        self._save = save
        self._target = target_pairs
        self.saved = 0

    def on_rgb(self, msg) -> None:
        self._offer("rgb", msg)

    def on_depth(self, msg) -> None:
        self._offer("depth", msg)

    def _offer(self, kind: str, msg) -> None:
        if self.done:
            return
        key = stamp_key(msg)
        mine = self._rgb if kind == "rgb" else self._depth
        mine[key] = msg
        if key in self._rgb and key in self._depth:
            rgb_msg = self._rgb.pop(key)
            depth_msg = self._depth.pop(key)
            self.saved += 1
            self._save(self.saved, rgb_msg, depth_msg)

    @property
    def done(self) -> bool:
        return self.saved >= self._target


def image_to_array(msg):
    import numpy as np

    mono = msg.encoding == "mono16"
    dtype = np.uint16 if mono else np.uint8
    channels = 1 if mono else 3
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    row_bytes = msg.step
    rows = buf.reshape(msg.height, row_bytes)
    itemsize = np.dtype(dtype).itemsize
    row_items = row_bytes // itemsize
    arr = rows.view(dtype).reshape(msg.height, row_items)
    if mono:
        return arr[:, : msg.width].copy()
    return arr.reshape(msg.height, row_items // channels, channels)[:, : msg.width, :].copy()


def save_pair(out_dir: Path, meta: list, index: int, rgb_msg, depth_msg) -> None:
    import cv2
    import numpy as np

    rgb = image_to_array(rgb_msg)
    depth = image_to_array(depth_msg)
    cv2.imwrite(str(out_dir / f"rgb_{index:03d}.png"), rgb)
    np.save(out_dir / f"depth_{index:03d}.npy", depth)
    meta.append({"index": index, "stamp_ns": stamp_key(rgb_msg),
                "rgb_file": f"rgb_{index:03d}.png", "depth_file": f"depth_{index:03d}.npy",
                "width": rgb_msg.width, "height": rgb_msg.height})
    print(f"  saved pair {index}: stamp_ns={stamp_key(rgb_msg)}")


def run(station_dir: Path, target_pairs: int, timeout_s: float) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    station_dir.mkdir(parents=True, exist_ok=True)
    meta: list = []
    pairer = FramePairer(lambda i, rgb, depth: save_pair(station_dir, meta, i, rgb, depth), target_pairs)

    rclpy.init()
    node = Node("venue_capture_frames")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Image, "/autokart/perception/rgb", pairer.on_rgb, qos)
    node.create_subscription(Image, "/autokart/perception/depth", pairer.on_depth, qos)
    deadline = time.monotonic() + timeout_s
    try:
        while not pairer.done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    (station_dir / "meta.json").write_text(json.dumps(
        {"target_pairs": target_pairs, "saved_pairs": pairer.saved, "frames": meta}, indent=2))
    if pairer.saved == 0:
        print("NO_PAIRS_CAPTURED: no matching RGB+depth pair arrived within the timeout -- is the "
              "phase-1 graph running with a live camera? Nothing was saved except meta.json.",
              file=sys.stderr)
        return 2
    print(f"Saved {pairer.saved}/{target_pairs} pairs to {station_dir}")
    return 0 if pairer.saved >= target_pairs else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--station-dir", required=True)
    parser.add_argument("--frames", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("--frames must be >= 1")
    return run(Path(args.station_dir), args.frames, args.timeout)


if __name__ == "__main__":
    raise SystemExit(main())
