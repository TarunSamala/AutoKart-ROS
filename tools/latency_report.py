#!/usr/bin/env python3
"""Measure camera-stamp to confirmed VCU DAC-zero latency from live ROS or a rosbag2 bag.

Requires the four stage status topics and /autokart/latency/trace. Timestamps must be
from one host monotonic clock; bag receipt times are never substituted for stage times.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import time

STAGES = ("perception", "mapping", "planning", "controller")
TOPICS = {f"/autokart/{stage}/status": stage for stage in STAGES}
TRACE_TOPIC = "/autokart/latency/trace"


class LatencyCollector:
    def __init__(self):
        self.frames: dict[int, dict] = {}
        self.tx: list[dict] = []
        self.tel: list[dict] = []

    def add(self, topic: str, message) -> None:
        if topic == TRACE_TOPIC:
            try:
                payload = json.loads(message.data)
                event = payload["event"]
                if event == "bridge_tx":
                    self.tx.append(payload)
                elif event == "vcu_tel":
                    self.tel.append(payload)
            except (ValueError, KeyError, TypeError):
                pass
            return
        stage = TOPICS.get(topic)
        if stage is None:
            return
        try:
            payload = json.loads(message.payload_json)
            stamp = int(payload["sample_time_ns"])
            completed = int(payload["processed_at_monotonic_ns"])
            if payload.get("source") != "LIVE" or stamp <= 0 or completed < stamp:
                return
            frame = self.frames.setdefault(stamp, {})
            frame[stage] = completed
            frame[f"{stage}_payload"] = payload
        except (ValueError, KeyError, TypeError):
            return

    def complete(self) -> list[dict]:
        tel_by_ack = {}
        for item in self.tel:
            try:
                if int(item["dac_mv"]) == 0:
                    key = int(item["ack_seq"])
                    at = int(item["time_ns"])
                    tel_by_ack[key] = min(at, tel_by_ack.get(key, at))
            except (ValueError, KeyError, TypeError):
                continue
        result = []
        seen = set()
        for item in sorted(self.tx, key=lambda entry: int(entry.get("time_ns", 0))):
            try:
                stamp = int(item["frame_stamp_ns"])
                seq = int(item["seq"])
                tx_at = int(item["time_ns"])
                tel_at = tel_by_ack[seq]
                frame = self.frames[stamp]
                if int(item["requested_dac_mv"]) != 0 or stamp in seen or not all(s in frame for s in STAGES):
                    continue
                if (frame["planning_payload"].get("decision") not in ("STOP_CANDIDATE", "HOLD") or
                        float(frame["controller_payload"].get("target_speed_mps", math.nan)) != 0.0):
                    continue
                chain = [stamp, *(frame[s] for s in STAGES), tx_at, tel_at]
                if any(b < a for a, b in zip(chain, chain[1:])):
                    continue
                result.append({"frame_stamp_ns": stamp, "camera_to_tel_ms": (tel_at - stamp) / 1e6,
                               "stage_ms": {label: (b - a) / 1e6 for label, a, b in zip(
                                   ("detections", "map", "plan", "controller", "bridge_tx", "vcu_tel"),
                                   chain, chain[1:])}})
                seen.add(stamp)
            except (ValueError, KeyError, TypeError):
                continue
        return result


def percentile(values: list[float], p: float) -> float:
    ordered = sorted(values)
    if not ordered:
        raise ValueError("no complete camera-to-DAC-zero frame chains")
    index = (len(ordered) - 1) * p
    lower = math.floor(index)
    return ordered[lower] + (ordered[math.ceil(index)] - ordered[lower]) * (index - lower)


def read_bag(path: str, collector: LatencyCollector) -> None:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag_path = Path(path)
    try:
        import yaml
        metadata = yaml.safe_load((bag_path / "metadata.yaml").read_text())
        storage_id = metadata["rosbag2_bagfile_information"]["storage_identifier"]
    except (ImportError, OSError, KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"rosbag2 metadata.yaml required to identify bag storage: {exc}") from exc
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage_id),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {item.name: get_message(item.type) for item in reader.get_all_topics_and_types()
             if item.name in TOPICS or item.name == TRACE_TOPIC}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in types:
            collector.add(topic, deserialize_message(data, types[topic]))


def read_live(seconds: float, collector: LatencyCollector) -> None:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from autokart_msgs.msg import MissionDebug

    rclpy.init()
    node = Node("autokart_latency_report")
    subscriptions = [node.create_subscription(MissionDebug, topic,
                     lambda msg, name=topic: collector.add(name, msg), 50) for topic in TOPICS]
    subscriptions.append(node.create_subscription(String, TRACE_TOPIC,
                         lambda msg: collector.add(TRACE_TOPIC, msg), 100))
    deadline = time.monotonic() + seconds if seconds > 0 else None
    try:
        while rclpy.ok() and (deadline is None or time.monotonic() < deadline):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--bag", help="rosbag2 directory with status and trace topics")
    source.add_argument("--live", action="store_true", help="subscribe to the running ROS graph")
    parser.add_argument("--seconds", type=float, default=30.0, help="live capture duration; 0 means Ctrl-C")
    parser.add_argument("--out-dir", default=str(Path(__file__).resolve().parents[1] / "results"))
    args = parser.parse_args()
    collector = LatencyCollector()
    try:
        if args.bag:
            read_bag(args.bag, collector)
        else:
            read_live(args.seconds, collector)
    except (ImportError, ValueError, OSError) as exc:
        parser.error(f"recording unavailable: {exc}")
    complete = collector.complete()
    if not complete:
        raise SystemExit("NO_MEASUREMENT: no complete monotonic frame -> detections -> map -> plan -> "
                         "controller -> bridge TX -> acknowledged VCU TEL with dac_mv 0 chain")
    for row in complete:
        stages = " ".join(f"{stage}={value:.1f}ms" for stage, value in row["stage_ms"].items())
        print(f"frame {row['frame_stamp_ns']}: {stages} total={row['camera_to_tel_ms']:.1f}ms")
    values = [row["camera_to_tel_ms"] for row in complete]
    p50, p95 = percentile(values, 0.5), percentile(values, 0.95)
    print(f"n={len(values)} p50={p50:.1f} ms p95={p95:.1f} ms max={max(values):.1f} ms")
    print("Paste into commissioning.yaml after reviewing the sample:")
    conservative_s = math.ceil((p95 + 50.0) * 10.0) / 10_000.0  # never round the margin down
    print(f"t_latency: {conservative_s:.4f}")
    from datetime import datetime, timezone
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    summary = {"created_utc": datetime.now(timezone.utc).isoformat(), "n": len(values),
              "p50_ms": p50, "p95_ms": p95, "max_ms": max(values),
              "suggested_t_latency_s": conservative_s, "source": "bag" if args.bag else "live",
              "frames": complete}
    out_path = out_dir / f"latency_report_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.json"
    out_path.write_text(json.dumps(summary, indent=2, sort_keys=True))
    print(f"Saved {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
