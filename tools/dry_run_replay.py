#!/usr/bin/env python3
"""Replay a recorded HP60C session (autokart_perception.recording.Replay directory, see
tools/depth_check.py's output format for how frames are normally captured) through the REAL
perception qualification pipeline, then through the planner's bumper-referenced stop-distance math,
with a SIMULATED constant speed advancing a synthetic straight-line pose between frames (there is no
real odometry in a camera-only replay). No ROS, no hardware, no serial port is opened. DISARMED by
construction: nothing here can reach a VCU.

This does NOT exercise the real mapping corridor-fitting algorithm (reusing it correctly from boundary
cones offline is out of this tool's scope); instead it builds a straight synthetic centerline ahead of
the kart and treats every qualified cone within the mission's lateral footprint as an obstacle, which is
sufficient to show the BRAKE decision moment and commanded volts for a straight-corridor replay, but is
not a substitute for a live mapping-qualified run.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [
    str(ROOT / "autonomy_stack_phase1" / "phase_01_perception" / "src"),
    str(ROOT / "kart" / "ros2_ws" / "src" / "autokart_vehicle_geometry"),
    str(ROOT / "kart" / "ros2_ws" / "src" / "autokart_nodes" / "autokart_planner"),
    str(ROOT / "kart" / "ros2_ws" / "src" / "autokart_vehicle_bridge"),
]


def qualified_cones(result, cone_radius_m: float) -> list[dict]:
    """Extract (x_m, y_m, radius_m) obstacles from a qualified perception result's cones; [] if none."""
    cones = getattr(result.perception_result, "cones", None) or []
    out = []
    for cone in cones:
        try:
            x, y = cone.xy
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            out.append({"x_m": float(x), "y_m": float(y), "radius_m": float(cone_radius_m)})
        except (AttributeError, TypeError, ValueError):
            continue
    return out


def run(replay_dir: str, v_sim_mps: float, a_cfg_mps2: float, t_latency_s: float, margin_m: float,
       cone_radius_m: float = 0.1, horizon_m: float = 20.0) -> dict:
    from autokart_perception.recording import Replay
    from autokart_perception.pipeline import Pipeline
    from autokart_perception.detector import Yolo26OnnxDetector
    from autokart_perception.application import CoreRuntime
    from autokart_vehicle_geometry.geometry import GEOMETRY
    from autokart_planner.mission import PlannerEngine
    from autokart_perception.shared_core import BrakeCapability
    from autokart_vehicle_bridge.core import ThrottleConfig, ThrottleController

    model_path = ROOT / "models" / "yolo26s" / "best.onnx"
    manifest_path = ROOT / "models" / "yolo26s" / "best.json"
    detector = Yolo26OnnxDetector(model_path, manifest_path=manifest_path) if model_path.is_file() else None
    core = CoreRuntime()
    pipeline = Pipeline(detector, dict(core.config), synthetic=False)
    replay = Replay(replay_dir)

    brake = BrakeCapability(a_cfg_mps2, t_latency_s, "SIMULATED", margin_m=margin_m)
    engine = PlannerEngine("BRAKING_TEST", "SIMULATED", brake, speed_limit_mps=v_sim_mps)
    throttle = ThrottleController(ThrottleConfig())
    centerline = [(float(i), 0.0) for i in range(1, int(horizon_m) + 1)]

    events = []
    x_sim = 0.0
    last_stamp_ns = None
    stamp = 1_000_000_000
    frame_index = 0
    brake_event = None
    while True:
        frame = replay.read()
        if frame is None:
            break
        frame_index += 1
        result = core.sdk.process_frame(frame, pipeline=pipeline) if hasattr(core, "sdk") else None
        if result is None:
            break
        dt = 0.05 if last_stamp_ns is None else max(1e-3, (frame.timestamp_ns - last_stamp_ns) / 1e9)
        last_stamp_ns = frame.timestamp_ns
        if brake_event is None:
            x_sim += v_sim_mps * dt
        obstacles = qualified_cones(result, cone_radius_m)
        stamp += int(dt * 1e9)
        mapping = {"valid": True, "source": "SIMULATED", "frame_id": "odom", "sample_time_ns": stamp,
                   "geometry": {"centerline_status": "PROVISIONAL", "centerline": centerline}}
        from autokart_perception.shared_core import PoseEstimate
        pose = PoseEstimate(x_sim, 0.0, 0.0, v_sim_mps if brake_event is None else 0.0, stamp, 0.05)
        plan = engine.evaluate(mapping, pose, obstacles)
        throttle_v = throttle.step(frame_index * 0.05, plan["target_speed_mps"], v_sim_mps if brake_event is None else 0.0)
        record = {"frame_index": frame_index, "x_sim_m": round(x_sim, 3), "decision": plan["decision"],
                  "commanded_v": round(throttle_v, 3), "bumper_available_m": plan.get("bumper_available_m")}
        events.append(record)
        if brake_event is None and plan["decision"] == "STOP_CANDIDATE":
            brake_event = dict(record)
    return {
        "replay_dir": replay_dir, "frames_replayed": frame_index, "v_sim_mps": v_sim_mps,
        "a_cfg_mps2": a_cfg_mps2, "t_latency_s": t_latency_s, "margin_m": margin_m,
        "camera_to_bumper_m": GEOMETRY.camera_to_front_bumper_horizontal_m,
        "brake_decision_event": brake_event, "events": events,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay_dir", help="directory of NNNNNN.json/.npz recorded frames")
    parser.add_argument("--v-sim", type=float, default=1.0, help="simulated constant approach speed, m/s")
    parser.add_argument("--a-cfg", type=float, default=0.5, help="configured deceleration for the stop formula, m/s^2")
    parser.add_argument("--t-latency", type=float, default=0.25, help="assumed reaction latency, s")
    parser.add_argument("--margin", type=float, default=1.0, help="assumed stop margin, m (commissioning default)")
    parser.add_argument("--cone-radius", type=float, default=0.1, help="assumed physical cone radius, m (not measured)")
    parser.add_argument("--out", help="save full event trace as JSON")
    args = parser.parse_args()

    print("DRY RUN, DISARMED, NO HARDWARE. Simulated speed and assumed cone radius are NOT measured values.")
    try:
        result = run(args.replay_dir, args.v_sim, args.a_cfg, args.t_latency, args.margin, args.cone_radius)
    except FileNotFoundError as exc:
        print(f"NOT EXECUTED: {exc}")
        return 2
    print(f"Replayed {result['frames_replayed']} frames from {args.replay_dir}")
    if result["brake_decision_event"] is None:
        print("No BRAKE decision occurred during this replay.")
    else:
        event = result["brake_decision_event"]
        print(f"BRAKE decision at frame {event['frame_index']}: simulated x={event['x_sim_m']} m, "
              f"commanded throttle={event['commanded_v']:.2f} V, "
              f"estimated bumper gap={event['bumper_available_m']}")
    if args.out:
        with open(args.out, "w", encoding="utf-8") as handle:
            json.dump(result, handle, indent=2, sort_keys=True, allow_nan=False)
        print(f"Saved {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
