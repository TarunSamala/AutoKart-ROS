#!/usr/bin/env python3
"""Calculate the cone-detection runway from measured speed, braking, latency, and shared geometry.

This is an arithmetic commissioning aid, not permission to move the kart. It never writes config.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "kart/ros2_ws/src/autokart_vehicle_geometry"))
from autokart_vehicle_geometry.geometry import GEOMETRY  # noqa: E402

GAPS_M = (1.0, 0.6, 0.4, 0.25, 0.15)


def calculate(v_run: float, a_verified: float, t_latency: float, margin: float,
              depth_max_m: float = 4.0, *, camera_to_bumper_m: float = GEOMETRY.camera_to_front_bumper_horizontal_m,
              gaps_m: tuple[float, ...] = GAPS_M) -> dict:
    values = (v_run, a_verified, t_latency, margin, depth_max_m, camera_to_bumper_m, *gaps_m)
    if not all(math.isfinite(value) for value in values) or v_run <= 0 or a_verified <= 0 or \
            t_latency < 0 or margin < 0 or depth_max_m <= camera_to_bumper_m or \
            camera_to_bumper_m < 0 or any(gap < 0 for gap in gaps_m):
        raise ValueError("need finite measured speed/deceleration, nonnegative latency/margin/gaps, and usable depth")
    first_bumper_gap = depth_max_m - camera_to_bumper_m
    stopping_distance = v_run * t_latency + v_run * v_run / (2 * a_verified) + margin
    return {"max_bumper_to_cone_at_first_detection_m": first_bumper_gap,
            "stopping_distance_m": stopping_distance,
            "rows": [{"stop_gap_m": gap, "runway_m": first_bumper_gap - gap - stopping_distance,
                      "less_than_0_5_m": first_bumper_gap - gap - stopping_distance < 0.5}
                     for gap in gaps_m]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--v-run", type=float, required=True, help="tape/video measured ground speed at 2 V, m/s")
    parser.add_argument("--a-verified", type=float, required=True, help="measured minimum deceleration, m/s^2")
    parser.add_argument("--t-latency", type=float, required=True, help="measured p95 + 50 ms, seconds")
    parser.add_argument("--margin", type=float, required=True, help="additional distance margin, metres")
    parser.add_argument("--depth-max", type=float, default=4.0, help="accepted depth maximum, metres")
    args = parser.parse_args()
    try:
        result = calculate(args.v_run, args.a_verified, args.t_latency, args.margin, args.depth_max)
    except ValueError as exc:
        parser.error(str(exc))
    print("ARITHMETIC ONLY: inputs must be physically measured; no vehicle gate is changed.")
    print(f"Camera-to-bumper horizontal offset: {GEOMETRY.camera_to_front_bumper_horizontal_m:.4f} m")
    print(f"Maximum bumper-to-cone at first detection: {result['max_bumper_to_cone_at_first_detection_m']:.3f} m")
    print(f"Stopping distance including latency and margin: {result['stopping_distance_m']:.3f} m")
    for row in result["rows"]:
        verdict = "LESS THAN 0.5 m RUNWAY" if row["less_than_0_5_m"] else "at least 0.5 m runway"
        print(f"gap {row['stop_gap_m']:.2f} m: runway {row['runway_m']:+.3f} m — {verdict}")
    if any(row["runway_m"] < 0 for row in result["rows"]):
        print("Negative runway means stopping before the requested gap is impossible under these inputs.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
