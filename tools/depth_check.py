#!/usr/bin/env python3
"""Stationary HP60C cone depth survey. Diagnostic only; never changes verified flags.

The operator tapes from the camera's ground projection to a cone base and selects the cone
body in the RGB frame. The reported position is a projected visible-surface sample, so its
error includes cone-body parallax as well as depth error. Retain the JSON for calibration review.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
from datetime import datetime, timezone

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "autonomy_stack_phase1", "phase_01_perception", "src"),
                os.path.join(ROOT, "kart", "ros2_ws", "src", "autokart_vehicle_geometry")]


def budget_m(distance_m: float) -> float:
    return 0.05 if distance_m <= 1.2 else 0.05 * distance_m


def projected_forward_m(depth_m: float, pixel_v: float, fy: float, cy: float, pitch_deg: float) -> float:
    angle = math.radians(pitch_deg)
    return depth_m * (math.cos(angle) - (pixel_v - cy) / fy * math.sin(angle))


def pitch_residuals(stations: list[dict], pitch_deg: float) -> list[dict]:
    """Station medians keep one high-frame-rate station from dominating the fit."""
    residuals = []
    for station in stations:
        samples = station.get("raw_samples", [])
        if not samples:
            continue
        estimates = [projected_forward_m(s["depth_m"], s["pixel_v"], s["fy"], s["cy"], pitch_deg)
                     for s in samples]
        residuals.append({"target_m": station["target_m"], "estimate_m": statistics.median(estimates),
                          "signed_error_m": statistics.median(estimates) - station["target_m"]})
    return residuals


def fit_pitch(stations: list[dict], *, configured_deg: float = 17.8,
              minimum_deg: float = 5.0, maximum_deg: float = 25.0) -> dict:
    """Diagnostic 0.01 degree grid search; requires four distinct measured stations."""
    usable = [row for row in stations if row.get("raw_samples")]
    if len({float(row["target_m"]) for row in usable}) < 4:
        raise ValueError("pitch fit needs samples at four distinct tape-measured stations")
    if not (math.isfinite(minimum_deg) and math.isfinite(maximum_deg) and
            0 < minimum_deg < maximum_deg < 90):
        raise ValueError("invalid pitch search range")

    def score(pitch: float) -> float:
        errors = pitch_residuals(usable, pitch)
        return statistics.mean(row["signed_error_m"] ** 2 for row in errors)

    grid = (minimum_deg + step * 0.01 for step in range(int((maximum_deg - minimum_deg) * 100) + 1))
    fitted = min(grid, key=score)
    candidates = {name: {"pitch_deg": pitch, "rmse_m": math.sqrt(score(pitch)),
                         "residuals": pitch_residuals(usable, pitch)}
                  for name, pitch in (("fitted", fitted), ("configured", configured_deg),
                                      ("old_ground_plane", 11.4))}
    boundary = fitted <= minimum_deg + 0.01 or fitted >= maximum_deg - 0.01
    within_budget = all(abs(row["signed_error_m"]) <= budget_m(row["target_m"])
                        for row in candidates["fitted"]["residuals"])
    candidates["recommendation"] = ("PROVISIONAL_FIT_REVIEW" if within_budget and not boundary
                                    else "NO_CONFIG_CHANGE_FIT_UNRELIABLE")
    candidates["diagnostic_only"] = True
    return candidates


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--distances", nargs="+", type=float, default=[1.2, 2, 3, 4])
    parser.add_argument("--fit-pitch", action="store_true",
                        help="fit 5-25 degrees from four taped stations; diagnostic only")
    parser.add_argument("--out-dir", default=os.path.join(ROOT, "results"))
    args = parser.parse_args()
    if args.frames < 3 or any(d <= 0 or not math.isfinite(d) for d in args.distances):
        parser.error("use at least 3 frames and positive finite tape distances")

    import cv2
    from autokart_perception.camera import HP60C, CameraError
    from autokart_perception.depth import sample_depth_diagnostic
    from autokart_vehicle_geometry.geometry import GEOMETRY
    from camera_visibility import PIPELINE_DEPTH_MIN_M, PIPELINE_DEPTH_MAX_M

    results = []
    print("STATIONARY DIAGNOSTIC ONLY. HP60C RAW16 scale, alignment, and mount remain unqualified.")
    print(f"Configured accepted depth is {PIPELINE_DEPTH_MIN_M:g} to {PIPELINE_DEPTH_MAX_M:g} m. "
          "Stations outside this range cannot qualify the fit.")
    try:
        with HP60C() as camera:
            for target in args.distances:
                input(f"Place one cone at {target:g} m tape-measured camera-forward, then press Enter...")
                frame = None
                for _ in range(10):
                    try:
                        frame = camera.read()
                        break
                    except CameraError:
                        continue
                if frame is None:
                    results.append({"target_m": target, "status": "NO_VALID_FRAME", "samples_m": []})
                    continue
                x, y, w, h = map(int, cv2.selectROI("Select cone body", frame.rgb, showCrosshair=True))
                cv2.destroyWindow("Select cone body")
                if w <= 0 or h <= 0:
                    results.append({"target_m": target, "status": "ROI_NOT_SELECTED", "samples_m": []})
                    continue
                box = (x, y, x + w, y + h)
                samples = []
                raw_samples = []
                rejections = {}
                for _ in range(args.frames):
                    try:
                        frame = camera.read()
                    except CameraError as exc:
                        reason = str(exc)
                        rejections[reason] = rejections.get(reason, 0) + 1
                        continue
                    if not frame.aligned or not frame.intrinsics.valid():
                        rejections["ALIGNMENT_OR_INTRINSICS_UNAVAILABLE"] = rejections.get("ALIGNMENT_OR_INTRINSICS_UNAVAILABLE", 0) + 1
                        continue
                    sample, reason = sample_depth_diagnostic(frame.depth, box, frame.depth_scale,
                                                             min_m=PIPELINE_DEPTH_MIN_M,
                                                             max_m=PIPELINE_DEPTH_MAX_M)
                    if sample is None:
                        rejections[reason] = rejections.get(reason, 0) + 1
                        continue
                    z, _sigma, _coverage, _u, v = sample
                    raw_samples.append({"depth_m": z, "pixel_v": v,
                                        "fy": frame.intrinsics.fy, "cy": frame.intrinsics.cy})
                    samples.append(projected_forward_m(z, v, frame.intrinsics.fy, frame.intrinsics.cy,
                                                       GEOMETRY.camera_pitch_down_deg))
                row = {"target_m": target, "roi_xywh": [x, y, w, h], "requested_frames": args.frames,
                       "samples_m": samples, "raw_samples": raw_samples, "rejections": rejections,
                       "budget_m": budget_m(target), "depth_scale_qualified": False,
                       "alignment_qualified": False, "mount_qualified": False}
                if samples:
                    median = statistics.median(samples)
                    row.update(median_measured_m=median, signed_error_cm=100 * (median - target),
                               absolute_error_cm=100 * abs(median - target))
                row["status"] = ("OUTSIDE_CONFIGURED_DEPTH_RANGE" if target > PIPELINE_DEPTH_MAX_M else
                                 "PASS_DIAGNOSTIC" if samples and abs(row["median_measured_m"] - target) <= row["budget_m"] else
                                 "FAIL_DIAGNOSTIC")
                results.append(row)
                print(f"{target:g} m: {row['status']}, {len(samples)}/{args.frames} samples, "
                      f"median {row.get('median_measured_m', float('nan')):.3f} m, "
                      f"error {row.get('signed_error_cm', float('nan')):+.1f} cm, "
                      f"budget {100 * row['budget_m']:.1f} cm")
    finally:
        cv2.destroyAllWindows()

    errors = [r["absolute_error_cm"] for r in results if "absolute_error_cm" in r]
    suggested_gap_m = max([GEOMETRY.stop_gap_from_bumper_m] + [e / 100 for e in errors])
    output = {"created_utc": datetime.now(timezone.utc).isoformat(), "diagnostic_only": True,
              "camera_geometry_source": GEOMETRY.source_path, "results": results,
              "largest_observed_absolute_error_cm": max(errors) if errors else None,
              "minimum_gap_indicated_by_observed_error_m": suggested_gap_m,
              "commissioning_gap_m": 1.0,
              "note": "Visible cone surface is compared with tape distance to cone base. This is not a qualified depth calibration."}
    if args.fit_pitch:
        fit_stations = [row for row in results if row.get("target_m", float("inf")) <= PIPELINE_DEPTH_MAX_M]
        try:
            comparison = fit_pitch(fit_stations, configured_deg=GEOMETRY.camera_pitch_down_deg)
            output["pitch_fit"] = comparison
            for name in ("fitted", "configured", "old_ground_plane"):
                item = comparison[name]
                print(f"{name}: {item['pitch_deg']:.2f} deg, RMSE {item['rmse_m']:.3f} m")
                for residual in item["residuals"]:
                    print(f"  {residual['target_m']:g} m: {residual['signed_error_m']:+.3f} m")
            print(f"Pitch recommendation: {comparison['recommendation']} (diagnostic only)")
            if comparison["recommendation"] == "PROVISIONAL_FIT_REVIEW":
                print("Candidate lines for human review only; do not mark calibration verified:")
                print("autonomy_stack_phase1/phase_01_perception/src/autokart_perception/data/perception.yaml:")
                print("  height_m: 0.8382")
                print(f"  pitch_down_deg: {comparison['fitted']['pitch_deg']:.2f}")
                print("  camera_to_front_bumper_horizontal_m: 1.0668  # operator_measurements")
                print("  bumper_width_m: 1.016  # operator_measurements")
        except ValueError as exc:
            output["pitch_fit"] = {"diagnostic_only": True, "recommendation": "NO_CONFIG_CHANGE_INSUFFICIENT_DATA",
                                   "reason": str(exc)}
            print(f"Pitch recommendation: NO_CONFIG_CHANGE_INSUFFICIENT_DATA ({exc})")
    os.makedirs(args.out_dir, exist_ok=True)
    path = os.path.join(args.out_dir, "depth_check_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(output, handle, indent=2, allow_nan=False)
    print(f"Saved {path}")
    print(f"Observed error alone indicates a stop gap of at least {suggested_gap_m:.3f} m; "
          "start commissioning at 1.0 m and include braking and latency margins separately.")
    print("No verified flags were changed. A diagnostic pass does not qualify depth scale or the camera mount.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
