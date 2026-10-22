#!/usr/bin/env python3
"""Live, read-only HP60C depth log for whatever cones the detector sees.

Streams camera frames for a fixed duration, runs the verified coneslayer
detector on each frame, and samples depth for every detected cone box. This
is a sensing diagnostic only: no actuation, no VCU access, no verified
calibration flags are touched or implied.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime, timezone

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "autonomy_stack_phase1", "phase_01_perception", "src")]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=30.0,
                        help="how long to stream and log depth readings")
    parser.add_argument("--model", default=os.path.join(
        ROOT, "autonomy_stack_phase1", "phase_01_perception", "models", "coneslayer-simplified.onnx"))
    parser.add_argument("--confidence", type=float, default=0.54)
    parser.add_argument("--out-dir", default=os.path.join(ROOT, "results"))
    parser.add_argument("--save-frames", action="store_true", default=True,
                        help="save one annotated RGB image per frame into its own run folder (default on)")
    parser.add_argument("--no-save-frames", dest="save_frames", action="store_false",
                        help="disable per-frame image saving")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    import cv2

    from autokart_perception.camera import HP60C, CameraError
    from autokart_perception.depth import sample_depth_diagnostic
    from autokart_perception.detector import ConeDetector, DetectorError
    from camera_visibility import PIPELINE_DEPTH_MIN_M, PIPELINE_DEPTH_MAX_M

    print("READ-ONLY DIAGNOSTIC. No ARM, CMD, or VCU access. Depth is a visible-surface sample, "
          "not a qualified cone-centre measurement.")
    print(f"Configured accepted depth is {PIPELINE_DEPTH_MIN_M:g} to {PIPELINE_DEPTH_MAX_M:g} m.")

    try:
        detector = ConeDetector(args.model, confidence=args.confidence)
    except DetectorError as exc:
        parser.error(f"detector unavailable: {exc}")

    run_stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = os.path.join(args.out_dir, "live_cone_depth_" + run_stamp)
    frames_dir = os.path.join(run_dir, "frames")
    if args.save_frames:
        os.makedirs(frames_dir, exist_ok=True)

    rows = []
    rejections: dict[str, int] = {}
    frames_read = 0
    deadline = time.monotonic() + args.seconds
    try:
        with HP60C() as camera:
            while time.monotonic() < deadline:
                try:
                    frame = camera.read()
                except CameraError as exc:
                    rejections[str(exc)] = rejections.get(str(exc), 0) + 1
                    continue
                frames_read += 1
                if not frame.aligned or not frame.intrinsics.valid():
                    rejections["ALIGNMENT_OR_INTRINSICS_UNAVAILABLE"] = \
                        rejections.get("ALIGNMENT_OR_INTRINSICS_UNAVAILABLE", 0) + 1
                    continue
                try:
                    raw_rows, geometry = detector.infer(frame.rgb)
                    detections = detector.postprocess(raw_rows, geometry)
                except DetectorError as exc:
                    rejections[f"DETECTOR:{exc}"] = rejections.get(f"DETECTOR:{exc}", 0) + 1
                    continue
                stamp = datetime.now(timezone.utc).isoformat()
                annotated = frame.rgb.copy() if args.save_frames else None
                for detection in detections:
                    box = detection.box
                    clipped = box[3] >= frame.rgb.shape[0] - 1
                    sample, reason = sample_depth_diagnostic(
                        frame.depth, box, frame.depth_scale,
                        min_m=PIPELINE_DEPTH_MIN_M, max_m=PIPELINE_DEPTH_MAX_M, clipped=clipped)
                    row = {"timestamp_utc": stamp, "frame_timestamp_ns": frame.timestamp_ns,
                           "cone_index": detection.id, "box_xyxy": list(map(int, box)),
                           "confidence": float(detection.confidence), "depth_status": reason}
                    if sample is not None:
                        z, sigma, coverage, u, v = sample
                        row.update(depth_m=z, depth_sigma_m=sigma, depth_coverage=coverage,
                                   sample_uv=[u, v])
                        label = f"cone{detection.id} {z:.2f}m"
                        print(f"[{stamp}] cone {detection.id}: {z:.3f} m "
                              f"(sigma {sigma:.3f} m, confidence {detection.confidence:.2f})")
                    else:
                        label = f"cone{detection.id} NO_DEPTH"
                        print(f"[{stamp}] cone {detection.id}: NO_DEPTH "
                              f"({reason}, confidence {detection.confidence:.2f})")
                    rows.append(row)
                    if annotated is not None:
                        x1, y1, x2, y2 = map(int, box)
                        color = (0, 255, 0) if sample is not None else (0, 0, 255)
                        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(annotated, label, (x1, max(0, y1 - 6)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)
                if annotated is not None:
                    # frame.rgb is HP60C SDK BGR8 (see autokart_perception.camera), already
                    # the right channel order for cv2.imwrite.
                    frame_path = os.path.join(frames_dir, f"frame_{frames_read:06d}.jpg")
                    cv2.imwrite(frame_path, annotated)
    except CameraError as exc:
        parser.error(f"camera unavailable: {exc}")
    finally:
        pass

    valid = [r for r in rows if "depth_m" in r]
    output = {"created_utc": datetime.now(timezone.utc).isoformat(), "diagnostic_only": True,
              "requested_seconds": args.seconds, "frames_read": frames_read,
              "detections_logged": len(rows), "detections_with_depth": len(valid),
              "frames_dir": frames_dir if args.save_frames else None,
              "frame_rejections": rejections, "readings": rows}
    os.makedirs(run_dir, exist_ok=True)
    path = os.path.join(run_dir, "summary.json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(output, handle, indent=2, allow_nan=False)

    print(f"Frames read: {frames_read}. Detections logged: {len(rows)} "
          f"({len(valid)} with a valid depth sample).")
    if rejections:
        print(f"Frame/detector rejections: {rejections}")
    if args.save_frames:
        print(f"Saved {frames_read} frame images to {frames_dir}")
    print(f"Saved {path}")
    print("No verified flags were changed. This is a live sensing diagnostic only.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
