#!/usr/bin/env python3
"""Read-only detected-cone metric check against tape-measured floor stations.

This runs the pinned detector, production depth association, and cone-foot
ground projection. It never opens a VCU port and never changes calibration or
readiness flags. Results remain diagnostic until independently reviewed.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PERCEPTION = ROOT / "autonomy_stack_phase1/phase_01_perception/src"
GEOMETRY_PACKAGE = ROOT / "kart/ros2_ws/src/autokart_vehicle_geometry"
sys.path[:0] = [str(PERCEPTION), str(GEOMETRY_PACKAGE), str(ROOT / "tools")]

from autokart_vehicle_geometry.geometry import GEOMETRY  # noqa: E402


def summarize_station(rows: list[dict], tape_camera_m: float,
                      bumper_offset_m: float) -> dict:
    """Summarize repeated cone-foot candidates without authorizing calibration."""
    if not rows:
        return {"tape_camera_m": tape_camera_m, "valid_samples": 0,
                "status": "NO_VALID_METRIC_SAMPLES"}
    xs = [float(row["cone_forward_m"]) for row in rows]
    ys = [float(row["cone_left_m"]) for row in rows]
    depths = [float(row["surface_depth_m"]) for row in rows]
    median_x = statistics.median(xs)
    expected_gap = tape_camera_m - bumper_offset_m
    measured_gap = median_x - bumper_offset_m
    error = median_x - tape_camera_m
    tolerance = max(0.05, 0.05 * tape_camera_m)
    return {
        "tape_camera_m": tape_camera_m,
        "expected_bumper_gap_m": expected_gap,
        "valid_samples": len(rows),
        "cone_forward_m": median_x,
        "cone_left_m": statistics.median(ys),
        "bumper_gap_m": measured_gap,
        "surface_depth_m": statistics.median(depths),
        "forward_span_m": max(xs) - min(xs),
        "error_m": error,
        "absolute_error_m": abs(error),
        "tolerance_m": tolerance,
        "status": "PASS_DIAGNOSTIC" if abs(error) <= tolerance else "FAIL_DIAGNOSTIC",
    }


def conenet_ort_detector(kind: str, confidence: float):
    """Load a pinned ConeNet with ORT without changing the dashboard detector."""
    import numpy as np
    import onnxruntime as ort
    from autokart_perception.detector import DetectorError, Yolo11OnnxDetector, _blob_and_geometry
    from autokart_perception.live_mission import (
        CONENET_CLASSES, CONENET_MANIFEST, CONENET_MODEL, CONENET_SHA256,
        CONENET_V2_MANIFEST, CONENET_V2_MODEL, CONENET_V2_SHA256,
    )

    model, manifest_path, expected_sha, display_name = (
        (CONENET_MODEL, CONENET_MANIFEST, CONENET_SHA256, "ConeNet v1")
        if kind == "conenet-v1" else
        (CONENET_V2_MODEL, CONENET_V2_MANIFEST, CONENET_V2_SHA256, "ConeNet v2")
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    digest = hashlib.sha256(model.read_bytes()).hexdigest()
    if (digest != expected_sha or digest != manifest.get("model_sha256") or
            tuple(manifest.get("classes", ())) != CONENET_CLASSES or
            manifest.get("format") != "ultralytics_yolo11_raw_onnx"):
        raise DetectorError("CONENET_IDENTITY_MISMATCH")

    class DiagnosticOrtYolo11(Yolo11OnnxDetector):
        def __init__(self):
            self.classes = list(CONENET_CLASSES)
            self.input_size = int(manifest["input_size"])
            self.confidence = confidence
            self.iou = 0.5
            self.version = model.name
            self.latency_ms = 0.0
            self.stage = manifest["stage"]
            self.backend = "ONNX Runtime CPU (diagnostic adapter)"
            self.supported_classes = tuple(self.classes)
            self.model_sha256 = digest
            self.verified_checkpoint = False
            self.display_name = display_name
            self.last_counts = {"raw_output_rows": 0, "above_confidence": 0,
                                "valid_boxes": 0, "after_nms": 0}
            options = ort.SessionOptions()
            options.intra_op_num_threads = 4
            options.inter_op_num_threads = 1
            options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            self.session = ort.InferenceSession(str(model), sess_options=options,
                                                providers=["CPUExecutionProvider"])

        def infer(self, image):
            blob, geometry = _blob_and_geometry(image, self.input_size)
            started = time.perf_counter()
            raw = self.session.run(None, {self.session.get_inputs()[0].name: blob})[0]
            self.latency_ms = (time.perf_counter() - started) * 1000
            channels = 4 + len(self.classes)
            if raw.ndim != 3 or raw.shape[0] != 1:
                raise DetectorError("UNSUPPORTED_MODEL_OUTPUT")
            if raw.shape[1] == channels:
                rows = raw[0].T
            elif raw.shape[2] == channels:
                rows = raw[0]
            else:
                raise DetectorError("UNSUPPORTED_MODEL_OUTPUT")
            return np.asarray(rows), geometry

    return DiagnosticOrtYolo11()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--distances", type=float, nargs="+", default=[1.55, 2.20, 3.00, 3.80],
                        help="tape distances from camera ground projection to cone foot")
    parser.add_argument("--frames", type=int, default=30,
                        help="valid metric samples required at each station")
    parser.add_argument("--station-timeout", type=float, default=45.0)
    parser.add_argument("--label", default="blue",
                        help="detector label of the single validation cone")
    parser.add_argument("--confidence", type=float,
                        help="temporary detector threshold for this diagnostic run")
    parser.add_argument("--detector", choices=("yolo26s", "conenet-v2", "conenet-v1", "coneslayer"),
                        default="yolo26s")
    parser.add_argument("--pitch-deg", type=float,
                        help="temporary optical pitch override for this diagnostic run only")
    parser.add_argument("--config", type=Path, default=PERCEPTION / "autokart_perception/data/perception.yaml")
    parser.add_argument("--out-dir", type=Path, default=ROOT / "results")
    args = parser.parse_args()
    if args.frames < 3 or args.station_timeout <= 0 or any(
            not math.isfinite(value) or value <= 0 for value in args.distances):
        parser.error("use at least 3 frames, a positive timeout, and positive finite distances")
    if args.confidence is not None and (
            not math.isfinite(args.confidence) or not 0 <= args.confidence <= 1):
        parser.error("--confidence must be finite and between 0 and 1")

    import cv2
    import yaml
    from autokart_perception.camera import CameraError, HP60C
    from autokart_perception.depth import localize_diagnostic
    from autokart_perception.live_mission import live_detector
    from autokart_perception.mount import mount_from_config

    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    if args.pitch_deg is not None:
        if not math.isfinite(args.pitch_deg) or not -60 <= args.pitch_deg <= 75:
            parser.error("--pitch-deg must be finite and between -60 and 75")
        config = {**config, "camera_to_kart": None,
                  "camera_mount": {**config["camera_mount"],
                                   "pitch_down_deg": args.pitch_deg}}
    mount, mount_status = mount_from_config(config)
    if mount_status != "MOUNT_CONFIGURED_UNVERIFIED":
        parser.error(f"usable diagnostic mount unavailable: {mount_status}")
    confidence = float(config["confidence"] if args.confidence is None else args.confidence)
    detector = (conenet_ort_detector(args.detector, confidence)
                if args.detector in {"conenet-v1", "conenet-v2"}
                else live_detector(args.detector))
    detector.confidence = confidence
    bumper_offset = float(GEOMETRY.camera_to_front_bumper_horizontal_m)

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = args.out_dir / f"cone_metric_check_{stamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    output = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "diagnostic_only": True,
        "calibration_verified": False,
        "detector": args.detector,
        "target_label": args.label,
        "confidence_threshold": confidence,
        "mount_status": mount_status,
        "camera_mount": config["camera_mount"],
        "temporary_pitch_override": args.pitch_deg is not None,
        "camera_to_front_bumper_horizontal_m": bumper_offset,
        "stations": [],
    }

    print("READ-ONLY CAMERA DIAGNOSTIC: no ARM, motor, brake, or VCU access.")
    print(f"Detector={args.detector}; target={args.label}; confidence>={confidence:.2f}; "
          f"mount={mount_status}")
    print(f"Camera height={config['camera_mount']['height_m']:.3f} m; "
          f"pitch down={config['camera_mount']['pitch_down_deg']:.1f} deg; "
          f"camera-to-bumper={bumper_offset:.3f} m")
    print("Keep traction isolated and leave exactly one target cone visible.")

    try:
        with HP60C() as camera:
            for station_index, tape_distance in enumerate(args.distances, 1):
                input(f"\nPlace the {args.label} cone foot at {tape_distance:.2f} m from the "
                      "camera ground projection, then press Enter...")
                rows: list[dict] = []
                rejected: dict[str, int] = {}
                deadline = time.monotonic() + args.station_timeout
                last_annotated = None
                while len(rows) < args.frames and time.monotonic() < deadline:
                    try:
                        frame = camera.read()
                        detections = [item for item in detector.detect(frame.rgb)
                                      if item.label == args.label]
                    except Exception as exc:
                        key = f"CAPTURE_OR_DETECT:{type(exc).__name__}:{exc}"
                        rejected[key] = rejected.get(key, 0) + 1
                        continue
                    if len(detections) != 1:
                        key = "TARGET_MISSING" if not detections else "TARGET_AMBIGUOUS"
                        rejected[key] = rejected.get(key, 0) + 1
                        continue
                    detection = detections[0]
                    cone, detail = localize_diagnostic(frame, detection, mount, config)
                    if cone is None or detail.get("localization_status") != "METRIC_OK":
                        key = detail.get("localization_status", "METRIC_FAILED")
                        rejected[key] = rejected.get(key, 0) + 1
                        continue
                    x, y = map(float, cone.xy)
                    row = {
                        "frame_timestamp_ns": frame.timestamp_ns,
                        "label": detection.label,
                        "confidence": float(detection.confidence),
                        "cone_forward_m": x,
                        "cone_left_m": y,
                        "bumper_gap_m": x - bumper_offset,
                        "surface_depth_m": float(detail["sampled_depth_m"]),
                        "uncertainty_m": float(detail["uncertainty_m"]),
                        "box_xyxy": list(map(int, detection.box)),
                    }
                    rows.append(row)
                    last_annotated = frame.rgb.copy()
                    x1, y1, x2, y2 = map(int, detection.box)
                    cv2.rectangle(last_annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    text = f"{args.label} x={x:.2f}m bumper={x-bumper_offset:+.2f}m"
                    cv2.putText(last_annotated, text, (x1, max(20, y1 - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
                    print(f"\r  valid {len(rows):02d}/{args.frames}: cone X={x:.3f} m, "
                          f"bumper gap={x-bumper_offset:+.3f} m, confidence={detection.confidence:.2f}",
                          end="", flush=True)
                print()
                summary = summarize_station(rows, tape_distance, bumper_offset)
                summary["rejections"] = rejected
                summary["readings"] = rows
                output["stations"].append(summary)
                if last_annotated is not None:
                    image_path = run_dir / f"station_{station_index}_{tape_distance:.2f}m.jpg"
                    cv2.imwrite(str(image_path), last_annotated)
                    summary["annotated_image"] = str(image_path)
                if summary["valid_samples"]:
                    print(f"  {summary['status']}: metric X={summary['cone_forward_m']:.3f} m; "
                          f"tape={tape_distance:.3f} m; error={summary['error_m']*100:+.1f} cm; "
                          f"bumper gap={summary['bumper_gap_m']:.3f} m")
                else:
                    print(f"  {summary['status']}; rejections={rejected}")
    except CameraError as exc:
        parser.error(f"camera unavailable: {exc}")

    output["overall_status"] = (
        "PASS_DIAGNOSTIC" if output["stations"] and all(
            item["status"] == "PASS_DIAGNOSTIC" for item in output["stations"])
        else "FAIL_DIAGNOSTIC"
    )
    path = run_dir / "summary.json"
    path.write_text(json.dumps(output, indent=2, allow_nan=False), encoding="utf-8")
    print(f"\nOverall: {output['overall_status']}")
    print(f"Saved evidence: {path}")
    print("No calibration/readiness flag was changed.")
    return 0 if output["overall_status"] == "PASS_DIAGNOSTIC" else 2


if __name__ == "__main__":
    raise SystemExit(main())
