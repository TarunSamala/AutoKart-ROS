#!/usr/bin/env python3
"""Minimal HP60C blue-cone guard for the proven MicroPython BLDC controller.

This process has exactly one actuator capability: HTTP GET /lock.  It cannot
arm, release the brake, select a gear, or command motion.  Loss of this process
is independently handled by the controller's 750 ms fail-closed link timeout.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
PERCEPTION = ROOT / "autonomy_stack_phase1/phase_01_perception/src"
GEOMETRY_PACKAGE = ROOT / "kart/ros2_ws/src/autokart_vehicle_geometry"
sys.path[:0] = [str(PERCEPTION), str(GEOMETRY_PACKAGE), str(ROOT / "tools")]

from autokart_vehicle_geometry.geometry import GEOMETRY  # noqa: E402


def get(url: str, timeout: float = 0.4) -> str:
    with urlopen(url, timeout=timeout) as response:
        return response.read().decode("utf-8")


def controller_status(base: str) -> dict:
    return json.loads(get(base + "/status"))


def lock_and_verify(base: str) -> dict:
    reply = get(base + "/lock")
    status = controller_status(base)
    if not (status.get("wheel_locked") and not status.get("armed") and
            not status.get("running") and not status.get("brake_released") and
            float(status.get("voltage", 1)) <= 0.01 and
            float(status.get("target_voltage", 1)) <= 0.01):
        raise RuntimeError("LOCK_VERIFICATION_FAILED: " + json.dumps(status, sort_keys=True))
    return {"reply": reply, "status": status}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", default="http://192.168.4.1")
    parser.add_argument("--trigger-gap", type=float, default=1.0,
                        help="apply full BRK at or below this bumper-to-cone gap")
    parser.add_argument("--target-gap", type=float, default=0.40,
                        help="required final bumper gap, evidence only")
    parser.add_argument("--pitch-deg", type=float, default=16.37)
    parser.add_argument("--confidence", type=float, default=0.80)
    parser.add_argument("--confirmations", type=int, default=2)
    parser.add_argument("--enable-lock", action="store_true",
                        help="permit the sole actuator action: GET /lock")
    parser.add_argument("--out-dir", type=Path, default=ROOT / "results")
    args = parser.parse_args()
    if not (0.40 <= args.trigger_gap <= 2.0 and 0.30 <= args.target_gap <= args.trigger_gap):
        parser.error("require 0.40 <= target-gap <= trigger-gap <= 2.0 m")
    if not (0.5 <= args.confidence <= 1.0) or not (1 <= args.confirmations <= 5):
        parser.error("invalid confidence or confirmation count")

    import yaml
    from autokart_perception.camera import CameraError, HP60C
    from autokart_perception.depth import localize_diagnostic
    from autokart_perception.live_mission import live_detector
    from autokart_perception.mount import mount_from_config

    config_path = PERCEPTION / "autokart_perception/data/perception.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    config = {**config, "camera_to_kart": None,
              "camera_mount": {**config["camera_mount"],
                               "pitch_down_deg": args.pitch_deg}}
    mount, mount_status = mount_from_config(config)
    if mount_status != "MOUNT_CONFIGURED_UNVERIFIED":
        raise SystemExit("NO-GO: camera mount unavailable: " + mount_status)
    detector = live_detector("yolo26s")
    detector.confidence = args.confidence

    initial = controller_status(args.controller)
    if not initial.get("gp_online") or not initial.get("encoder_ready"):
        raise SystemExit("NO-GO: GP8630N or AS5047P not healthy: " + json.dumps(initial))
    if args.enable_lock and not initial.get("wheel_locked"):
        raise SystemExit("NO-GO: start the guard with the controller locked")
    if args.enable_lock:
        phrase = input("Wheels raised, kart secured, power cut ready. Type WHEELS-OFF: ").strip()
        if phrase != "WHEELS-OFF":
            raise SystemExit("No actuator access enabled")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    evidence_path = args.out_dir / ("cone_brake_guard_" + stamp + ".json")
    evidence = {"created_utc": datetime.now(timezone.utc).isoformat(),
                "mode": "LOCK_ENABLED" if args.enable_lock else "DRY_RUN",
                "trigger_gap_m": args.trigger_gap, "target_gap_m": args.target_gap,
                "pitch_deg": args.pitch_deg, "confidence": args.confidence,
                "controller_initial": initial, "samples": [], "lock": None}
    gaps = deque(maxlen=args.confirmations)
    bumper_offset = float(GEOMETRY.camera_to_front_bumper_horizontal_m)
    corridor_half_width = float(GEOMETRY.bumper_width_m) / 2 + float(GEOMETRY.lateral_margin_m)
    motion_seen = False
    last_print = 0.0
    return_code = 0
    camera_warmup_deadline = time.monotonic() + 10.0

    print("CONE BRAKE GUARD", evidence["mode"])
    print("Only permitted actuator endpoint: /lock")
    print(f"Trigger <= {args.trigger_gap:.2f} m bumper gap; final target >= {args.target_gap:.2f} m")
    print(f"Blue confidence >= {args.confidence:.2f}; corridor |y| <= {corridor_half_width:.3f} m")

    try:
        with HP60C() as camera:
            while True:
                status = controller_status(args.controller)  # heartbeat and health check
                motion_seen |= bool(status.get("armed") or status.get("running") or
                                    float(status.get("voltage", 0)) > 0.01)
                if not status.get("gp_online") or not status.get("encoder_ready"):
                    raise RuntimeError("CONTROLLER_HEALTH_LOST")
                try:
                    frame = camera.read()
                except CameraError as exc:
                    if not motion_seen and time.monotonic() < camera_warmup_deadline:
                        print("CAMERA WARM-UP:", exc)
                        continue
                    raise
                candidates = []
                for detection in detector.detect(frame.rgb):
                    if detection.label != "blue" or detection.confidence < args.confidence:
                        continue
                    cone, detail = localize_diagnostic(frame, detection, mount, config)
                    if cone is None or detail.get("localization_status") != "METRIC_OK":
                        continue
                    x, y = map(float, cone.xy)
                    gap = x - bumper_offset
                    if x > bumper_offset and abs(y) <= corridor_half_width:
                        candidates.append((gap, x, y, float(detection.confidence)))
                if not candidates:
                    gaps.clear()
                    if time.monotonic() - last_print >= 1.0:
                        print("NO BLOCKING BLUE CONE")
                        last_print = time.monotonic()
                    continue
                gap, x, y, confidence = min(candidates)
                gaps.append(gap)
                row = {"time_ns": time.time_ns(), "camera_forward_m": x,
                       "bumper_gap_m": gap, "cone_left_m": y,
                       "confidence": confidence, "speed_kmh": status.get("speed_kmh"),
                       "voltage": status.get("voltage")}
                evidence["samples"].append(row)
                print(f"BLUE gap={gap:.3f} m y={y:+.3f} m conf={confidence:.2f} "
                      f"speed={float(status.get('speed_kmh', 0)):.2f} km/h")
                if len(gaps) == args.confirmations and statistics.median(gaps) <= args.trigger_gap:
                    if not args.enable_lock:
                        print("DRY-RUN TRIGGER: lock would be applied")
                        evidence["lock"] = {"dry_run": True, "trigger_gap_m": statistics.median(gaps)}
                        break
                    result = lock_and_verify(args.controller)
                    evidence["lock"] = {"dry_run": False,
                                        "trigger_gap_m": statistics.median(gaps), **result}
                    print("FULL BRAKE VERIFIED: SV=0 V, EN=STOP, BRK=APPLIED")
                    break
    except KeyboardInterrupt:
        print("INTERRUPTED")
        return_code = 130
    except Exception as exc:
        evidence["error"] = f"{type(exc).__name__}: {exc}"
        print("GUARD FAULT:", evidence["error"], file=sys.stderr)
        if args.enable_lock:
            evidence["fault_lock"] = lock_and_verify(args.controller)
        return_code = 2
    else:
        return_code = 0
    finally:
        if args.enable_lock and motion_seen and evidence.get("lock") is None:
            try:
                evidence["exit_lock"] = lock_and_verify(args.controller)
            except Exception as exc:
                evidence["exit_lock_error"] = str(exc)
        evidence_path.write_text(json.dumps(evidence, indent=2, allow_nan=False), encoding="utf-8")
        print("Evidence:", evidence_path)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
