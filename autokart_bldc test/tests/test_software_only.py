#!/usr/bin/env python3
"""Software-only checks runnable without arming or moving anything.

Run: python3 tests/test_software_only.py
Prints one JSON object per check; exit code is nonzero if any check with
requires_hardware=False failed.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import yaml  # noqa: E402

from camera.hp60c_bridge import probe as camera_probe  # noqa: E402
from perception.cone_detector import probe as detector_probe, load_detector  # noqa: E402
from perception.cone_geometry import probe as geometry_probe  # noqa: E402
from planning.obstacle_checker import check_obstacle  # noqa: E402
from control.steering_controller import preview_steering  # noqa: E402
from control.throttle_preview import ThrottlePreview  # noqa: E402
from control.stop_controller import StopController, StopControllerInputs  # noqa: E402
from vcu.serial_interface import probe as vcu_probe  # noqa: E402

sys.path.insert(0, str(ROOT.parent / "autonomy_stack_phase1/phase_01_perception/src"))
from autokart_perception.types import Cone, PathSample  # noqa: E402


def _record(results, name, requires_hardware, ok, detail):
    results.append({"test": name, "requires_hardware": requires_hardware, "pass": ok, "detail": detail})


def main() -> int:
    cfg = yaml.safe_load((ROOT / "config/test_config.yaml").read_text())
    results = []

    cam = camera_probe(cfg["camera"]["bridge_library"], cfg["camera"]["camera_configuration"])
    _record(results, "camera_detection", True, cam["pass"], cam["detail"])

    det = detector_probe(cfg["perception"]["detector"])
    _record(results, "detector_load", False, det["pass"], det["detail"])

    try:
        detector = load_detector(cfg["perception"]["detector"])
    except Exception:
        detector = None
    geo = geometry_probe(detector) if detector is not None else {"pass": False, "detail": "detector unavailable"}
    _record(results, "geometry_engine_init", False, geo["pass"], geo["detail"])

    class FakeResult:
        def __init__(self, cones):
            self.cones = cones
    blocking_cone = Cone(id=1, label="large_orange", confidence=0.9, xy=(2.0, 0.0),
                        sigma_m=0.05, distance_m=2.0, box=(0, 0, 10, 10), observed_ns=1)
    clear_result = FakeResult([])
    blocked_result = FakeResult([blocking_cone])
    plan_cfg = {**cfg["planning"], **cfg["stopping"]}
    try:
        clear = check_obstacle(clear_result, plan_cfg, current_speed_mps=0.3)
        blocked = check_obstacle(blocked_result, plan_cfg, current_speed_mps=0.3)
        ok = (not clear.blocked) and blocked.blocked and not blocked.clearance_qualified
        _record(results, "obstacle_checker_logic", False, ok,
               {"clear_blocked": clear.blocked, "blocked_blocked": blocked.blocked,
                "blocked_clearance_qualified": blocked.clearance_qualified,
                "blocked_required_clearance_m": blocked.required_clearance_m})
    except Exception as exc:
        _record(results, "obstacle_checker_logic", False, False, f"{type(exc).__name__}: {exc}")

    try:
        profile = [PathSample(xy=(1.0, 0.0), heading_rad=0.0, segment_m=1.0, curvature_m_inv=0.2, observed_ns=1)]
        preview = preview_steering(profile, cfg["perception"]["wheelbase_m"])
        _record(results, "steering_preview_logic", False, preview.valid,
               {"steering_angle_rad": preview.steering_angle_rad})
    except Exception as exc:
        _record(results, "steering_preview_logic", False, False, f"{type(exc).__name__}: {exc}")

    try:
        throttle = ThrottlePreview(cfg["throttle"])
        disabled_state = throttle.update(1.0, 0.1, armed=True, stop_requested=False)
        ok = disabled_state.dac_mv == 0  # motor_output_enabled is false by default in test_config.yaml
        _record(results, "throttle_preview_disabled_by_default", False, ok,
               {"dac_mv": disabled_state.dac_mv, "motor_output_enabled": cfg["throttle"]["motor_output_enabled"]})
    except Exception as exc:
        _record(results, "throttle_preview_disabled_by_default", False, False, f"{type(exc).__name__}: {exc}")

    try:
        sc = StopController()
        ok_states = []
        for _ in range(3):
            out = sc.step(StopControllerInputs(
                camera_ok=True, frame_stale=False, depth_valid=True, corridor_status="PROVISIONAL",
                obstacle_blocked=False, obstacle_clearance_qualified=False, obstacle_forward_m=None,
                vcu_link_ok=True, safety_state="OK", vehicle_authorized=True))
            ok_states.append(out.state)
        blocked_out = sc.step(StopControllerInputs(
            camera_ok=True, frame_stale=False, depth_valid=True, corridor_status="PROVISIONAL",
            obstacle_blocked=True, obstacle_clearance_qualified=False, obstacle_forward_m=0.5,
            vcu_link_ok=True, safety_state="OK", vehicle_authorized=True))
        ok = ok_states[-1] == "DRIVING" and blocked_out.state == "STOP_REQUESTED" and blocked_out.decision == 0
        _record(results, "stop_controller_state_machine", False, ok,
               {"progression": ok_states, "after_obstacle": blocked_out.state})
    except Exception as exc:
        _record(results, "stop_controller_state_machine", False, False, f"{type(exc).__name__}: {exc}")

    vcu = vcu_probe(cfg["vcu"]["port"], cfg["vcu"]["baud"])
    _record(results, "pico_vcu_passive_connectivity", True, vcu["pass"], vcu["detail"])

    for r in results:
        print(json.dumps(r))

    software_only_failed = [r for r in results if not r["requires_hardware"] and not r["pass"]]
    return 1 if software_only_failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
