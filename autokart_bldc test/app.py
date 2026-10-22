#!/usr/bin/env python3
"""Experimental AutoKart BLDC test harness -- orchestrator.

Wires: camera -> detector -> cone geometry -> corridor -> obstacle check ->
steering/throttle preview -> stop-controller state machine -> passive Pico VCU monitor
(best-effort; absent hardware is reported, not faked) -> dashboard + CSV log.

This file never transmits any VCU command. All actuator values are previews.
STOP REQUEST records a software decision only, not an acknowledged Pico stop.

    python3 app.py --mode live             # real camera + passive Pico serial monitor
    python3 app.py --mode camera-only      # perception only, no VCU connection attempt
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from camera.hp60c_bridge import CameraBridge  # noqa: E402
from perception.cone_detector import load_detector  # noqa: E402
from perception.cone_geometry import ConeGeometryEngine  # noqa: E402
from planning.corridor_planner import build_corridor  # noqa: E402
from planning.obstacle_checker import check_obstacle  # noqa: E402
from control.steering_controller import preview_steering  # noqa: E402
from control.throttle_preview import ThrottlePreview  # noqa: E402
from control.stop_controller import StopController, StopControllerInputs  # noqa: E402
from vcu.serial_interface import VCUInterface  # noqa: E402
from dashboard.server import AppState, run_server  # noqa: E402

PERCEPTION_SRC = ROOT.parent / "autonomy_stack_phase1/phase_01_perception/src"
sys.path.insert(0, str(PERCEPTION_SRC))
from autokart_perception.live_render import encode_jpeg  # noqa: E402
from autokart_perception.camera import CameraError  # noqa: E402
from dashboard.render import annotated_rgb  # noqa: E402


def load_config() -> dict:
    return yaml.safe_load((ROOT / "config/test_config.yaml").read_text())


def open_csv(results_dir: Path):
    results_dir.mkdir(parents=True, exist_ok=True)
    path = results_dir / f"run_{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}.csv"
    fh = open(path, "w", newline="")
    writer = csv.writer(fh)
    writer.writerow(["t_s", "state", "decision", "obstacle_forward_m", "bumper_gap_estimate_m",
                      "target_speed_mps", "dac_mv", "steering_angle_rad", "vcu_connected",
                      "vcu_link_fresh", "reason"])
    return path, fh, writer


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Experimental AutoKart BLDC test harness")
    parser.add_argument("--mode", choices=("live", "camera-only"), default="live")
    parser.add_argument("--frames", type=int, default=0, help="0 = run until Ctrl-C")
    args = parser.parse_args(argv)

    cfg = load_config()
    if cfg["throttle"]["motor_output_enabled"]:
        raise SystemExit("motor_output_enabled must remain false in this passive Pico harness")
    state = AppState()
    server = run_server(state, cfg["dashboard"]["host"], cfg["dashboard"]["port"])
    print(f"dashboard: http://{cfg['dashboard']['host']}:{cfg['dashboard']['port']}/")

    detector_probe_error = None
    try:
        detector = load_detector(cfg["perception"]["detector"])
    except Exception as exc:
        detector = None
        detector_probe_error = f"{type(exc).__name__}: {exc}"

    camera = CameraBridge(bridge_library=cfg["camera"]["bridge_library"],
                          camera_configuration=cfg["camera"]["camera_configuration"],
                          max_frame_age_ms=cfg["camera"]["max_frame_age_ms"],
                          warmup_attempts=cfg["camera"]["warmup_read_attempts"])
    geometry = ConeGeometryEngine(detector) if detector is not None else None
    throttle = ThrottlePreview(cfg["throttle"])
    stop_controller = StopController()

    vcu = None
    if args.mode == "live":
        vcu = VCUInterface(cfg["vcu"]["port"], cfg["vcu"]["baud"])
        if vcu.start():
            print(f"Pico serial: opened read-only on {cfg['vcu']['port']}; awaiting matching HELLO")
        else:
            print(f"Pico serial: NOT connected ({vcu.connect_error}) -- continuing perception-only")

    results_dir = ROOT / cfg["logging"]["results_dir"]
    csv_path, csv_fh, csv_writer = open_csv(results_dir)
    print(f"logging to {csv_path}")

    frame_count = 0
    t_start = time.monotonic()
    last_t = t_start
    navigation_preview_enabled = False
    perception_enabled = True
    operator_stop_latched = False
    try:
        camera.start()
        while args.frames == 0 or frame_count < args.frames:
            for action in state.drain_actions():
                if action == "camera_start":
                    if not camera.started:
                        camera.start()
                elif action == "camera_stop":
                    camera.stop()
                elif action == "perception_start":
                    perception_enabled = True
                elif action == "perception_stop":
                    perception_enabled = False
                elif action == "navigation_preview_on":
                    navigation_preview_enabled = True
                elif action == "navigation_preview_off":
                    navigation_preview_enabled = False
                elif action == "stop_request":
                    operator_stop_latched = True
                elif action == "reset_fault":
                    operator_stop_latched = False
                    stop_controller.reset_fault()

            now = time.monotonic()
            dt_s = now - last_t
            last_t = now

            try:
                frame = camera.read(timeout_ms=1000)
                camera_ok = True
            except CameraError as exc:
                camera_ok = False
                frame = None
                camera_error = str(exc)

            corridor_status = "NO_PATH"
            safety_state = "SENSOR_FAULT"
            vehicle_authorized = False
            obstacle = None
            steering = None
            birds_eye = []
            depth_valid = False
            frame_stale = not camera_ok
            result = None
            corridor = None
            jpeg = None
            detections = []
            pipeline = {}
            frame_id = None

            if camera_ok and perception_enabled and geometry is not None:
                result = geometry.process(frame)
                depth_valid = result.status not in ("DEPTH_INVALID", "STALE_FRAME", "INVALID_FRAME_FORMAT")
                corridor = build_corridor(result)
                corridor_status = corridor.status
                safety_state = result.diagnostics.get("safety_state", "UNKNOWN")
                vehicle_authorized = bool(result.diagnostics.get("vehicle_authorized", False))
                obstacle = check_obstacle(result, {**cfg["planning"], **cfg["stopping"]}, current_speed_mps=0.0)
                steering = preview_steering(result.path_profile, cfg["perception"]["wheelbase_m"])
                birds_eye = geometry.birds_eye_points(result)
                cone_by_id = {cone.id: cone for cone in result.cones}
                detail_by_id = {item.get("id"): item for item in result.diagnostics.get("cone_details", [])}
                detections = [{
                    "id": item.id, "label": item.label, "confidence": item.confidence,
                    "box": list(item.box),
                    "candidate_distance_m": (cone_by_id[item.id].distance_m
                                             if item.id in cone_by_id else None),
                    "localization_status": detail_by_id.get(item.id, {}).get("localization_status", "UNAVAILABLE"),
                    "validation_reasons": detail_by_id.get(item.id, {}).get("validation_reasons", []),
                } for item in result.detections]
                diag = result.diagnostics
                pipeline = {
                    "model": getattr(detector, "display_name", cfg["perception"]["detector"]),
                    "detector_kind": cfg["perception"]["detector"],
                    "detected": len(result.detections),
                    "localized_candidates": len(result.cones),
                    "rejected_metric": diag.get("rejected_metric_cones"),
                    "inference_ms": diag.get("inference_ms"),
                    "processing_ms": diag.get("processing_ms"),
                    "rgb_fps": diag.get("rgb_fps"),
                    "depth_fps": diag.get("depth_fps"),
                    "valid_depth_pct": diag.get("valid_depth_pct"),
                    "frame_age_ms": diag.get("frame_age_ms"),
                    "graph_status": result.delaunay.status,
                    "delaunay_edges": len(result.delaunay.edges),
                    "gabriel_edges": len(result.gabriel_edges),
                    "left_boundary_cones": diag.get("left_boundary_cones", 0),
                    "right_boundary_cones": diag.get("right_boundary_cones", 0),
                    "candidate_gates": diag.get("candidate_gates", 0),
                    "centerline_status": diag.get("centerline_status"),
                    "curvature_status": diag.get("curvature_status"),
                    "safety_reasons": list(diag.get("safety_reasons", [])),
                    "mount_status": getattr(geometry.pipeline, "mount_status", "UNVERIFIED"),
                }
                jpeg = encode_jpeg(annotated_rgb(frame, result))
            elif camera_ok:
                jpeg = encode_jpeg(frame.rgb)
            if camera_ok:
                frame_id = f"{frame.rgb_sequence}-{frame.depth_sequence}-{frame.timestamp_ns}"

            vcu_connected = vcu is not None and vcu.connected
            vcu_link_fresh = vcu.is_link_fresh(cfg["vcu"]["link_timeout_ms"]) if vcu_connected else False

            inputs = StopControllerInputs(
                camera_ok=camera_ok, frame_stale=frame_stale, depth_valid=depth_valid,
                corridor_status=corridor_status,
                obstacle_blocked=bool(obstacle.blocked) if obstacle else False,
                obstacle_clearance_qualified=bool(obstacle.clearance_qualified) if obstacle else False,
                obstacle_forward_m=obstacle.nearest_forward_m if obstacle else None,
                vcu_link_ok=vcu_link_fresh if vcu is not None else True,
                safety_state=safety_state, vehicle_authorized=vehicle_authorized,
            )
            output = stop_controller.step(inputs, cruise_speed_mps=0.3 if navigation_preview_enabled else 0.0)
            if operator_stop_latched:
                output = stop_controller.request_stop("DASHBOARD_STOP_REQUEST_LATCHED")

            throttle_state = throttle.update(output.target_speed_mps, dt_s,
                                             armed=(vcu_connected and vcu.telemetry is not None and
                                                    vcu.telemetry.state_name == "ARMED"),
                                             stop_requested=output.decision == 0)
            bumper_gap_m = (obstacle.nearest_forward_m - cfg["planning"]["stop_gap_from_bumper_m"]
                            if obstacle and obstacle.nearest_forward_m is not None else None)

            status = {
                "t_s": round(now - t_start, 2),
                "frame_count": frame_count,
                "frame_id": frame_id,
                "camera_ok": camera_ok,
                "camera_error": None if camera_ok else camera_error,
                "detector_error": detector_probe_error,
                "perception_enabled": perception_enabled,
                "detections": detections,
                "pipeline": pipeline,
                "centerline_xy": corridor.centerline_xy if corridor else [],
                "left_cone_ids": list(corridor.left_cone_ids) if corridor else [],
                "right_cone_ids": list(corridor.right_cone_ids) if corridor else [],
                "operator_stop_latched": operator_stop_latched,
                "stop_controller_state": output.state,
                "decision": output.decision,
                "reason": output.reason,
                "corridor_status": corridor_status,
                "safety_state": safety_state,
                "vehicle_authorized_by_perception": vehicle_authorized,
                "navigation_preview_enabled": navigation_preview_enabled,
                "obstacle_blocked": inputs.obstacle_blocked,
                "obstacle_forward_m": obstacle.nearest_forward_m if obstacle else None,
                "obstacle_clearance_required_m": obstacle.required_clearance_m if obstacle else None,
                "obstacle_clearance_qualified": inputs.obstacle_clearance_qualified,
                "bumper_gap_estimate_m": bumper_gap_m,
                "steering_angle_rad": steering.steering_angle_rad if steering and steering.valid else None,
                "target_speed_mps": output.target_speed_mps,
                "dac_mv_preview": throttle_state.dac_mv,
                "motor_output_enabled": cfg["throttle"]["motor_output_enabled"],
                "vcu_connected": vcu_connected,
                "vcu_board": "Raspberry Pi Pico (RP2040)",
                "vcu_hello": vcu.last_hello if vcu_connected else None,
                "vcu_identity_verified": vcu.identity_verified if vcu_connected else False,
                "vcu_monitor_error": vcu.connect_error if vcu is not None else None,
                "vcu_command_transmitted": False,
                "stop_acknowledged": False,
                "vcu_link_fresh": vcu_link_fresh,
                "vcu_telemetry": None if not vcu_connected or vcu.telemetry is None else {
                    "state": vcu.telemetry.state_name, "fault": vcu.telemetry.fault_names(),
                    "en": vcu.telemetry.en, "brk": vcu.telemetry.brk, "dac_mv": vcu.telemetry.dac_mv},
                "birds_eye_cones": birds_eye,
                "birds_eye_qualification": "DIAGNOSTIC_UNVERIFIED" if birds_eye else "NO_CANDIDATES",
            }
            state.update(status, jpeg=jpeg)
            csv_writer.writerow([status["t_s"], output.state, output.decision,
                                 status["obstacle_forward_m"], status["bumper_gap_estimate_m"],
                                 output.target_speed_mps, throttle_state.dac_mv,
                                 status["steering_angle_rad"], vcu_connected, vcu_link_fresh, output.reason])
            csv_fh.flush()
            frame_count += 1
    except KeyboardInterrupt:
        print("\nstopping (Ctrl-C)")
    finally:
        camera.stop()
        if vcu is not None:
            vcu.stop()
        csv_fh.close()
        server.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
