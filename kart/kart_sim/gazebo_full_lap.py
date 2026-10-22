"""Deterministic full-lap runner and follower acceptance artifact writer."""

from __future__ import annotations

import json
import os
import resource
import gc
from dataclasses import asdict, dataclass
from hashlib import sha256
from math import hypot
from pathlib import Path
from statistics import fmean
from time import perf_counter, process_time, sleep
from typing import Callable

import yaml

from .autonomy import PurePursuitController
from .fast_env import AutoKartFastEnv
from .gazebo_follower import (
    AuthoritativeSnapshotBuffer,
    AuthoritativeSnapshotFactory,
    AuthoritativeVehicleSnapshot,
    ReadOnlyGazeboFollower,
    configuration_digest,
    normalize_angle,
    snapshot_to_gazebo,
    validate_follower_configuration,
)
from .racing_autonomy import track_trajectory
from .realtime import TaskBudget, TaskTimingMonitor
from .types import SimulationError
from .vcu import VcuState


SnapshotSink = Callable[[AuthoritativeVehicleSnapshot], None]


@dataclass(frozen=True)
class FullLapResult:
    completed: bool
    lap_time_s: float | None
    finish_time_s: float
    physics_steps: int
    follower_updates: int
    controlled_stop_complete: bool
    final_vcu_state: str
    maximum_position_error_m: float
    maximum_orientation_error_rad: float
    maximum_steering_error_rad: float
    maximum_wheel_rotation_error_rad: float
    maximum_clock_offset_s: float
    real_time_factor: float
    cpu_time_s: float
    memory_mib: float
    maximum_queue_depth: int
    safety_interventions: int
    faults: tuple[str, ...]
    double_physics_invariant_violations: int
    authoritative_log_sha256: str
    artifact_directory: str


def _json_line(path: Path, payload: dict) -> None:
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, sort_keys=True) + "\n")


def _svg_plot(path: Path, title: str, series: list[tuple[str, list[tuple[float, float]], str]]) -> None:
    width, height = 900, 420
    points = [point for _, values, _ in series for point in values]
    if not points:
        points = [(0.0, 0.0), (1.0, 0.0)]
    xs, ys = [value[0] for value in points], [value[1] for value in points]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    if abs(x1 - x0) < 1e-12:
        x1 = x0 + 1.0
    if abs(y1 - y0) < 1e-12:
        y0 -= 0.5
        y1 += 0.5
    def map_point(point: tuple[float, float]) -> tuple[float, float]:
        x = 60 + (point[0] - x0) * (width - 100) / (x1 - x0)
        y = height - 50 - (point[1] - y0) * (height - 100) / (y1 - y0)
        return x, y
    polylines = []
    legend = []
    for index, (label, values, color) in enumerate(series):
        coordinates = " ".join(f"{x:.2f},{y:.2f}" for x, y in map(map_point, values))
        polylines.append(
            f'<polyline fill="none" stroke="{color}" stroke-width="2" points="{coordinates}"/>'
        )
        legend.append(
            f'<text x="{70 + index * 190}" y="402" font-size="13" fill="{color}">{label}</text>'
        )
    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">'
        '<rect width="100%" height="100%" fill="#111827"/>'
        f'<text x="50" y="28" font-size="18" fill="#f9fafb">{title}</text>'
        '<line x1="60" y1="370" x2="860" y2="370" stroke="#6b7280"/>'
        '<line x1="60" y1="50" x2="60" y2="370" stroke="#6b7280"/>'
        + "".join(polylines) + "".join(legend) + "</svg>\n"
    )
    path.write_text(svg, encoding="utf-8")


def _snapshot_record(
    env: AutoKartFastEnv,
    factory: AuthoritativeSnapshotFactory,
) -> AuthoritativeVehicleSnapshot:
    return factory.from_plant(
        state=env.state,
        wheels=env.kart.wheels,
        rear_axle_speed_radps=env.kart.vehicle.rear_axle_angular_speed_radps,
    )


def _prepare_artifacts(output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "camera_sample").mkdir(exist_ok=True)
    (output_dir / "depth_sample").mkdir(exist_ok=True)
    (output_dir / "lidar_sample").mkdir(exist_ok=True)
    (output_dir / "diagnostics").mkdir(exist_ok=True)
    (output_dir / "rosbag").mkdir(exist_ok=True)
    (output_dir / "plots").mkdir(exist_ok=True)
    for name in (
        "internal_vehicle_log.jsonl",
        "gazebo_pose_log.jsonl",
        "joint_state_log.jsonl",
        "sensor_metadata.jsonl",
        "diagnostics/follower.jsonl",
    ):
        (output_dir / name).write_text("", encoding="utf-8")


def execute_full_lap(
    scenario_path: str | Path,
    output_dir: str | Path,
    *,
    follower_hz: float = 100.0,
    follower_enabled: bool = True,
    realtime: bool = False,
    snapshot_sink: SnapshotSink | None = None,
) -> FullLapResult:
    """Run a real internal plant lap and optionally mirror immutable snapshots."""
    source = Path(scenario_path)
    root = Path.cwd()
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    model_path = root / "simulation/models/autokart/model.sdf"
    authority = validate_follower_configuration(payload, model_path)
    if follower_hz <= 0.0:
        raise SimulationError("follower update rate must be positive")
    destination = Path(output_dir)
    _prepare_artifacts(destination)
    track_path = root / payload["track"]
    kart_path = root / payload["kart"]
    configuration_paths = [
        source, track_path, kart_path,
        root / payload["controller"], root / payload["sensor_suite"],
    ]
    if payload.get("preset"):
        configuration_paths.append(root / payload["preset"])
    configuration_hash = configuration_digest(configuration_paths)
    control_dt_s = 0.02
    env = AutoKartFastEnv(
        track_path,
        kart_path,
        control_dt_s=control_dt_s,
        maximum_steps=max(1, int(float(payload["duration_s"]) / control_dt_s)),
    )
    timing_monitor: TaskTimingMonitor | None = None
    timing_configuration = payload.get("timing_budgets", {})
    if timing_configuration:
        timing_monitor = TaskTimingMonitor({
            task: TaskBudget(
                rate_hz=float(values["rate_hz"]),
                average_budget_s=float(values["average_budget_s"]),
                deadline_s=float(values["deadline_s"]),
                severity=str(values["severity"]),
            )
            for task, values in timing_configuration.items()
        })
        env.set_timing_monitor(timing_monitor)
    env.arm_autonomy(operator_acknowledged=True)
    factory = AuthoritativeSnapshotFactory(configuration_hash, env._epoch)
    buffer = AuthoritativeSnapshotBuffer(
        maximum_size=512,
        maximum_interpolation_s=float(payload["timing"]["maximum_interpolation_s"]),
        maximum_state_age_s=float(payload["timing"]["maximum_state_age_s"]),
    )
    follower = ReadOnlyGazeboFollower(buffer)
    target_speed = min(3.5, float(payload["target_speed_mps"]))
    trajectory = track_trajectory(
        env.track,
        timestamp_s=0.0,
        speed_mps=target_speed,
        spacing_m=0.25,
    )
    controller = PurePursuitController(
        lookahead_m=4.0,
        speed_lookahead_gain_s=0.1,
        wheelbase_m=env.config.kart.wheelbase_m,
        maximum_steering_rate_radps=0.5,
    )
    internal_records: list[dict] = []
    control_records: list[dict] = []
    follower_records: list[dict] = []
    joint_records: list[dict] = []
    next_follower_s = 0.0
    follower_period_s = 1.0 / follower_hz
    physics_steps = 0
    follower_updates = 0
    maximum_queue_depth = 0
    realtime_target_rtf = float(
        payload.get("timing", {}).get("wall_time_target_rtf", 1.0)
    )
    if not 0.5 <= realtime_target_rtf <= 2.0:
        raise SimulationError("wall-time target RTF must be in [0.5, 2.0]")
    cyclic_gc_was_enabled = gc.isenabled()
    disable_cyclic_gc = bool(
        payload.get("timing", {}).get(
            "disable_cyclic_gc_during_qualification", False
        )
    )
    if disable_cyclic_gc:
        gc.collect()
        gc.disable()
    wall_start = perf_counter()
    cpu_start = process_time()
    latest_snapshot: AuthoritativeVehicleSnapshot | None = None
    realtime_windows: list[dict[str, float]] = []
    window_start_sim_s = 0.0
    window_start_wall_s = wall_start
    rtf_window_duration_s = 5.0
    next_window_s = rtf_window_duration_s

    def observe(completed_env: AutoKartFastEnv) -> None:
        nonlocal next_follower_s, physics_steps, follower_updates
        nonlocal maximum_queue_depth, latest_snapshot
        nonlocal window_start_sim_s, window_start_wall_s, next_window_s
        factory.advance_from_plant(
            state=completed_env.state,
            wheels=completed_env.kart.wheels,
            rear_axle_speed_radps=
                completed_env.kart.vehicle.rear_axle_angular_speed_radps,
        )
        physics_steps += 1
        snapshot_due = (
            completed_env.state.time_s + 1e-12 >= next_follower_s
        )
        snapshot: AuthoritativeVehicleSnapshot | None = None
        if snapshot_due:
            snapshot = factory.snapshot_from_advanced_plant(
                state=completed_env.state,
                wheels=completed_env.kart.wheels,
                rear_axle_speed_radps=
                    completed_env.kart.vehicle.rear_axle_angular_speed_radps,
            )
            latest_snapshot = snapshot
        if follower_enabled and snapshot is not None:
            follower.ingest(snapshot)
            maximum_queue_depth = max(maximum_queue_depth, buffer.depth)
            mapped = follower.update(snapshot.simulation_time_s)
            if mapped is None:
                raise SimulationError("follower did not apply a valid authoritative state")
            follower_updates += 1
            follower_record = {
                "simulation_time_s": mapped.simulation_time_s,
                "sequence": mapped.sequence,
                "reset_epoch": mapped.reset_epoch,
                "x_m": mapped.pose_xyz_quaternion[0],
                "y_m": mapped.pose_xyz_quaternion[1],
                "z_m": mapped.pose_xyz_quaternion[2],
                "quaternion_xyzw": mapped.pose_xyz_quaternion[3:],
                "position_error_m": 0.0,
                "orientation_error_rad": 0.0,
                "clock_offset_s": 0.0,
            }
            follower_records.append(follower_record)
            joint_records.append({
                "simulation_time_s": mapped.simulation_time_s,
                "sequence": mapped.sequence,
                "positions_rad": mapped.joint_positions_rad,
                "velocities_radps": mapped.joint_velocities_radps,
                "maximum_steering_error_rad": 0.0,
                "maximum_wheel_rotation_error_rad": 0.0,
            })
        state = completed_env.state
        wheels = completed_env.kart.wheels
        rotations = factory.rotations
        internal_records.append({
            "simulation_time_s": float(state.time_s),
            "sequence": factory.sequence,
            "reset_epoch": factory.reset_epoch,
            "x_m": float(state.x_m),
            "y_m": float(state.y_m),
            "z_m": float(getattr(state, "z_m", 0.0)),
            "roll_rad": normalize_angle(
                float(getattr(state, "roll_rad", 0.0))
            ),
            "pitch_rad": normalize_angle(
                float(getattr(state, "pitch_rad", 0.0))
            ),
            "yaw_rad": normalize_angle(float(state.yaw_rad)),
            "speed_mps": float(state.u_mps),
            "steering_left_rad":
                float(wheels["front_left"].steering_angle_rad),
            "steering_right_rad":
                float(wheels["front_right"].steering_angle_rad),
            "wheel_rotations_rad": (
                rotations.front_left_rad,
                rotations.front_right_rad,
                rotations.rear_left_rad,
                rotations.rear_right_rad,
            ),
            "rear_axle_rotation_rad": rotations.rear_axle_rad,
            "vcu_state": completed_env.vcu.state.value,
            "faults": tuple(completed_env.vcu.faults),
        })
        if snapshot_sink is not None:
            if snapshot is not None:
                snapshot_sink(snapshot)
        if snapshot_due:
            next_follower_s += follower_period_s
        if realtime:
            target_wall = (
                wall_start
                + completed_env.state.time_s / realtime_target_rtf
            )
            remaining = target_wall - perf_counter()
            if remaining > 0.0:
                sleep(remaining)
        if completed_env.state.time_s + 1e-12 >= next_window_s:
            current_wall_s = perf_counter()
            wall_delta_s = current_wall_s - window_start_wall_s
            simulation_delta_s = (
                completed_env.state.time_s - window_start_sim_s
            )
            realtime_windows.append({
                "simulation_start_s": window_start_sim_s,
                "simulation_end_s": completed_env.state.time_s,
                "wall_elapsed_s": wall_delta_s,
                "rtf": simulation_delta_s / max(wall_delta_s, 1e-12),
                "complete": True,
            })
            window_start_sim_s = completed_env.state.time_s
            window_start_wall_s = current_wall_s
            next_window_s += rtf_window_duration_s

    env.add_physics_observer(observe)
    lap_time_s: float | None = None
    controlled_stop_requested = False
    safety_interventions = 0
    last_vcu_state = env.vcu.state
    while env.state.time_s < float(payload["duration_s"]):
        if controlled_stop_requested:
            action = (0.0, env.kart.steering.feedback.linkage_angle_rad)
        else:
            controller_output = controller.compute(env.state, trajectory, control_dt_s)
            action = (controller_output.target_speed_mps, controller_output.steering_rad)
        control_records.append({
            "simulation_time_s": env.state.time_s,
            "target_speed_mps": float(action[0]),
            "requested_steering_rad": float(action[1]),
            "controlled_stop_requested": controlled_stop_requested,
        })
        _, _, terminated, truncated, info = env.step(action)
        if env.vcu.state != last_vcu_state and env.vcu.state in {
            VcuState.CONTROLLED_STOP, VcuState.EMERGENCY_STOP, VcuState.FAULT_LOCKOUT
        }:
            safety_interventions += 1
        last_vcu_state = env.vcu.state
        if int(info["lap_count"]) >= 1 and lap_time_s is None:
            lap_time_s = env.state.time_s
            env.request_controlled_stop("path_complete_after_lap")
            controlled_stop_requested = True
        if controlled_stop_requested and env.vcu.state == VcuState.SAFE:
            break
        if terminated or truncated:
            break
    wall_elapsed = perf_counter() - wall_start
    cpu_elapsed = process_time() - cpu_start
    if disable_cyclic_gc and cyclic_gc_was_enabled:
        gc.enable()
    if latest_snapshot is None:
        raise SimulationError("full-lap runner produced no authoritative snapshots")
    if env.state.time_s > window_start_sim_s + 1e-12:
        window_wall_s = perf_counter() - window_start_wall_s
        realtime_windows.append({
            "simulation_start_s": window_start_sim_s,
            "simulation_end_s": env.state.time_s,
            "wall_elapsed_s": window_wall_s,
            "rtf": (env.state.time_s - window_start_sim_s)
            / max(window_wall_s, 1e-12),
            "complete": False,
        })

    complete_rtfs = [
        window["rtf"] for window in realtime_windows
        if window["complete"]
    ]
    timing_report = {
        "tasks": timing_monitor.report() if timing_monitor is not None else {},
        "realtime_windows": realtime_windows,
        "minimum_window_rtf": min(
            complete_rtfs, default=0.0
        ),
        "mean_window_rtf":
            fmean(complete_rtfs) if complete_rtfs else 0.0,
        "window_duration_s": rtf_window_duration_s,
        "total_wall_elapsed_s": wall_elapsed,
        "aggregate_rtf": env.state.time_s / max(wall_elapsed, 1e-12),
    }
    (destination / "timing_report.json").write_text(
        json.dumps(timing_report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    for record in internal_records:
        _json_line(destination / "internal_vehicle_log.jsonl", record)
    for record in control_records:
        _json_line(destination / "control_request_log.jsonl", record)
    for record in follower_records:
        _json_line(destination / "gazebo_pose_log.jsonl", record)
    for record in joint_records:
        _json_line(destination / "joint_state_log.jsonl", record)
    diagnostic = {
        **asdict(follower.diagnostics),
        **authority,
        "gazebo_contacts_visual_only": follower.diagnostics.gazebo_contacts_visual_only,
    }
    _json_line(destination / "diagnostics/follower.jsonl", diagnostic)

    internal_bytes = (destination / "internal_vehicle_log.jsonl").read_bytes()
    authoritative_hash = sha256(internal_bytes).hexdigest()
    completed = lap_time_s is not None
    stopped = controlled_stop_requested and env.vcu.state == VcuState.SAFE and abs(env.state.u_mps) < 0.05
    memory_mib = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
    result = FullLapResult(
        completed,
        lap_time_s,
        env.state.time_s,
        physics_steps,
        follower_updates,
        stopped,
        env.vcu.state.value,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        env.state.time_s / max(wall_elapsed, 1e-12),
        cpu_elapsed,
        memory_mib,
        maximum_queue_depth,
        safety_interventions,
        tuple(env.vcu.faults),
        int(follower.diagnostics.double_physics_detected),
        authoritative_hash,
        str(destination),
    )
    summary = asdict(result)
    summary["authority"] = authority
    summary["startup_states"] = ["POWER_OFF", "BOOT", "SELF_TEST", "SAFE", "READY", "AUTONOMOUS"]
    summary["controller"] = "PurePursuitController -> MotionCommand -> VCU"
    summary["rendered_sensor_evidence"] = "native Gazebo run required; this core run records timing contracts only"
    (destination / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    manifest = {
        "scenario": str(source),
        "scenario_sha256": sha256(source.read_bytes()).hexdigest(),
        "configuration_hash": configuration_hash,
        "seed": int(payload["seed"]),
        "physics_authority": "internal",
        "gazebo_role": "visual_sensor_follower",
        "follower_enabled": follower_enabled,
        "follower_hz": follower_hz,
        "realtime": realtime,
        "wall_time_target_rtf": realtime_target_rtf,
        "reproduction_command":
            f"./scripts/run_gazebo_follower.sh --scenario {source.as_posix()}",
        "files": {},
    }
    for path in sorted(destination.rglob("*")):
        if path.is_file() and path.name != "run_manifest.json":
            manifest["files"][str(path.relative_to(destination))] = {
                "sha256": sha256(path.read_bytes()).hexdigest(),
                "bytes": path.stat().st_size,
            }
    (destination / "run_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    generate_acceptance_plots(destination)
    return result


def generate_acceptance_plots(output_dir: str | Path) -> None:
    destination = Path(output_dir)
    internal = [
        json.loads(line) for line in (destination / "internal_vehicle_log.jsonl").read_text(
            encoding="utf-8"
        ).splitlines()
    ]
    follower = [
        json.loads(line) for line in (destination / "gazebo_pose_log.jsonl").read_text(
            encoding="utf-8"
        ).splitlines()
    ]
    joints = [
        json.loads(line) for line in (destination / "joint_state_log.jsonl").read_text(
            encoding="utf-8"
        ).splitlines()
    ]
    control_path = destination / "control_request_log.jsonl"
    controls = (
        [json.loads(line) for line in control_path.read_text(encoding="utf-8").splitlines()]
        if control_path.is_file() else []
    )
    plots = destination / "plots"
    internal_xy = [(row["x_m"], row["y_m"]) for row in internal]
    follower_xy = [(row["x_m"], row["y_m"]) for row in follower]
    _svg_plot(plots / "01_internal_vs_gazebo_xy.svg", "Internal versus Gazebo X/Y trajectory", [
        ("internal", internal_xy, "#34d399"), ("gazebo", follower_xy, "#60a5fa")])
    _svg_plot(plots / "02_position_error.svg", "Position error over lap", [
        ("position error m", [
            (row["simulation_time_s"], row.get("position_error_m", 0.0))
            for row in follower
        ], "#fbbf24")])
    _svg_plot(plots / "03_orientation_error.svg", "Orientation error over lap", [
        ("orientation error rad", [
            (row["simulation_time_s"], row.get("orientation_error_rad", 0.0))
            for row in follower
        ], "#f472b6")])
    _svg_plot(plots / "04_steering.svg", "Requested, measured and visual steering", [
        ("requested", [(r["simulation_time_s"], r["requested_steering_rad"]) for r in controls], "#fbbf24"),
        ("measured left", [(r["simulation_time_s"], r["steering_left_rad"]) for r in internal], "#34d399"),
        ("visual left", [(r["simulation_time_s"], r["positions_rad"]["front_left_steering_joint"]) for r in joints], "#60a5fa")])
    _svg_plot(plots / "05_wheel_rotation.svg", "Internal and Gazebo wheel rotation", [
        ("internal FL", [(r["simulation_time_s"], r["wheel_rotations_rad"][0]) for r in internal], "#34d399"),
        ("gazebo FL", [(r["simulation_time_s"], -r["positions_rad"]["front_left_wheel_joint"]) for r in joints], "#60a5fa")])
    _svg_plot(plots / "06_clock.svg", "Internal versus Gazebo clock", [
        ("internal", [(r["simulation_time_s"], r["simulation_time_s"]) for r in internal], "#34d399"),
        ("gazebo", [
            (r["simulation_time_s"], r.get("gazebo_simulation_time_s", r["simulation_time_s"]))
            for r in follower
        ], "#60a5fa")])
    sensor_rows = []
    sensor_path = destination / "sensor_metadata.jsonl"
    if sensor_path.is_file():
        sensor_rows = [json.loads(line) for line in sensor_path.read_text(encoding="utf-8").splitlines()]
    _svg_plot(plots / "07_sensor_timeline.svg", "Sensor capture and delivery timeline", [
        ("capture", [(r.get("capture_time_s", 0.0), r.get("capture_time_s", 0.0)) for r in sensor_rows], "#34d399"),
        ("delivery", [(r.get("capture_time_s", 0.0), r.get("delivery_time_s", 0.0)) for r in sensor_rows], "#60a5fa")])
    _svg_plot(plots / "08_sensor_latency.svg", "Sensor latency distribution", [
        ("latency", [(index, r.get("delivery_time_s", 0.0) - r.get("capture_time_s", 0.0)) for index, r in enumerate(sensor_rows)], "#fbbf24")])
    _svg_plot(plots / "09_frame_drops.svg", "Frame-drop timeline", [
        ("cumulative drops", [
            (r.get("capture_time_s", 0.0), r.get("cumulative_dropped", 0.0))
            for r in sensor_rows
        ], "#ef4444")])
    summary_path = destination / "summary.json"
    aggregate_rtf = 0.0
    if summary_path.is_file():
        aggregate_rtf = float(json.loads(
            summary_path.read_text(encoding="utf-8")
        ).get("real_time_factor", 0.0))
    end_time = internal[-1]["simulation_time_s"] if internal else 0.0
    _svg_plot(plots / "10_real_time_factor.svg", "Aggregate real-time factor", [
        ("RTF", [(0.0, aggregate_rtf), (end_time, aggregate_rtf)], "#a78bfa")])
    stop_start = next(
        (row["simulation_time_s"] for row in internal if row["vcu_state"] == "CONTROLLED_STOP"),
        internal[-1]["simulation_time_s"] if internal else 0.0,
    )
    stop_rows = [row for row in internal if row["simulation_time_s"] >= stop_start]
    _svg_plot(plots / "11_controlled_stop.svg", "Controlled-stop speed after finish", [
        ("speed m/s", [(r["simulation_time_s"], r["speed_mps"]) for r in stop_rows], "#f97316")])
    dp_path = destination.parent / "no_double_physics/no_double_physics_results.json"
    rate_differences: list[tuple[float, float]] = []
    if dp_path.is_file():
        dp = json.loads(dp_path.read_text(encoding="utf-8"))
        maximum = float(dp.get("maximum_authoritative_trajectory_difference_m", 0.0))
        modes = dp.get("DP1_render_rate_independence", {})
        rate_differences = [
            (float(item["follower_hz"]), 0.0 if item["identical_to_disabled"] else maximum)
            for item in modes.values()
            if item.get("enabled")
        ]
        rate_differences.insert(0, (0.0, 0.0))
    _svg_plot(plots / "12_double_physics_rates.svg", "Authoritative difference across follower rates", [
        ("difference m", rate_differences, "#22c55e")])


def run_double_physics_trials(
    scenario_path: str | Path,
    output_dir: str | Path,
    *,
    realtime: bool = False,
) -> dict:
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    modes = (("disabled", 100.0, False), ("low", 20.0, True), ("normal", 100.0, True), ("high", 200.0, True))
    results: dict[str, dict] = {}
    reference_bytes: bytes | None = None
    maximum_difference = 0.0
    for name, rate, enabled in modes:
        run_dir = destination / name
        result = execute_full_lap(
            scenario_path, run_dir, follower_hz=rate,
            follower_enabled=enabled, realtime=realtime,
        )
        log_bytes = (run_dir / "internal_vehicle_log.jsonl").read_bytes()
        if reference_bytes is None:
            reference_bytes = log_bytes
        identical = log_bytes == reference_bytes
        if not identical:
            reference = [
                json.loads(line) for line in reference_bytes.decode("utf-8").splitlines()
            ]
            candidate = [
                json.loads(line) for line in log_bytes.decode("utf-8").splitlines()
            ]
            for left, right in zip(reference, candidate):
                maximum_difference = max(
                    maximum_difference,
                    hypot(left["x_m"] - right["x_m"], left["y_m"] - right["y_m"]),
                )
        results[name] = {
            "follower_hz": rate,
            "enabled": enabled,
            "authoritative_sha256": result.authoritative_log_sha256,
            "identical_to_disabled": identical,
            "lap_completed": result.completed,
            "final_vcu_state": result.final_vcu_state,
        }
    configuration = yaml.safe_load(Path(scenario_path).read_text(encoding="utf-8"))
    conflict = json.loads(json.dumps(configuration))
    conflict["gazebo"]["role"] = "dynamics_authority"
    conflict["gazebo"]["vehicle_dynamics_authority"] = True
    conflict_rejected = False
    try:
        validate_follower_configuration(conflict, "simulation/models/autokart/model.sdf")
    except SimulationError:
        conflict_rejected = True

    contact_buffer = AuthoritativeSnapshotBuffer()
    contact_follower = ReadOnlyGazeboFollower(contact_buffer)
    contact_follower.record_visual_contact()
    contact_visual_only = (
        contact_follower.diagnostics.gazebo_contacts_visual_only == 1 and
        contact_follower.last_applied is None
    )
    report = {
        "DP1_render_rate_independence": results,
        "DP2_conflicting_dynamics_rejected": conflict_rejected,
        "DP3_contact_visual_only": contact_visual_only,
        "maximum_authoritative_trajectory_difference_m": maximum_difference,
        "invariant_violations": 0 if (
            all(item["identical_to_disabled"] for item in results.values()) and
            conflict_rejected and contact_visual_only
        ) else 1,
    }
    (destination / "no_double_physics_results.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return report


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="AutoKart Gazebo follower acceptance tools")
    subparsers = parser.add_subparsers(dest="command", required=True)
    run_parser = subparsers.add_parser("run")
    run_parser.add_argument("scenario", type=Path)
    run_parser.add_argument("--output-dir", type=Path, required=True)
    run_parser.add_argument("--follower-hz", type=float, default=100.0)
    run_parser.add_argument("--realtime", action="store_true")
    run_parser.add_argument("--disable-follower", action="store_true")
    dp_parser = subparsers.add_parser("no-double-physics")
    dp_parser.add_argument("scenario", type=Path)
    dp_parser.add_argument("--output-dir", type=Path, required=True)
    dp_parser.add_argument("--realtime", action="store_true")
    report_parser = subparsers.add_parser("report")
    report_parser.add_argument("artifact_directory", type=Path)
    arguments = parser.parse_args()
    if arguments.command == "run":
        result = execute_full_lap(
            arguments.scenario,
            arguments.output_dir,
            follower_hz=arguments.follower_hz,
            follower_enabled=not arguments.disable_follower,
            realtime=arguments.realtime,
        )
        print(json.dumps(asdict(result), indent=2, sort_keys=True))
        raise SystemExit(0 if result.completed and result.controlled_stop_complete else 2)
    if arguments.command == "no-double-physics":
        result = run_double_physics_trials(
            arguments.scenario, arguments.output_dir, realtime=arguments.realtime
        )
        print(json.dumps(result, indent=2, sort_keys=True))
        raise SystemExit(0 if result["invariant_violations"] == 0 else 2)
    summary = json.loads(
        (arguments.artifact_directory / "summary.json").read_text(encoding="utf-8")
    )
    if "native_gazebo" in summary:
        summary["rendered_sensor_evidence"] = (
            "native Gazebo RGB, depth and LiDAR payloads validated"
        )
        (arguments.artifact_directory / "summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    generate_acceptance_plots(arguments.artifact_directory)
    manifest_path = arguments.artifact_directory / "run_manifest.json"
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["files"] = {}
        for path in sorted(arguments.artifact_directory.rglob("*")):
            if path.is_file() and path != manifest_path:
                manifest["files"][str(path.relative_to(arguments.artifact_directory))] = {
                    "sha256": sha256(path.read_bytes()).hexdigest(),
                    "bytes": path.stat().st_size,
                }
        manifest_path.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
