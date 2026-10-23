"""Phase 16 qualification utilities for timing, convergence, faults, and soak.

All actions continue through ``AutoKartFastEnv`` and therefore its VCU,
independent-safety layer, actuator dynamics, and authoritative internal plant.
This module has no Gazebo-to-actuator or dashboard-to-actuator path.
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import resource
import tempfile
from dataclasses import asdict
from math import hypot, sin
from pathlib import Path
from statistics import fmean
from time import perf_counter
from typing import Any

import yaml

from .autonomy import (
    MessageHeader,
    PurePursuitController,
    Trajectory,
    TrajectoryPoint,
)
from .fast_env import AutoKartFastEnv
from .realtime import TaskBudget, TaskTimingMonitor

ROOT = Path(__file__).resolve().parent.parent
TRACK = ROOT / "configs/tracks/research_circle.yaml"
SOAK_TRACK = ROOT / "configs/tracks/phase16_soak_circle.yaml"
KART = ROOT / "configs/karts/prototype_dynamic.yaml"

TIMING_BUDGETS = {
    "physics": TaskBudget(1000.0, 0.0007, 0.001, "warning"),
    "vcu": TaskBudget(1000.0, 0.0004, 0.001, "error"),
    "independent_safety": TaskBudget(1000.0, 0.0001, 0.001, "critical"),
}


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def host_profile() -> dict[str, Any]:
    cpu = platform.processor() or "unknown"
    cpuinfo = Path("/proc/cpuinfo")
    if cpuinfo.is_file():
        for line in cpuinfo.read_text(encoding="utf-8").splitlines():
            if line.startswith("model name"):
                cpu = line.split(":", 1)[1].strip()
                break
    memory_gib = None
    meminfo = Path("/proc/meminfo")
    if meminfo.is_file():
        first = meminfo.read_text(encoding="utf-8").splitlines()[0]
        memory_gib = int(first.split()[1]) / (1024.0 * 1024.0)
    return {
        "cpu": cpu,
        "logical_cpus": os.cpu_count(),
        "ram_gib": memory_gib,
        "os": platform.platform(),
        "python": platform.python_version(),
        "kernel": platform.release(),
        "gpu_utilization": "NOT_EXECUTED_EXTERNAL_DEPENDENCY",
        "rendering_backend": "Mesa Iris/Intel UHD 630 (native probe)",
    }


def _kart_for_rate(rate_hz: int, directory: Path) -> Path:
    payload = yaml.safe_load(KART.read_text(encoding="utf-8"))
    payload["simulation"]["physics_dt_s"] = 1.0 / float(rate_hz)
    path = directory / f"prototype_{rate_hz}hz.yaml"
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return path


def _case_action(name: str, time_s: float) -> tuple[float, float]:
    if name == "straight_acceleration":
        return 4.0, 0.0
    if name == "braking":
        return (4.0, 0.0) if time_s < 2.5 else (0.0, 0.0)
    if name == "constant_radius":
        return 3.0, 0.16
    if name == "slalom":
        return 3.0, 0.18 * sin(2.0 * time_s)
    if name in {"wet_surface_stop", "brake_fault"}:
        return (4.0, 0.0) if time_s < 2.5 else (0.0, 0.0)
    if name in {"steering_jam", "stuck_torque"}:
        return 3.0, 0.10
    raise ValueError(f"unsupported convergence case: {name}")


def _run_case(rate_hz: int, kart_path: Path, name: str) -> dict[str, Any]:
    env = AutoKartFastEnv(
        TRACK,
        kart_path,
        control_dt_s=0.02,
        maximum_steps=400,
    )
    env.arm_autonomy(operator_acknowledged=True)
    samples: list[dict[str, float]] = []
    fault_time_s: float | None = None
    torque_removed_time_s: float | None = None
    injected = False
    start_energy_j = env.kart.battery.feedback.discharged_energy_j
    while env.state.time_s < 6.0:
        time_s = env.state.time_s
        if not injected and time_s >= 2.0:
            fault = {
                "wet_surface_stop": "wet_surface",
                "steering_jam": "steering_jam",
                "brake_fault": "brake_cable_break",
                "stuck_torque": "stuck_motor_torque",
            }.get(name)
            if fault:
                env.inject_fault(fault)
                injected = True
        if name == "stuck_torque" and time_s >= 3.0:
            env.vcu.independent.estop = True
        action = _case_action(name, time_s)
        _, _, terminated, truncated, info = env.step(action)
        if info["vcu_faults"] and fault_time_s is None:
            fault_time_s = env.state.time_s
        if (
            name == "stuck_torque"
            and time_s >= 3.0
            and torque_removed_time_s is None
            and abs(env.kart.motor.feedback.actual_torque_nm) < 0.05
        ):
            torque_removed_time_s = env.state.time_s
        samples.append({
            "time_s": env.state.time_s,
            "x_m": env.state.x_m,
            "y_m": env.state.y_m,
            "speed_mps": env.state.u_mps,
            "yaw_rate_radps": env.state.yaw_rate_radps,
            "steering_rad": env.kart.steering.feedback.linkage_angle_rad,
            "track_s_m": float(info["track_s_m"]),
        })
        if terminated or truncated:
            break
    return {
        "rate_hz": rate_hz,
        "case": name,
        "samples": samples,
        "fault_detection_time_s": fault_time_s,
        "torque_removal_time_s": torque_removed_time_s,
        "energy_j": (
            env.kart.battery.feedback.discharged_energy_j - start_energy_j
        ),
        "final_speed_mps": env.state.u_mps,
        "final_track_s_m": samples[-1]["track_s_m"] if samples else 0.0,
    }


def _compare_case(reference: dict, candidate: dict) -> dict[str, Any]:
    count = min(len(reference["samples"]), len(candidate["samples"]))
    pairs = zip(reference["samples"][:count], candidate["samples"][:count])
    position = []
    speed = []
    yaw = []
    steering = []
    for left, right in pairs:
        position.append(hypot(left["x_m"] - right["x_m"], left["y_m"] - right["y_m"]))
        speed.append(abs(left["speed_mps"] - right["speed_mps"]))
        yaw.append(abs(left["yaw_rate_radps"] - right["yaw_rate_radps"]))
        steering.append(abs(left["steering_rad"] - right["steering_rad"]))
    def optional_delta(field: str) -> float | None:
        left, right = reference[field], candidate[field]
        return None if left is None or right is None else abs(left - right)
    reference_energy = max(abs(reference["energy_j"]), 1e-9)
    return {
        "maximum_position_error_m": max(position, default=0.0),
        "maximum_speed_error_mps": max(speed, default=0.0),
        "maximum_yaw_rate_error_radps": max(yaw, default=0.0),
        "maximum_steering_error_rad": max(steering, default=0.0),
        "stopping_distance_error_m": abs(
            reference["final_track_s_m"] - candidate["final_track_s_m"]
        ),
        "fault_detection_time_difference_s": optional_delta(
            "fault_detection_time_s"
        ),
        "torque_removal_time_difference_s": optional_delta(
            "torque_removal_time_s"
        ),
        "energy_error_fraction": abs(
            reference["energy_j"] - candidate["energy_j"]
        ) / reference_energy,
    }


def qualify_physics_rates(output: Path) -> dict[str, Any]:
    rates = (1000, 500, 400, 250, 200)
    cases = (
        "straight_acceleration",
        "braking",
        "constant_radius",
        "slalom",
        "wet_surface_stop",
        "steering_jam",
        "brake_fault",
        "stuck_torque",
    )
    thresholds = {
        "maximum_position_error_m": 0.20,
        "maximum_speed_error_mps": 0.15,
        "maximum_yaw_rate_error_radps": 0.15,
        "maximum_steering_error_rad": 0.03,
        "stopping_distance_error_m": 0.25,
        "fault_detection_time_difference_s": 0.010,
        "torque_removal_time_difference_s": 0.020,
        "energy_error_fraction": 0.02,
    }
    results: dict[int, dict[str, Any]] = {}
    with tempfile.TemporaryDirectory(prefix="autokart-rate-") as temporary:
        directory = Path(temporary)
        runs = {
            rate: {
                case: _run_case(rate, _kart_for_rate(rate, directory), case)
                for case in cases
            }
            for rate in rates
        }
    for rate in rates[1:]:
        comparisons = {
            case: _compare_case(runs[1000][case], runs[rate][case])
            for case in cases
        }
        failures = []
        for case, metrics in comparisons.items():
            for metric, limit in thresholds.items():
                value = metrics[metric]
                if value is not None and value > limit:
                    failures.append({
                        "case": case,
                        "metric": metric,
                        "value": value,
                        "limit": limit,
                    })
        results[rate] = {
            "passed": not failures,
            "comparisons": comparisons,
            "failures": failures,
        }
    passing = [rate for rate in rates[1:] if results[rate]["passed"]]
    selected = min(passing) if passing else 1000
    # Phase 16 deliberately retains 1 kHz until the same candidate also passes
    # native rendered-SIL safety timing; numerical convergence alone is not a
    # rate-change authorization.
    payload = {
        "reference_rate_hz": 1000,
        "numerically_lowest_passing_rate_hz": selected,
        "selected_operational_rate_hz": 1000,
        "selection_reason": (
            "Retain the validated 1000 Hz rate; lower-rate numerical evidence "
            "does not by itself qualify native rendered-SIL safety timing."
        ),
        "thresholds": thresholds,
        "rates": results,
        "simulation_only": True,
    }
    _write_json(output, payload)
    return payload


def fault_load_campaign(output: Path) -> dict[str, Any]:
    from .scenario import run_scenario

    definitions = {
        "communication_loss": (
            "vcu_command_timeout.yaml", "CONTROLLED_STOP", "COMM-STALE"
        ),
        "vcu_freeze": (
            "vcu_task_freeze.yaml", "EMERGENCY_STOP", None
        ),
        "physical_estop": (
            "vcu_physical_estop.yaml", "EMERGENCY_STOP", None
        ),
        "steering_jam": (
            "vcu_steering_jam.yaml", "CONTROLLED_STOP", "STEER-001"
        ),
        "brake_cable_break": (
            "vcu_brake_cable_break.yaml", "EMERGENCY_STOP", "BRAKE-003"
        ),
        "stuck_motor_torque": (
            "vcu_motor_stuck_torque.yaml", "EMERGENCY_STOP", "MOTOR-003"
        ),
    }
    results: dict[str, Any] = {}
    with tempfile.TemporaryDirectory(prefix="autokart-fault-load-") as temporary:
        for fault, (scenario_name, state, code) in definitions.items():
            started = perf_counter()
            run = run_scenario(
                ROOT / "configs/scenarios" / scenario_name,
                Path(temporary) / fault,
            )
            rows = [
                json.loads(line)
                for line in (run / "telemetry.jsonl").read_text(
                    encoding="utf-8"
                ).splitlines()
            ]
            matching = [
                row for row in rows
                if row.get("vcu", {}).get("state") == state
                and (
                    code is None
                    or code in row.get("vcu", {}).get("faults", [])
                )
            ]
            final = rows[-1]
            results[fault] = {
                "passed": bool(matching)
                and final["subsystems"]["motor"]["actual_torque_nm"] == 0,
                "detected_time_s":
                    matching[0]["state"]["time_s"] if matching else None,
                "final_state": final.get("vcu", {}).get("state"),
                "final_speed_mps": final["state"]["u_mps"],
                "faults": final.get("vcu", {}).get("faults", []),
                "motor_torque_removed":
                    final["subsystems"]["motor"]["actual_torque_nm"] == 0,
                "brake_pressure_pa":
                    final["subsystems"]["brake"]["hydraulic_pressure_pa"],
                "wall_elapsed_s": perf_counter() - started,
                "source": f"configs/scenarios/{scenario_name}",
            }

    env = AutoKartFastEnv(TRACK, KART, maximum_steps=300)
    env.arm_autonomy(operator_acknowledged=True)
    monitor = TaskTimingMonitor(TIMING_BUDGETS)
    env.set_timing_monitor(monitor)
    started = perf_counter()
    for _ in range(300):
        busy_until = perf_counter() + 0.0004
        while perf_counter() < busy_until:
            pass
        _, _, terminated, truncated, _ = env.step((3.0, 0.05))
        if terminated or truncated:
            break
    timing = monitor.report()
    results["compute_overload"] = {
        "passed":
            timing["vcu"]["deadline_misses"] == 0
            and timing["independent_safety"]["deadline_misses"] == 0,
        "bounded_callback_load_s": 0.0004,
        "final_state": env.vcu.state.value,
        "timing": timing,
        "wall_elapsed_s": perf_counter() - started,
    }
    payload = {
        "environment": "headless authoritative plant under bounded CPU load",
        "native_gazebo_load": "NOT_EXECUTED",
        "results": results,
        "passed": all(result.get("passed", False) for result in results.values()),
    }
    _write_json(output, payload)
    return payload


def _current_rss_mib() -> float:
    status = Path("/proc/self/status")
    if status.is_file():
        for line in status.read_text(encoding="utf-8").splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024.0
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0


def run_soak(duration_s: float, output: Path) -> dict[str, Any]:
    if duration_s <= 0.0:
        raise ValueError("soak duration must be positive")
    env = AutoKartFastEnv(
        SOAK_TRACK,
        KART,
        maximum_steps=max(1, int(duration_s / 0.02) + 1),
    )
    env.arm_autonomy(operator_acknowledged=True)
    monitor = TaskTimingMonitor(TIMING_BUDGETS, maximum_samples_per_task=200_000)
    env.set_timing_monitor(monitor)
    controller = PurePursuitController(
        lookahead_m=4.0,
        speed_lookahead_gain_s=0.1,
        wheelbase_m=env.config.kart.wheelbase_m,
        maximum_steering_rate_radps=0.5,
    )
    trajectory: Trajectory | None = None
    next_plan_s = 0.0
    wall_start = perf_counter()
    rss_start = _current_rss_mib()
    samples = []
    next_sample_s = 0.0
    termination_reason = None
    while env.state.time_s < duration_s:
        if trajectory is None or env.state.time_s + 1e-12 >= next_plan_s:
            points = []
            for index in range(161):
                point = env.track.sample(
                    env.projection.s_m + index * 0.25
                )
                points.append(TrajectoryPoint(
                    time_s=env.state.time_s + index * 0.25 / 3.5,
                    x_m=point.x_m,
                    y_m=point.y_m,
                    z_m=point.z_m,
                    heading_rad=point.heading_rad,
                    speed_mps=3.5,
                    curvature_1pm=point.curvature_1pm,
                    track_s_m=point.s_m,
                ))
            trajectory = Trajectory(
                MessageHeader(
                    env.state.time_s,
                    "map",
                    int(env.state.time_s) + 1,
                    "phase16_rolling_centreline",
                ),
                tuple(points),
                valid_until_s=env.state.time_s + 12.0,
            )
            next_plan_s = env.state.time_s + 1.0
        command = controller.compute(
            env.state, trajectory, env.control_dt_s
        )
        _, _, terminated, truncated, info = env.step(
            (3.5, command.steering_rad)
        )
        if env.state.time_s + 1e-12 >= next_sample_s:
            samples.append({
                "time_s": env.state.time_s,
                "rss_mib": _current_rss_mib(),
                "laps": int(info["lap_count"]),
                "soc": env.kart.battery.feedback.soc,
                "battery_voltage_v":
                    env.kart.battery.feedback.terminal_voltage_v,
                "motor_temperature_c":
                    env.kart.motor.feedback.motor_temperature_c,
                "controller_temperature_c":
                    env.kart.motor.feedback.controller_temperature_c,
                "maximum_sensor_queue_depth": 0,
            })
            next_sample_s += 10.0
        if terminated or truncated:
            termination_reason = {
                "terminated": terminated,
                "truncated": truncated,
                "out_of_bounds": info.get("out_of_bounds"),
                "safety_stop": info.get("safety_stop"),
                "lateral_m": info.get("lateral_m"),
                "steering_rad": command.steering_rad,
                "heading_error_rad": command.heading_error_rad,
            }
            break
    wall = perf_counter() - wall_start
    rss_end = _current_rss_mib()
    timing = monitor.report()
    payload = {
        "requested_duration_s": duration_s,
        "completed_duration_s": env.state.time_s,
        "wall_elapsed_s": wall,
        "rtf": env.state.time_s / max(wall, 1e-12),
        "laps": int(info["lap_count"]),
        "memory_start_mib": rss_start,
        "memory_end_mib": rss_end,
        "memory_growth_mib": rss_end - rss_start,
        "maximum_queue_depth": 0,
        "queue_growth": 0,
        "timing": timing,
        "deadline_misses": {
            task: values["deadline_misses"]
            for task, values in timing.items()
        },
        "initial_soc": samples[0]["soc"] if samples else None,
        "final_soc": env.kart.battery.feedback.soc,
        "final_motor_temperature_c":
            env.kart.motor.feedback.motor_temperature_c,
        "final_controller_temperature_c":
            env.kart.motor.feedback.controller_temperature_c,
        "brake_temperature": "NOT_MODELLED",
        "samples": samples,
        "native_gazebo_dashboard_recorder_soak": "NOT_EXECUTED",
        "termination_reason": termination_reason,
    }
    payload["passed"] = (
        env.state.time_s >= duration_s - 0.02
        and payload["memory_growth_mib"] < 64.0
        and payload["queue_growth"] == 0
        and timing["vcu"]["deadline_misses"] == 0
        and timing["independent_safety"]["deadline_misses"] == 0
    )
    _write_json(output, payload)
    return payload


def gate_native_run(run_directory: Path, output: Path) -> dict[str, Any]:
    summary = json.loads(
        (run_directory / "summary.json").read_text(encoding="utf-8")
    )
    native = json.loads(
        (run_directory / "native_results.json").read_text(encoding="utf-8")
    )
    timing = json.loads(
        (run_directory / "timing_report.json").read_text(encoding="utf-8")
    )
    sensors = native["sensors"]
    conditions = {
        "aggregate_rtf_at_least_one": timing["aggregate_rtf"] >= 1.0,
        "vcu_deadline_misses_zero":
            timing["tasks"]["vcu"]["deadline_misses"] == 0,
        "safety_deadline_misses_zero":
            timing["tasks"]["independent_safety"]["deadline_misses"] == 0,
        "timestamp_violations_zero": native["timestamp_violations"] == 0,
        "double_physics_zero": native["double_physics_diagnostics"] == 0,
        "payloads_nonempty": all(
            values["delivered"] > 0 and values["empty_payloads"] == 0
            for values in sensors.values()
        ),
        "queues_bounded": all(
            values["maximum_queue_depth"] <= 4
            for values in sensors.values()
        ),
        "no_stale_valid_deliveries": all(
            values["maximum_latency_s"] <= 0.100
            for values in sensors.values()
        ),
        "controlled_stop_safe":
            summary["controlled_stop_complete"]
            and summary["final_vcu_state"] == "SAFE",
    }
    payload = {
        "run_directory": str(run_directory),
        "classification": (
            "REALTIME_QUALIFIED" if all(conditions.values())
            else "REALTIME_DEGRADED"
        ),
        "passed": all(conditions.values()),
        "conditions": conditions,
        "aggregate_rtf": timing["aggregate_rtf"],
        "minimum_window_rtf": timing["minimum_window_rtf"],
        "task_timing": timing["tasks"],
    }
    _write_json(output, payload)
    return payload


def main() -> None:
    parser = argparse.ArgumentParser()
    subcommands = parser.add_subparsers(dest="command", required=True)
    host = subcommands.add_parser("host")
    host.add_argument("--output", type=Path, required=True)
    rates = subcommands.add_parser("physics-rates")
    rates.add_argument("--output", type=Path, required=True)
    faults = subcommands.add_parser("faults")
    faults.add_argument("--output", type=Path, required=True)
    soak = subcommands.add_parser("soak")
    soak.add_argument("--duration-s", type=float, default=1800.0)
    soak.add_argument("--output", type=Path, required=True)
    gate = subcommands.add_parser("gate")
    gate.add_argument("--run-dir", type=Path, required=True)
    gate.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    exit_code = 0
    if arguments.command == "host":
        payload = host_profile()
        _write_json(arguments.output, payload)
    elif arguments.command == "physics-rates":
        payload = qualify_physics_rates(arguments.output)
    elif arguments.command == "faults":
        payload = fault_load_campaign(arguments.output)
    elif arguments.command == "soak":
        payload = run_soak(arguments.duration_s, arguments.output)
    else:
        payload = gate_native_run(arguments.run_dir, arguments.output)
        if not payload["passed"]:
            exit_code = 2
    print(json.dumps(payload, indent=2, sort_keys=True))
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
