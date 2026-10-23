"""Repeatable local fidelity-tier performance measurements."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from importlib.util import find_spec
from pathlib import Path
from resource import RUSAGE_SELF, getrusage
from time import perf_counter, process_time
from typing import Callable

from .backends import detect_chrono
from .config import load_kart_config
from .fast_env import AutoKartFastEnv
from .scenario import run_scenario
from .types import PlanarKartState, VehicleCommand
from .vehicle import PlanarBicycleKart


@dataclass(frozen=True)
class TierPerformance:
    tier: int
    status: str
    workload: str
    simulated_duration_s: float | None
    wall_time_s: float | None
    cpu_time_s: float | None
    cpu_utilization_percent: float | None
    peak_rss_mib: float | None
    update_rate_hz: float | None
    real_time_factor: float | None
    gpu: str
    detail: str = ""


def _measure(tier: int, workload: str, simulated_duration_s: float, steps: int, callback: Callable[[], None]) -> TierPerformance:
    rss_before = getrusage(RUSAGE_SELF).ru_maxrss
    wall_start, cpu_start = perf_counter(), process_time()
    callback()
    wall = perf_counter() - wall_start
    cpu = process_time() - cpu_start
    rss = max(rss_before, getrusage(RUSAGE_SELF).ru_maxrss) / 1024.0
    return TierPerformance(
        tier,
        "EXECUTED",
        workload,
        simulated_duration_s,
        wall,
        cpu,
        100.0 * cpu / wall if wall else None,
        rss,
        steps / wall if wall else None,
        simulated_duration_s / wall if wall else None,
        "NOT_USED_HEADLESS",
    )


def measure_supported_tiers(root: str | Path, output_dir: str | Path) -> tuple[TierPerformance, ...]:
    workspace = Path(root)
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    planar_config = load_kart_config(workspace / "configs/karts/prototype.yaml")
    tier0_steps = 5000

    def tier0() -> None:
        kart = PlanarBicycleKart(planar_config, PlanarKartState())
        for _ in range(tier0_steps):
            kart.step(VehicleCommand(drive=0.2), planar_config.simulation.physics_dt_s)

    tier1_steps = 200

    def tier1() -> None:
        env = AutoKartFastEnv(
            workspace / "configs/tracks/cone_track.yaml",
            workspace / "configs/karts/prototype_dynamic.yaml",
            maximum_steps=tier1_steps,
            lidar_beams=108,
        )
        env.reset(seed=20260729, options={"arm": True, "operator_acknowledged": True})
        for _ in range(tier1_steps):
            _, _, terminated, truncated, _ = env.step((2.0, 0.0))
            if terminated or truncated:
                break

    scenario_path = workspace / "configs/scenarios/vcu_controlled_stop_nominal.yaml"
    import yaml

    scenario_payload = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
    tier2_duration = float(scenario_payload["duration_s"])
    tier2_steps = int(tier2_duration / 0.001)

    reports = [
        _measure(
            0,
            "planar unit plant, 5000 fixed steps",
            tier0_steps * planar_config.simulation.physics_dt_s,
            tier0_steps,
            tier0,
        ),
        _measure(1, "VCU-routed fast environment with 108-beam LiDAR", tier1_steps * 0.02, tier1_steps, tier1),
        _measure(
            2,
            "dynamic four-wheel controlled-stop engineering scenario",
            tier2_duration,
            tier2_steps,
            lambda: run_scenario(scenario_path, output / "tier2"),
        ),
    ]
    gazebo_available = Path("/usr/bin/gz").exists() or Path("/usr/bin/gazebo").exists()
    reports.append(TierPerformance(
        3,
        "NOT_EXECUTED_EXTERNAL_DEPENDENCY",
        "Gazebo 3D robotics/rendered sensors",
        None, None, None, None, None, None, None,
        "NOT_MEASURED",
        "binary detected but a validated native headless world/sensor run is required" if gazebo_available else "Gazebo unavailable",
    ))
    chrono = detect_chrono()
    reports.append(TierPerformance(
        4,
        "NOT_EXECUTED_EXTERNAL_DEPENDENCY",
        "Chrono/high-resolution offline research",
        None, None, None, None, None, None, None,
        "NOT_MEASURED",
        chrono.detail,
    ))
    return tuple(reports)


def performance_payload(reports: tuple[TierPerformance, ...]) -> dict:
    return {"tiers": [asdict(report) for report in reports]}

