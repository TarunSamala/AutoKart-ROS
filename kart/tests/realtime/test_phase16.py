from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

from kart_sim.gazebo_follower import AuthoritativeSnapshotFactory
from kart_sim.gazebo_preset import load_gazebo_preset, prepare_runtime
from kart_sim.dashboard_server import DashboardServerConfig, DashboardService
from kart_sim.phase16 import gate_native_run
from kart_sim.realtime import RealTimeSynchronizer, TaskBudget, TaskTimingMonitor

ROOT = Path(__file__).resolve().parents[2]


def test_three_presets_preserve_authority_and_order_fidelity(tmp_path):
    realtime = load_gazebo_preset(
        ROOT / "configs/presets/gazebo_realtime_sil.yaml"
    )
    balanced = load_gazebo_preset(
        ROOT / "configs/presets/gazebo_engineering_balanced.yaml"
    )
    offline = load_gazebo_preset(
        ROOT / "configs/presets/gazebo_high_fidelity_offline.yaml"
    )
    assert all(
        preset["physics_authority"] == "internal"
        and preset["vehicle_dynamics_authority"] is False
        for preset in (realtime, balanced, offline)
    )
    assert (
        realtime["sensors"]["rgb_camera"]["width"]
        < balanced["sensors"]["rgb_camera"]["width"]
        < offline["sensors"]["rgb_camera"]["width"]
    )
    runtime = prepare_runtime(
        ROOT / "configs/presets/gazebo_realtime_sil.yaml",
        tmp_path,
        project_root=ROOT,
    )
    world = Path(runtime["world_path"]).read_text(encoding="utf-8")
    assert "<max_step_size>0.01</max_step_size>" in world
    assert "<real_time_update_rate>140</real_time_update_rate>" in world


def test_factory_can_advance_at_physics_rate_and_allocate_at_follower_rate():
    state = SimpleNamespace(
        time_s=0.001,
        x_m=1.0,
        y_m=2.0,
        z_m=0.0,
        roll_rad=0.0,
        pitch_rad=0.0,
        yaw_rad=0.2,
        u_mps=3.0,
        v_mps=0.0,
        yaw_rate_radps=0.1,
    )
    wheels = {
        name: SimpleNamespace(
            angular_speed_radps=10.0 + index,
            steering_angle_rad=0.05 if name.startswith("front") else 0.0,
        )
        for index, name in enumerate(
            ("front_left", "front_right", "rear_left", "rear_right")
        )
    }
    direct = AuthoritativeSnapshotFactory("a" * 64, 1)
    staged = AuthoritativeSnapshotFactory("a" * 64, 1)
    expected = direct.from_plant(
        state=state, wheels=wheels, rear_axle_speed_radps=12.0
    )
    staged.advance_from_plant(
        state=state, wheels=wheels, rear_axle_speed_radps=12.0
    )
    observed = staged.snapshot_from_advanced_plant(
        state=state, wheels=wheels, rear_axle_speed_radps=12.0
    )
    assert observed == expected


def test_task_monitor_reports_percentiles_misses_and_jitter():
    monitor = TaskTimingMonitor({
        "vcu": TaskBudget(1000.0, 0.0005, 0.001, "error"),
    })
    for value in (0.0001, 0.0002, 0.0003, 0.0020, 0.0001):
        monitor.record("vcu", value)
    report = monitor.report()["vcu"]
    assert report["samples"] == 5
    assert report["deadline_misses"] == 1
    assert report["maximum_consecutive_misses"] == 1
    assert report["p99_s"] == 0.002
    assert report["mean_jitter_s"] > 0.0


def test_realtime_synchronizer_modes_and_rewind_guard():
    synchronizer = RealTimeSynchronizer("deterministic_offline")
    first = synchronizer.synchronize(0.1)
    assert first.mode == "deterministic_offline"
    try:
        synchronizer.synchronize(0.05)
    except Exception as error:
        assert "rewind" in str(error)
    else:
        raise AssertionError("rewind must be rejected")


def test_native_gate_requires_safety_timing_and_bounded_sensor_evidence(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    (run / "summary.json").write_text(json.dumps({
        "controlled_stop_complete": True,
        "final_vcu_state": "SAFE",
    }), encoding="utf-8")
    sensor = {
        "delivered": 2,
        "empty_payloads": 0,
        "maximum_queue_depth": 1,
        "maximum_latency_s": 0.02,
        "drops_by_reason": {"stale_rejections": 0},
    }
    (run / "native_results.json").write_text(json.dumps({
        "timestamp_violations": 0,
        "double_physics_diagnostics": 0,
        "sensors": {
            name: sensor
            for name in ("rgb_camera", "depth_camera", "lidar", "imu")
        },
    }), encoding="utf-8")
    (run / "timing_report.json").write_text(json.dumps({
        "aggregate_rtf": 1.02,
        "minimum_window_rtf": 1.0,
        "tasks": {
            "vcu": {"deadline_misses": 0},
            "independent_safety": {"deadline_misses": 0},
        },
    }), encoding="utf-8")
    result = gate_native_run(run, tmp_path / "gate.json")
    assert result["passed"]
    assert result["classification"] == "REALTIME_QUALIFIED"


def test_drop_taxonomy_and_dashboard_performance_view_are_exposed():
    message = (
        ROOT / "ros2_ws/src/autokart_msgs/msg/SensorDeliveryMetadata.msg"
    ).read_text(encoding="utf-8")
    for field in (
        "warmup_rejections",
        "intentional_rate_decimations",
        "queue_overflow_drops",
        "processing_deadline_misses",
        "transport_drops",
        "stale_rejections",
    ):
        assert field in message
    html = (ROOT / "dashboard/index.html").read_text(encoding="utf-8")
    script = (ROOT / "dashboard/js/app.js").read_text(encoding="utf-8")
    assert 'data-panel="performance"' in html
    assert 'id="perf-status"' in html
    assert "sensor_drops_by_reason" in script


def test_dashboard_replays_native_snapshot_and_reports_degraded_timing(tmp_path):
    telemetry = tmp_path / "internal_vehicle_log.jsonl"
    telemetry.write_text(json.dumps({
        "simulation_time_s": 0.001,
        "sequence": 1,
        "x_m": 10.0,
        "y_m": 0.0,
        "z_m": 0.0,
        "roll_rad": 0.0,
        "pitch_rad": 0.0,
        "yaw_rad": 0.1,
        "speed_mps": 1.0,
        "vcu_state": "AUTONOMOUS",
        "faults": [],
    }) + "\n", encoding="utf-8")
    (tmp_path / "summary.json").write_text(json.dumps({
        "cpu_time_s": 1.0,
        "memory_mib": 64.0,
    }), encoding="utf-8")
    (tmp_path / "timing_report.json").write_text(json.dumps({
        "aggregate_rtf": 1.01,
        "minimum_window_rtf": 1.0,
        "tasks": {
            "vcu": {"deadline_misses": 2},
            "independent_safety": {"deadline_misses": 0},
        },
    }), encoding="utf-8")
    (tmp_path / "native_results.json").write_text(json.dumps({
        "sensors": {
            "rgb_camera": {
                "maximum_latency_s": 0.01,
                "maximum_queue_depth": 1,
                "drops_by_reason": {"stale_rejections": 1},
            },
        },
    }), encoding="utf-8")
    service = DashboardService(DashboardServerConfig(
        telemetry=telemetry,
        track=ROOT / "configs/tracks/research_circle.yaml",
        kart=ROOT / "configs/karts/prototype_dynamic.yaml",
        static_root=ROOT / "dashboard",
    ))
    record = service.telemetry()[0]
    performance = service.performance()
    assert record["pose"]["x_m"] == 10.0
    assert record["vcu"]["state"] == "AUTONOMOUS"
    assert performance["qualification"] == "REALTIME_DEGRADED"
    assert performance["deadline_misses"]["vcu"] == 2
