from __future__ import annotations

import json
from pathlib import Path

import pytest

from kart_sim.config import load_kart_config
from kart_sim.dashboard import (
    ReplayDataset,
    TelemetryStore,
    dashboard_kart_payload,
    dashboard_track_payload,
    normalize_dashboard_record,
)
from kart_sim.dashboard_server import DashboardServerConfig, DashboardService, create_dashboard_app
from kart_sim.scenario import run_scenario
from kart_sim.track import Track
from kart_sim.types import SimulationError


ROOT = Path(__file__).parents[2]


def sample_row(time_s: float = 1.0) -> dict:
    return {
        "state": {
            "time_s": time_s,
            "x_m": 10.0,
            "y_m": 0.0,
            "z_m": 0.0,
            "roll_rad": 0.01,
            "pitch_rad": 0.02,
            "yaw_rad": 1.5707963267948966,
            "u_mps": 3.0,
            "v_mps": 0.2,
            "ax_mps2": 0.4,
            "ay_mps2": 1.2,
            "yaw_rate_radps": 0.3,
        },
        "commands": {
            "requested": {"drive": 0.5, "brake": 0.0, "steering_rad": 0.2},
            "validated": {"drive": 0.5, "brake": 0.0, "steering_rad": 0.2},
            "safety_limited": {"drive": 0.4, "brake": 0.0, "steering_rad": 0.2},
            "final": {"drive": 0.4, "brake": 0.0, "steering_rad": 0.2},
        },
        "vcu": {"state": "AUTONOMOUS", "faults": []},
        "subsystems": {
            "steering": {"left_wheel_angle_rad": 0.21, "right_wheel_angle_rad": 0.19},
            "four_wheel": {"wheels": {"front_left": {"angular_speed_radps": 21.0}}},
        },
    }


def test_normalized_pose_track_and_command_data_come_from_simulator():
    track = Track.from_yaml(ROOT / "configs/tracks/research_circle.yaml")
    normalized = normalize_dashboard_record(sample_row(), track=track, sequence=7)
    assert normalized["sequence"] == 7
    assert normalized["pose"] == {
        "x_m": 10.0,
        "y_m": 0.0,
        "z_m": 0.0,
        "roll_rad": 0.01,
        "pitch_rad": 0.02,
        "yaw_rad": pytest.approx(1.5707963267948966),
    }
    assert normalized["track"]["s_m"] == pytest.approx(0.0)
    assert normalized["track"]["lateral_m"] == pytest.approx(0.0)
    assert normalized["commands"]["final"]["drive"] == 0.4
    assert normalized["subsystems"]["steering"]["left_wheel_angle_rad"] == 0.21


def test_track_and_kart_browser_schemas_preserve_physics_dimensions():
    track = Track.from_yaml(ROOT / "configs/tracks/research_circle.yaml")
    kart = load_kart_config(ROOT / "configs/karts/prototype_dynamic.yaml")
    track_payload = dashboard_track_payload(track, spacing_m=1.0)
    kart_payload = dashboard_kart_payload(kart)
    assert track_payload["name"] == track.name
    assert track_payload["closed"]
    assert track_payload["points"][0]["left_width_m"] == 2.0
    assert track_payload["surface_regions"][0]["material"] == "wet_asphalt"
    assert kart_payload["dimensions"]["wheelbase_m"] == kart.kart.wheelbase_m
    assert kart_payload["dimensions"]["wheel_radius_m"] == kart.tires.radius_m
    assert kart_payload["placeholder"] and kart_payload["replacement_asset"].endswith("kart.glb")
    with pytest.raises(SimulationError):
        dashboard_track_payload(track, spacing_m=0.0)


def test_replay_normalization_is_bounded_and_deterministic(tmp_path):
    telemetry = tmp_path / "telemetry.jsonl"
    telemetry.write_text(
        "\n".join(json.dumps({**sample_row(index / 10), "track_s_m": index}) for index in range(10)) + "\n",
        encoding="utf-8",
    )
    replay = ReplayDataset.from_jsonl(telemetry)
    assert [record["sequence"] for record in replay.decimate(3)] == [0, 4, 9]
    assert replay.decimate(1)[0]["sequence"] == 9
    with pytest.raises(SimulationError):
        replay.decimate(0)


def test_dashboard_service_is_read_only_and_exposes_real_run(tmp_path):
    output = run_scenario(ROOT / "configs/scenarios/dashboard_research_lap.yaml", tmp_path)
    config = DashboardServerConfig(
        telemetry=output / "telemetry.jsonl",
        track=ROOT / "configs/tracks/research_circle.yaml",
        kart=ROOT / "configs/karts/prototype_dynamic.yaml",
        static_root=ROOT / "dashboard",
        maximum_replay_points=400,
    )
    service = DashboardService(config)
    assert service.manifest()["actuator_authority"] == "NONE"
    assert service.manifest()["read_only"]
    assert service.manifest()["scenario"] == "dashboard_research_lap"
    records = service.telemetry()
    assert len(records) <= 400
    assert records[0]["pose"]["x_m"] != records[-1]["pose"]["x_m"]
    assert records[0]["track"]["s_m"] != records[-1]["track"]["s_m"]
    assert records[-1]["subsystems"]["four_wheel"]["wheels"]["front_left"]["angular_speed_radps"] != 0
    app = create_dashboard_app(config)
    resources = {resource.canonical for resource in app.router.resources()}
    assert "/api/manifest" in resources and "/api/telemetry" in resources and "/ws" in resources
    assert not any("command" in resource or "actuator" in resource or "reset" in resource for resource in resources)
    live_config = DashboardServerConfig(
        telemetry=output / "telemetry.jsonl",
        track=config.track,
        kart=config.kart,
        static_root=config.static_root,
        maximum_replay_points=7000,
        live_tail=True,
    )
    live = DashboardService(live_config)
    initial = len(live.dataset.records)
    appended = sample_row(live.dataset.records[-1]["timestamp_s"] + 0.001)
    with live_config.telemetry.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(appended) + "\n")
    live.refresh()
    assert len(live.dataset.records) == initial + 1
    assert live.dataset.records[-1]["sequence"] == initial


def test_dashboard_configuration_rejects_invalid_bounds(tmp_path):
    telemetry = tmp_path / "missing.jsonl"
    config = DashboardServerConfig(
        telemetry=telemetry,
        track=ROOT / "configs/tracks/research_circle.yaml",
        kart=ROOT / "configs/karts/prototype_dynamic.yaml",
        static_root=ROOT / "dashboard",
    )
    with pytest.raises(SimulationError, match="telemetry file not found"):
        DashboardService(config)


def test_store_marks_backward_timestamp_invalid(tmp_path):
    telemetry = tmp_path / "telemetry.jsonl"
    telemetry.write_text(
        json.dumps({"state": {"time_s": 1.0}}) + "\n" + json.dumps({"state": {"time_s": 0.5}}) + "\n",
        encoding="utf-8",
    )
    store = TelemetryStore.from_jsonl(telemetry)
    assert store.records[0].valid
    assert store.records[1].stale and not store.records[1].valid
