from __future__ import annotations

import json
from pathlib import Path

from kart_sim.scenario import run_scenario


ROOT = Path(__file__).parents[2]


def rows(output: Path) -> list[dict]:
    return [json.loads(line) for line in (output / "telemetry.jsonl").read_text(encoding="utf-8").splitlines()]


def test_constant_radius_dashboard_run_uses_track_pose_wheels_and_surface(tmp_path):
    telemetry = rows(run_scenario(ROOT / "configs/scenarios/dashboard_research_lap.yaml", tmp_path))
    assert telemetry[0]["track"]["name"] == "research_circle_provisional"
    assert telemetry[-1]["track"]["projection"]["s_m"] > telemetry[0]["track"]["projection"]["s_m"]
    assert max(abs(row["track"]["projection"]["lateral_m"]) for row in telemetry) < 1.2
    assert any(abs(row["subsystems"]["steering"]["left_wheel_angle_rad"]) > 0.05 for row in telemetry)
    assert any(abs(row["subsystems"]["four_wheel"]["wheels"]["front_left"]["angular_speed_radps"]) > 1 for row in telemetry)
    assert {
        surface["material_id"]
        for row in telemetry
        for surface in row["track"]["wheel_surfaces"].values()
    } == {"dry_asphalt", "wet_asphalt"}
    assert all(row["vcu"]["state"] == "AUTONOMOUS" for row in telemetry)


def test_slalom_dashboard_run_steering_visual_changes_sign_from_schedule(tmp_path):
    telemetry = rows(run_scenario(ROOT / "configs/scenarios/dashboard_slalom.yaml", tmp_path))
    steering = [row["subsystems"]["steering"]["left_wheel_angle_rad"] for row in telemetry]
    y_positions = [row["state"]["y_m"] for row in telemetry]
    assert max(steering) > 0.05 and min(steering) < -0.05
    assert max(y_positions) - min(y_positions) > 0.1
    assert all(row["commands"]["requested"] is not None for row in telemetry)
