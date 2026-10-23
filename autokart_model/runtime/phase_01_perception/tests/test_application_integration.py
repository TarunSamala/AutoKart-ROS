"""Execution proof for CORE direct calls and LAB's JSON adapter."""

import json
import math
from pathlib import Path

import pytest

from autokart_perception.application import CoreRuntime, LabAdapter, core_main
from autokart_perception.pipeline import Pipeline
from autokart_perception.recording import Recorder, Replay
from autokart_perception.synthetic import Synthetic
from autokart_perception.shared_core import PoseEstimate
from autokart_perception.lab_api import LabSimulation


def _cone(index, x, y, label):
    return {"id": index, "label": label, "confidence": 0.95, "xy": [x, y],
            "sigma_m": 0.05, "distance_m": math.hypot(x, y), "box": [0, 0, 1, 1]}


def _pose(now, x=0.0, y=0.0, yaw=0.0):
    return {"x_m": x, "y_m": y, "yaw_rad": yaw, "speed_mps": 1.0,
            "timestamp_ns": now, "position_std_m": 0.05}


def _straight():
    now = 1_000_000_000
    return {"timestamp_ns": now, "source": "SIMULATED", "mission": "STRAIGHT_TRACK",
            "pose": _pose(now), "cones": [
                _cone(2 * index, x, 1.5, "blue") for index, x in enumerate((2, 5, 8))] + [
                _cone(2 * index + 1, x, -1.5, "yellow") for index, x in enumerate((2, 5, 8))]}


def _skidpad():
    now = 1_000_000_000
    cones = []
    for center_x in (-9.0, 9.0):
        for radius, label in ((5.0, "blue"), (8.0, "yellow")):
            for index in range(16):
                angle = 2 * math.pi * index / 16
                world_x = center_x + radius * math.cos(angle)
                world_y = radius * math.sin(angle)
                # Kart at (-2.5,0) facing +Y; transform observed ground points to base_link.
                cones.append(_cone(len(cones), world_y, -(world_x + 2.5), label))
    return {"timestamp_ns": now, "source": "SIMULATED", "mission": "SKIDPAD",
            "pose": _pose(now, -2.5, 0, math.pi / 2), "cones": cones}


@pytest.mark.parametrize("sample", [_straight, _skidpad])
def test_core_and_lab_identical_actual_algorithm_stages(sample):
    payload = sample()
    core = CoreRuntime().evaluate_observations(payload)
    lab = LabAdapter().evaluate(json.loads(json.dumps(payload)))
    assert core["geometry"] == lab["geometry"]
    assert core["planning"] == lab["planning"]
    assert core["mission_state"] == lab["mission_state"]
    assert core["control_candidate"] == lab["control_candidate"]
    assert core["vehicle_authorized"] is lab["vehicle_authorized"] is False
    geometry = core["geometry"]
    assert geometry["delaunay_edges"] and geometry["gabriel_edges"]
    if payload["mission"] == "STRAIGHT_TRACK":
        assert geometry["boundaries"]["left_ids"] and geometry["boundaries"]["right_ids"]
        assert geometry["gates"] and geometry["centerline"]
    else:
        assert geometry["skidpad_topology"]["valid"]
        assert geometry["skidpad_topology"]["provenance"] == "OBSERVED_CONES"
        assert core["mission_state"]["state"] == "LEFT"
        assert geometry["centerline"]
    assert core["planning"]["path_valid"]
    assert core["control_candidate"]["valid"]
    assert math.isfinite(core["control_candidate"]["steering_angle_rad"])


def test_core_cli_runs_without_lab_server(tmp_path, capsys):
    sample = tmp_path / "sample.jsonl"
    sample.write_text(json.dumps(_straight()) + "\n")
    assert core_main(["--source", "sample", "--input", str(sample)]) == 0
    output = json.loads(capsys.readouterr().out)
    assert output["geometry"]["gates"]
    assert output["control_candidate"]["valid"]
    assert not output["vehicle_authorized"]


def test_recorded_sensor_through_perception_geometry_planning(tmp_path, capsys):
    runtime = CoreRuntime()
    synthetic_result = Pipeline(None, runtime.config, synthetic=True).process(Synthetic().read())
    recording = tmp_path / "capture"
    Recorder(recording).write(synthetic_result)
    frame = Replay(recording).read()
    output = runtime.evaluate_frame(frame, Pipeline(None, runtime.config), None)
    assert output["perception"]["cone_count"] == 6
    assert output["geometry"]["delaunay_edges"]
    assert output["geometry"]["gabriel_edges"]
    assert output["geometry"]["gates"]
    assert "INVALID_POSE" in output["reasons"]
    assert not output["control_candidate"]["valid"]
    assert not output["vehicle_authorized"]
    assert core_main(["--source", "replay", "--input", str(recording), "--frames", "1"]) == 0
    replay_cli = json.loads(capsys.readouterr().out)
    assert replay_cli["perception"]["cone_count"] == 6
    assert not replay_cli["vehicle_authorized"]


def test_simulated_frame_through_pipeline_to_controller_candidate():
    runtime = CoreRuntime()
    frame = Synthetic().read()
    pose = PoseEstimate(0, 0, 0, 0, frame.timestamp_ns, 0.05)
    output = runtime.evaluate_frame(frame, Pipeline(None, runtime.config, synthetic=True), pose)
    assert output["perception"]["provenance"] == "INJECTED_SYNTHETIC_CONES"
    assert output["perception"]["cone_count"] == 6
    assert output["geometry"]["delaunay_edges"] and output["geometry"]["gabriel_edges"]
    assert output["geometry"]["gates"] and output["geometry"]["centerline"]
    assert output["planning"]["path_valid"] and output["control_candidate"]["valid"]
    assert not output["vehicle_authorized"]


def test_lab_step_edit_and_reset_use_fresh_shared_results():
    lab = LabSimulation()
    first = lab.step()
    assert first["tick"] == 1
    assert first["diagnostic"]["timestamp_ns"] == 1_100_000_000
    before = first["diagnostic"]["geometry"]["centerline"]
    edited = lab.move_cone(0, 2.0, 2.0)
    assert edited["tick"] == 2
    assert edited["diagnostic"]["timestamp_ns"] == 1_200_000_000
    assert edited["diagnostic"]["geometry"]["centerline"] != before
    assert not edited["diagnostic"]["vehicle_authorized"]
    assert LabSimulation().snapshot()["tick"] == 0


@pytest.mark.parametrize("fault", ["missing_pose", "stale_pose", "invalid_cone", "unknown_brake"])
def test_faults_never_authorize_motion(fault):
    payload = _straight()
    if fault == "missing_pose":
        payload["pose"] = None
    elif fault == "stale_pose":
        payload["pose"]["timestamp_ns"] -= 500_000_000
    elif fault == "invalid_cone":
        payload["cones"][0]["xy"] = [float("nan"), 1.5]
    else:
        payload["mission"] = "BRAKING_TEST"
    result = CoreRuntime().evaluate_observations(payload)
    assert not result["planning"]["path_valid"]
    assert not result["control_candidate"]["valid"]
    assert not result["vehicle_authorized"]
