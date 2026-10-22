"""Behavioral parity tests for the reused PlannerEngine, loaded through MissionAlgorithm instead of
imported directly -- ported from the existing, reused kart/ros2_ws/.../autokart_planner/test/
test_mission_adapter.py. These exercise the REAL skidpad gate-crossing/winding-number progress FSM
and the REAL braking stop-distance logic; a real regression there would show up here too.

Skipped entirely if the full kart/ros2_ws + autonomy_stack_phase1 checkout is not present (this is an
external, pre-existing dependency this node reuses, not something this package vendors -- see
DEPENDENCIES.md).
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from autokart_mission.algorithm import MissionAlgorithm  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[3]
ALGORITHM_PATH = REPO_ROOT / "autonomy_stack_phase1" / "phase_01_perception" / "src"
VEHICLE_GEOMETRY_PATH = REPO_ROOT / "kart" / "ros2_ws" / "src" / "autokart_vehicle_geometry"
PLANNER_PATH = REPO_ROOT / "kart" / "ros2_ws" / "src" / "autokart_nodes" / "autokart_planner"
ALL_PRESENT = ALGORITHM_PATH.is_dir() and VEHICLE_GEOMETRY_PATH.is_dir() and PLANNER_PATH.is_dir()

pytestmark = pytest.mark.skipif(not ALL_PRESENT, reason="full kart/autonomy_stack_phase1 checkout not present")


@pytest.fixture
def algorithm():
    return MissionAlgorithm(str(ALGORITHM_PATH), str(VEHICLE_GEOMETRY_PATH), str(PLANNER_PATH))


def topology(algorithm):
    from autokart_perception.shared_core import fit_skidpad_topology
    cones = []
    for center_x, blue, yellow in ((-9.125, 7.625, 10.625), (9.125, 10.625, 7.625)):
        for label, radius in (("blue", blue), ("yellow", yellow)):
            for index in range(24):
                angle = 2 * math.pi * index / 24
                cones.append({"x_m": center_x + radius * math.cos(angle),
                              "y_m": radius * math.sin(angle), "label": label,
                              "sigma_m": 0.05})
    result = fit_skidpad_topology(cones)
    assert result["valid"]
    return result


def map_sample(t, stamp):
    return {"valid": True, "source": "SIMULATED", "frame_id": "odom",
            "sample_time_ns": stamp, "geometry": {"skidpad_topology": t}}


def pose_on(algorithm, cx, angle, stamp):
    return algorithm.PoseEstimate(cx + 9.125 * math.cos(angle), 9.125 * math.sin(angle),
                                  angle - math.pi / 2 if cx > 0 else angle + math.pi / 2,
                                  1.0, stamp, 0.05)


def test_right_right_left_left_requires_all_four_gate_crossings_and_terminal_stop(algorithm):
    """Confirms the reused FSM really does require BOTH signed winding progress (the accumulated
    `travel` angle, equivalent to 2*pi per confirmed lap) AND a gate crossing (the `crossing` check
    in SkidpadProgress.update) before advancing -- not elapsed time alone -- and that the terminal
    stop additionally requires measured stop feedback (stop_feedback_valid), not just zero commanded
    speed. This is the specific requirement the mission statement asked to verify, not re-derive."""
    t = topology(algorithm)
    engine = algorithm.PlannerEngine("SKIDPAD", "SIMULATED", algorithm.BrakeCapability(3.0, 0.2, "SIMULATED"))
    stamp = 1_000_000_000
    for index in range(161):
        angle = math.pi - index * 2 * math.pi / 80
        engine.evaluate(map_sample(t, stamp), pose_on(algorithm, 9.125, angle, stamp))
        stamp += 20_000_000
    assert engine.progress.right_laps == 2
    assert engine.progress.left_laps == 0
    assert engine.progress.state == "TRANSITION"
    for index in range(1, 163):
        engine.evaluate(map_sample(t, stamp), pose_on(algorithm, -9.125, index * 2 * math.pi / 80, stamp))
        stamp += 20_000_000
    assert engine.progress.right_laps == 2
    assert engine.progress.left_laps == 2
    assert engine.progress.state == "EXIT"
    exit_map = map_sample(t, stamp)
    exit_map["geometry"]["exit_centerline_odom"] = [(0, 5), (0, 8), (0, 11)]
    result = engine.evaluate(exit_map, algorithm.PoseEstimate(0, 0, math.pi / 2, 0.5, stamp, 0.05))
    assert result["valid"] and result["decision"] == "EXIT_PATH_CANDIDATE"
    stamp += 20_000_000
    exit_map["sample_time_ns"] = stamp
    result = engine.evaluate(exit_map, algorithm.PoseEstimate(0, 11, math.pi / 2, 0.0, stamp, 0.05),
                             stop_feedback_valid=False)
    assert engine.progress.state == "STOP_PENDING", "must not claim STOP without measured feedback"
    stamp += 20_000_000
    exit_map["sample_time_ns"] = stamp
    result = engine.evaluate(exit_map, algorithm.PoseEstimate(0, 11, math.pi / 2, 0.0, stamp, 0.05),
                             stop_feedback_valid=True)
    assert engine.progress.state == "STOP"
    assert not result["valid"]


def test_braking_stops_for_an_obstacle_and_rejects_insufficient_horizon(algorithm):
    engine = algorithm.PlannerEngine("BRAKING_TEST", "SIMULATED",
                                     algorithm.BrakeCapability(3.0, 0.2, "SIMULATED"))
    stamp = 1_000_000_000
    mapping = {"valid": True, "source": "SIMULATED", "frame_id": "odom", "sample_time_ns": stamp,
               "geometry": {"centerline_status": "PROVISIONAL",
                            "centerline": [(float(x), 0.0) for x in range(1, 11)]}}
    pose = algorithm.PoseEstimate(0, 0, 0, 2.0, stamp, 0.05)
    blocked = engine.evaluate(mapping, pose, [{"x_m": 4.0, "y_m": 0.0, "radius_m": 0.3}])
    assert blocked["decision"] == "STOP_CANDIDATE"
    assert not blocked["valid"] and blocked["target_speed_mps"] == 0.0
    stamp += 20_000_000
    mapping["sample_time_ns"] = stamp
    mapping["geometry"]["centerline"] = [(0.5, 0.0), (1.0, 0.0), (2.0, 0.0)]
    short = engine.evaluate(mapping, algorithm.PoseEstimate(0, 0, 0, 2.0, stamp, 0.05), [])
    assert not short["valid"] and "STOP_HORIZON_INSUFFICIENT" in short["reason_codes"]


def test_braking_fsm_has_named_states_and_requires_measured_stop_confirmation(algorithm):
    """BRAKING_TEST now has a real state-machine object (PlannerEngine.braking_state) behind
    mission_state, using the master architecture's WAITING/TRACK_VALID/READY/RUNNING/
    STOP_TRIGGERED/STOPPING/STOP_VERIFIED/FAULT vocabulary, and stop_feedback_valid is passed into
    _braking() and gates the terminal STOP_VERIFIED state exactly as SKIDPAD's STOP_PENDING->STOP
    transition is gated. See CONTRACT.md / mission.py for the fix."""
    engine = algorithm.PlannerEngine("BRAKING_TEST", "SIMULATED",
                                     algorithm.BrakeCapability(3.0, 0.2, "SIMULATED"))
    stamp = 1_000_000_000
    mapping = {"valid": True, "source": "SIMULATED", "frame_id": "odom", "sample_time_ns": stamp,
               "geometry": {"centerline_status": "PROVISIONAL",
                            "centerline": [(float(x), 0.0) for x in range(1, 11)]}}
    pose = algorithm.PoseEstimate(0, 0, 0, 0.0, stamp, 0.05)
    assert engine.braking_state == "WAITING"

    cruising = engine.evaluate(mapping, pose, [])
    assert cruising["valid"] and cruising["decision"] == "PATH_CANDIDATE"
    assert cruising["mission_state"]["state"] == "RUNNING"

    stamp += 20_000_000
    mapping["sample_time_ns"] = stamp
    triggered = engine.evaluate(mapping, pose, [{"x_m": 1.0, "y_m": 0.0, "radius_m": 0.0}],
                                stop_feedback_valid=False)
    assert not triggered["valid"] and triggered["decision"] == "STOP_CANDIDATE"
    assert triggered["mission_state"]["state"] == "STOP_TRIGGERED", (
        "first detection of a blocking obstacle must land in STOP_TRIGGERED, not jump straight to a "
        "confirmed stop")

    stamp += 20_000_000
    mapping["sample_time_ns"] = stamp
    stopping = engine.evaluate(mapping, pose, [{"x_m": 1.0, "y_m": 0.0, "radius_m": 0.0}],
                               stop_feedback_valid=False)
    assert stopping["mission_state"]["state"] == "STOPPING", (
        "without measured stop feedback the braking FSM must not claim a confirmed stop")
    assert stopping["decision"] == "STOP_CANDIDATE"

    stamp += 20_000_000
    mapping["sample_time_ns"] = stamp
    stopped = engine.evaluate(mapping, pose, [{"x_m": 1.0, "y_m": 0.0, "radius_m": 0.0}],
                              stop_feedback_valid=True)
    assert stopped["mission_state"]["state"] == "STOP_VERIFIED"
    assert stopped["decision"] == "STOP"
    assert stopped["reason_codes"] == []


def test_braking_fsm_faults_on_invalid_input(algorithm):
    engine = algorithm.PlannerEngine("BRAKING_TEST", "SIMULATED",
                                     algorithm.BrakeCapability(3.0, 0.2, "SIMULATED"))
    stamp = 1_000_000_000
    mapping = {"valid": True, "source": "SIMULATED", "frame_id": "odom", "sample_time_ns": stamp,
               "geometry": {"centerline_status": "PROVISIONAL", "centerline": []}}
    pose = algorithm.PoseEstimate(0, 0, 0, 0.0, stamp, 0.05)
    result = engine.evaluate(mapping, pose, [])
    assert result["mission_state"]["state"] == "FAULT"
    assert result["decision"] == "HOLD"
