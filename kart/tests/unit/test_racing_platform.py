from pathlib import Path

import pytest

from kart_sim.cones import ConeClass, ConeNoise, ConePerception, boundary_cones
from kart_sim.race_control import RaceControl, SessionState
from kart_sim.racing_autonomy import FollowTheGapController, cone_midpoint_path
from kart_sim.track import Track


ROOT = Path(__file__).parents[2]


def test_cones_are_semantic_deterministic_and_ground_truth_separated() -> None:
    track = Track.from_yaml(ROOT / "configs/tracks/cone_track.yaml")
    cones = boundary_cones(track, 4.0)
    assert {cone.semantic_class for cone in cones} == {ConeClass.BLUE, ConeClass.YELLOW}
    perception_a = ConePerception(ConeNoise(missed_detection_probability=0), seed=3)
    perception_b = ConePerception(ConeNoise(missed_detection_probability=0), seed=3)
    state = __import__("kart_sim.types", fromlist=["PlanarKartState"]).PlanarKartState(
        x_m=track.points[0][0], y_m=track.points[0][1]
    )
    assert perception_a.detect(state, cones) == perception_b.detect(state, cones)
    trajectory = cone_midpoint_path(cones, timestamp_s=0.0)
    assert trajectory.header.source == "cone_midpoint_planner"


def test_gap_controller_does_not_treat_unknown_as_free() -> None:
    controller = FollowTheGapController()
    stopped = controller.compute([float("inf"), float("nan"), 0.2, 0.1])
    assert stopped.target_speed_mps == 0.0
    moving = controller.compute([0.2, 2.0, 3.0, 2.5, 0.1])
    assert moving.target_speed_mps > 0 and moving.gap_start_index == 1


def test_race_control_transitions_timing_penalties_and_estop_authority() -> None:
    race = RaceControl(("/kart_01",))
    for state in (SessionState.GRID, SessionState.READY, SessionState.COUNTDOWN, SessionState.RUNNING):
        race.transition(state, 0.0)
    race.lap_crossing("/kart_01", 1.0)
    race.lap_crossing("/kart_01", 11.0)
    race.penalty("/kart_01", "cone", 5.0)
    race.update_safety("/kart_01", "EMERGENCY_STOP", ("ESTOP-ACTIVE",), True)
    snapshot = race.snapshot()["karts"]["/kart_01"]
    assert snapshot["best_lap_s"] == pytest.approx(10.0)
    assert snapshot["cone_penalties"] == 1 and snapshot["dnf"]

