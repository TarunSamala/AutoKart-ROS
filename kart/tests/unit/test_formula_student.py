from pathlib import Path

import pytest

from kart_sim.diagnostics_bridge import project_faults
from kart_sim.formula_student import EventEvaluator, EventType, load_event


ROOT = Path(__file__).parents[2]


def test_event_templates_load_and_acceleration_scores_penalties() -> None:
    # my_track.yaml is a documentation/discovery entry only, not driven through
    # load_event()/EventEvaluator -- see test_my_track_is_registered_but_not_lap_timed below.
    templates = {
        path.stem: load_event(path) for path in sorted((ROOT / "configs/events").glob("*.yaml")) if path.stem != "my_track"
    }
    assert len(templates) == 5
    assert templates["emergency_brake_test"].event_type == EventType.EMERGENCY_BRAKE_TEST
    evaluator = EventEvaluator(templates["acceleration"])
    evaluator.update(timestamp_s=0, track_s_m=0, track_length_m=100, speed_mps=0)
    result = evaluator.update(
        timestamp_s=10,
        track_s_m=75,
        track_length_m=100,
        speed_mps=8,
        cone_hits=1,
    )
    assert result.completed and result.penalty_time_s == pytest.approx(2.0)


def test_my_track_is_registered_but_not_lap_timed() -> None:
    assert EventType.MY_TRACK.value == "my_track"
    from kart_sim.types import SimulationError
    with pytest.raises(SimulationError, match="my_track_runner"):
        load_event(ROOT / "configs/events/my_track.yaml")


def test_ebs_measures_stop_and_diagnostics_project_to_all_consumers() -> None:
    evaluator = EventEvaluator(load_event(ROOT / "configs/events/emergency_brake_test.yaml"))
    evaluator.update(
        timestamp_s=5,
        track_s_m=40,
        track_length_m=100,
        speed_mps=8,
        estop_active=True,
    )
    result = evaluator.update(
        timestamp_s=7,
        track_s_m=52,
        track_length_m=100,
        speed_mps=0,
        estop_active=True,
    )
    assert result.completed and result.stopping_distance_m == pytest.approx(12)
    diagnostics = project_faults(
        "/kart_01",
        7,
        vcu_state="EMERGENCY_STOP",
        vcu_faults=("ESTOP-ACTIVE",),
        simulation_faults=("wet_surface",),
    )
    assert all(item.level == "ERROR" for item in diagnostics)
    assert all("dashboard" in dict(item.values)["visible_in"] for item in diagnostics)

