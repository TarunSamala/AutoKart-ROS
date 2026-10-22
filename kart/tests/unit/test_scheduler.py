from kart_sim.scheduler import FixedStepScheduler
from kart_sim.types import SimulationError


def test_scheduler_is_fixed_step_and_deterministic() -> None:
    observed: list[tuple[float, float]] = []
    scheduler = FixedStepScheduler(0.001)
    scheduler.add_callback(lambda time_s, dt_s: observed.append((time_s, dt_s)))
    scheduler.run_until(0.010)
    assert scheduler.step_count == 10
    assert scheduler.time_s == 0.010
    assert observed[0] == (0.0, 0.001)
    assert observed[-1] == (0.009000000000000001, 0.001)


def test_scheduler_rejects_invalid_step_size() -> None:
    try:
        FixedStepScheduler(0.0)
    except SimulationError:
        pass
    else:
        raise AssertionError("invalid time step accepted")
