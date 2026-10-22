from pathlib import Path

import pytest

from kart_sim.fast_env import AutoKartFastEnv, VectorAutoKartEnv
from kart_sim.types import SimulationError
from kart_sim.vcu import MotionCommand, VcuState


ROOT = Path(__file__).parents[2]
TRACK = ROOT / "configs/tracks/cone_track.yaml"
KART = ROOT / "configs/karts/prototype_dynamic.yaml"


def run_trace(seed: int) -> list[tuple[float, float, float, str]]:
    env = AutoKartFastEnv(TRACK, KART, control_dt_s=0.01, lidar_beams=24, maximum_steps=10)
    env.reset(seed=seed, options={"arm": True, "operator_acknowledged": True})
    trace = []
    for _ in range(10):
        observation, _, _, _, _ = env.step((1.0, 0.02))
        trace.append((env.state.x_m, env.state.y_m, observation["lidar"][3], observation["safety_state"]))
    return trace


def test_fast_environment_is_seeded_and_vcu_routed() -> None:
    assert run_trace(7) == run_trace(7)
    env = AutoKartFastEnv(TRACK, KART, control_dt_s=0.01, maximum_steps=2)
    _, info = env.reset(seed=2, options={"arm": True, "operator_acknowledged": True})
    assert info["vcu_routed"] and env.vcu.state == VcuState.AUTONOMOUS
    with pytest.raises(SimulationError):
        env.step((9.0, 0.0))


def test_reset_does_not_automatically_rearm_or_clear_latched_faults() -> None:
    env = AutoKartFastEnv(TRACK, KART, control_dt_s=0.01)
    _, info = env.reset(seed=3)
    assert env.vcu.state == VcuState.SAFE
    with pytest.raises(SimulationError, match="not autonomously armed"):
        env.step((1.0, 0.0))
    env.arm_autonomy(operator_acknowledged=True)
    env.inject_fault("physical_estop")
    env.step((1.0, 0.0))
    with pytest.raises(SimulationError, match="repair acknowledgement"):
        env.reset(seed=3)


def test_reset_epoch_rejects_old_commands_and_vector_namespaces_are_isolated() -> None:
    first = AutoKartFastEnv(TRACK, KART, namespace="/kart_01", control_dt_s=0.01)
    old_epoch = first._epoch
    first.reset(seed=1, options={"arm": True, "operator_acknowledged": True})
    first.vcu.independent.pulse(0.0)
    old = MotionCommand("baseline_controller", 1, 0.0, 0.05, 1.0, 0.0, reset_epoch=old_epoch).signed()
    first.vcu.update(0.0, old)
    assert first.vcu.state == VcuState.CONTROLLED_STOP
    second = AutoKartFastEnv(TRACK, KART, namespace="/kart_02", control_dt_s=0.01)
    vector = VectorAutoKartEnv((first, second))
    assert [env.namespace for env in vector.environments] == ["/kart_01", "/kart_02"]
