from pathlib import Path

from kart_sim.fast_env import AutoKartFastEnv
from kart_sim.vcu import VcuState


ROOT = Path(__file__).parents[2]


def test_independent_estop_overrides_fast_mode_and_removes_propulsion() -> None:
    env = AutoKartFastEnv(
        ROOT / "configs/tracks/acceleration_event.yaml",
        ROOT / "configs/karts/prototype_dynamic.yaml",
        control_dt_s=0.01,
    )
    env.reset(seed=1, options={"arm": True, "operator_acknowledged": True})
    env.vcu.independent.estop = True
    _, _, terminated, _, info = env.step((4.0, 0.0))
    assert terminated
    assert env.vcu.state in {VcuState.EMERGENCY_STOP, VcuState.FAULT_LOCKOUT}
    assert env.vcu_output is not None
    assert not env.vcu_output.motor_enable and env.vcu_output.command.drive == 0.0
    assert info["vcu_faults"]
