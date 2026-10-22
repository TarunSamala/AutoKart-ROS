from __future__ import annotations

from kart_sim.autonomy import ControllerManager, MessageHeader, PurePursuitController, Trajectory, TrajectoryPoint
from kart_sim.ros2_sil import DeterministicRosBridge, SimulationClock
from kart_sim.types import PlanarKartState
from kart_sim.vcu import VCU, VcuInputs, VcuState


def test_sil_controller_command_uses_vcu_freshness_and_clock_is_deterministic():
    clock = SimulationClock(0.004)
    bridge = DeterministicRosBridge(clock)
    path = Trajectory(MessageHeader(0, "map", 1, "planner"), (
        TrajectoryPoint(0, 0, 0, speed_mps=3), TrajectoryPoint(1, 5, 0, speed_mps=3),
    ), 2)
    path.validate(current_time_s=0)
    manager = ControllerManager(PurePursuitController(maximum_steering_rate_radps=100))
    command = manager.command(PlanarKartState(), path, 0, .004, safety_speed_limit_mps=3)
    vcu = VCU(); vcu.state = VcuState.AUTONOMOUS; vcu.independent.pulse(0)
    accepted = vcu.cycle(VcuInputs(0, command, 0, 0, 0, 0, 0, True))
    assert accepted.command.drive > 0
    # Reliable transport does not excuse a stale high-level command.
    clock.step(50); vcu.independent.pulse(clock.time_s)
    stale = vcu.cycle(VcuInputs(clock.time_s, command, 0, 0, 2e5, 0, 0, True))
    assert stale.state == VcuState.CONTROLLED_STOP and stale.command.brake > 0
    assert bridge.publish_clock().time_s == clock.time_s
