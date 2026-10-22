import pytest

from kart_sim.types import PlanarKartState, SimulationError
from kart_sim.world_adapter import DynamicActor, VisualFollowerSession, WorldPose, simulator_to_unreal, unreal_to_simulator


def test_coordinate_round_trip_and_authoritative_visual_following():
    pose = WorldPose(1, 2, .3, .4)
    assert unreal_to_simulator(simulator_to_unreal(pose)) == pose
    session = VisualFollowerSession(.01)
    session.add_actor(DynamicActor("cone_1", "cone", WorldPose(5, 1, 0, 0)))
    tick = session.tick_from_authoritative_state(PlanarKartState(time_s=.1, x_m=2, y_m=3, yaw_rad=.2))
    assert tick.actor_poses["kart_visual"] == WorldPose(2, -3, 0, -.2)
    session.pause(); assert session.tick_from_authoritative_state(PlanarKartState(time_s=.2)).simulation_time_s == .1


def test_duplicate_actor_is_rejected():
    session = VisualFollowerSession(.01); actor = DynamicActor("a", "barrier", WorldPose(0, 0, 0, 0))
    session.add_actor(actor)
    with pytest.raises(SimulationError, match="unique"):
        session.add_actor(actor)
