from __future__ import annotations

import pytest

from kart_sim.autonomy import (
    ControllerManager,
    LongitudinalPidController,
    MessageHeader,
    PerceptionContract,
    PurePursuitController,
    SensorEnvelope,
    StanleyController,
    Trajectory,
    TrajectoryPoint,
)
from kart_sim.ros2_sil import DeterministicRosBridge, FrameTree, SimulationClock, Transform, validate_qos
from kart_sim.types import PlanarKartState, SimulationError
from kart_sim.vcu import VCU, VcuInputs, VcuState


def trajectory(points: tuple[TrajectoryPoint, ...] | None = None) -> Trajectory:
    return Trajectory(
        MessageHeader(0.0, "map", 1, "planner"),
        points or (
            TrajectoryPoint(0.0, 0.0, 0.0, heading_rad=0.0, speed_mps=2.0),
            TrajectoryPoint(1.0, 4.0, 0.0, heading_rad=0.0, speed_mps=2.0),
        ),
        2.0,
    )


def test_trajectory_requires_time_order_horizon_frame_and_finite_values():
    valid = trajectory()
    valid.validate(current_time_s=0.0)
    with pytest.raises(SimulationError, match="strictly increasing"):
        trajectory((TrajectoryPoint(0, 0, 0), TrajectoryPoint(0, 1, 0))).validate(current_time_s=0.0)
    with pytest.raises(SimulationError, match="stale"):
        Trajectory(valid.header, valid.points, -0.1).validate(current_time_s=0.0)
    with pytest.raises(SimulationError, match="speed"):
        trajectory((TrajectoryPoint(0, 0, 0, speed_mps=100), TrajectoryPoint(1, 1, 0))).validate(current_time_s=0.0)


def test_pure_pursuit_stanley_and_longitudinal_signs_limits_and_vcu_boundary():
    state = PlanarKartState(u_mps=1.0)
    pp = PurePursuitController(maximum_steering_rate_radps=100.0)
    assert abs(pp.compute(state, trajectory(), 0.01).steering_rad) < 1e-9
    left = trajectory((TrajectoryPoint(0, 0, 2, heading_rad=0, speed_mps=2), TrajectoryPoint(1, 4, 2, heading_rad=0, speed_mps=2)))
    assert pp.compute(state, left, 0.01).steering_rad > 0.0
    stanley = StanleyController(maximum_steering_rate_radps=100.0)
    assert stanley.compute(PlanarKartState(x_m=0, y_m=1, u_mps=2), trajectory(), 0.01).steering_rad < 0.0
    assert stanley.compute(PlanarKartState(x_m=0, y_m=-1, u_mps=2), trajectory(), 0.01).steering_rad > 0.0
    pid = LongitudinalPidController(kp=1.0, ki=1.0, integral_limit=.2)
    accelerating = pid.compute(5, 0, .1, safety_speed_limit_mps=2)
    assert accelerating.drive > 0 and accelerating.target_speed_mps == 2
    assert pid.compute(5, 0, .1, safety_speed_limit_mps=2, brake_stuck=True).drive == 0
    manager = ControllerManager(PurePursuitController(maximum_steering_rate_radps=100.0))
    high_level = manager.command(state, trajectory(), 0.0, 0.01, safety_speed_limit_mps=2)
    assert high_level.checksum == high_level.crc()
    vcu = VCU(); vcu.state = VcuState.AUTONOMOUS; vcu.independent.pulse(0.0)
    output = vcu.cycle(VcuInputs(0, high_level, 0, 0, 2e5, 0, 0, True))
    assert output.command.drive >= 0.0  # Controller remains upstream of VCU.


def test_clock_pause_step_reset_sensor_timestamp_and_production_ground_truth_isolation():
    clock = SimulationClock(.01)
    assert clock.step().time_s == pytest.approx(.01)
    clock.pause(); assert clock.step(5).time_s == pytest.approx(.01)
    clock.resume(); assert clock.step(2).time_s == pytest.approx(.03)
    assert clock.reset().epoch == 1 and clock.time_s == 0.0 and clock.paused
    bridge = DeterministicRosBridge(clock)
    envelope = SensorEnvelope(MessageHeader(.1, "imu", 2, "imu"), .09, .11, .1, .2, {"ax": 1.0})
    bridge.publish_sensor("/sensors/imu", envelope)
    published = bridge.messages[-1].message
    assert published.measurement_timestamp_s == .1 and published.delivery_timestamp_s == .2
    with pytest.raises(SimulationError, match="ground truth"):
        bridge.publish_debug_ground_truth("vehicle_state", MessageHeader(0, "map", 0, "sim"), {})


def test_frame_tree_and_qos_validation():
    tree = FrameTree()
    tree.add(Transform("map", "odom")); tree.add(Transform("odom", "base_link"))
    with pytest.raises(SimulationError, match="duplicate"):
        tree.add(Transform("map", "base_link"))
    cyclic = FrameTree()
    cyclic.add(Transform("a", "b"))
    with pytest.raises(SimulationError, match="cycle"):
        cyclic.add(Transform("b", "a"))
    validate_qos()


def test_contract_validation_rejects_invalid_perception():
    with pytest.raises(SimulationError):
        PerceptionContract(MessageHeader(0, "base_link", 1, "perception"), -1, 0, 1, 0).validate()
