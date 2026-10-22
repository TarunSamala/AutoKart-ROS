from __future__ import annotations

from dataclasses import replace
from math import pi
from pathlib import Path

import pytest
import yaml

from kart_sim.gazebo_follower import (
    AuthoritativeSnapshotBuffer,
    AuthoritativeVehicleSnapshot,
    ReadOnlyGazeboFollower,
    SensorDeliveryQueue,
    SensorQueueConfiguration,
    interpolate_snapshots,
    quaternion_from_rpy,
    rpy_from_quaternion,
    snapshot_to_gazebo,
    validate_follower_configuration,
)
from kart_sim.types import SimulationError


ROOT = Path(__file__).parents[2]
HASH = "a" * 64


def snapshot(time_s: float = 1.0, sequence: int = 1, epoch: int = 1) -> AuthoritativeVehicleSnapshot:
    return AuthoritativeVehicleSnapshot(
        time_s,
        sequence,
        1.0,
        2.0,
        0.1,
        0.01,
        -0.02,
        0.3,
        (3.0, 0.2, 0.0),
        (0.0, 0.0, 0.4),
        0.1,
        0.09,
        2.0,
        2.1,
        1.9,
        1.9,
        20.0,
        21.0,
        19.0,
        19.0,
        1.9,
        19.0,
        True,
        epoch,
        HASH,
    )


def test_snapshot_rejects_nonfinite_and_bad_hash() -> None:
    snapshot().validated()
    with pytest.raises(SimulationError, match="NaN"):
        replace(snapshot(), position_x_m=float("nan")).validated()
    with pytest.raises(SimulationError, match="SHA-256"):
        replace(snapshot(), configuration_hash="bad").validated()
    with pytest.raises(SimulationError, match="normalized"):
        replace(snapshot(), yaw_rad=4.0).validated()


def test_quaternion_round_trip_and_coordinate_joint_signs() -> None:
    rotation = quaternion_from_rpy(0.2, -0.1, 2.9)
    roll, pitch, yaw = rpy_from_quaternion(rotation)
    assert (roll, pitch, yaw) == pytest.approx((0.2, -0.1, 2.9), abs=1e-12)
    mapped = snapshot_to_gazebo(snapshot())
    assert mapped.pose_xyz_quaternion[:3] == (1.0, 2.0, 0.1)
    assert mapped.joint_positions_rad["front_left_steering_joint"] == 0.1
    assert mapped.joint_positions_rad["front_left_wheel_joint"] == -2.0
    assert mapped.joint_velocities_radps["rear_solid_axle_joint"] == -19.0


def test_interpolation_handles_yaw_wrap_without_extrapolation() -> None:
    left = replace(snapshot(1.0, 1), yaw_rad=pi - 0.1)
    right = replace(snapshot(1.01, 2), yaw_rad=-pi + 0.1, position_x_m=2.0)
    middle = interpolate_snapshots(left, right, 1.005, maximum_interval_s=0.02)
    assert abs(abs(middle.yaw_rad) - pi) < 1e-9
    assert middle.position_x_m == pytest.approx(1.5)
    with pytest.raises(SimulationError, match="extrapolate"):
        interpolate_snapshots(left, right, 1.02, maximum_interval_s=0.02)


def test_snapshot_buffer_enforces_sequence_epoch_and_staleness() -> None:
    buffer = AuthoritativeSnapshotBuffer(maximum_state_age_s=0.02)
    buffer.push(snapshot(1.0, 1))
    buffer.push(snapshot(1.01, 2))
    assert buffer.sample(1.005).position_x_m == pytest.approx(1.0)
    with pytest.raises(SimulationError, match="sequence"):
        buffer.push(snapshot(1.02, 2))
    with pytest.raises(SimulationError, match="stale"):
        buffer.sample(1.04)
    buffer.reset(2, HASH)
    assert buffer.depth == 0
    with pytest.raises(SimulationError, match="increase"):
        buffer.reset(2, HASH)


def test_sensor_queue_preserves_capture_timestamp_and_bounds() -> None:
    queue = SensorDeliveryQueue(SensorQueueConfiguration(
        "camera", "camera_optical_frame", 2,
        processing_latency_s=0.01,
        transport_latency_s=0.02,
        jitter_s=0.0,
    ), reset_epoch=1)
    queue.capture(sequence=1, capture_time_s=1.0, payload=b"rgb", encoding="rgb8", width=1, height=1)
    queue.capture(sequence=2, capture_time_s=1.1, payload=b"abc", encoding="rgb8", width=1, height=1)
    queue.capture(sequence=3, capture_time_s=1.2, payload=b"xyz", encoding="rgb8", width=1, height=1)
    assert queue.depth == 2
    assert queue.metrics.dropped_frames == 1
    delivered = queue.pop_due(1.3, 1)
    assert [frame.metadata.sequence for frame in delivered] == [2, 3]
    assert delivered[0].metadata.capture_time_s == 1.1
    assert delivered[0].metadata.delivery_time_s == pytest.approx(1.13)
    assert all(frame.metadata.payload_present for frame in delivered)


def test_sensor_reset_flushes_old_epoch_and_prevents_reorder() -> None:
    queue = SensorDeliveryQueue(SensorQueueConfiguration(
        "lidar", "lidar_link", 4, jitter_s=0.01, seed=4
    ), reset_epoch=1)
    queue.capture(sequence=1, capture_time_s=0.1, payload=b"x", encoding="ranges")
    queue.reset(2)
    assert queue.depth == 0
    assert queue.metrics.stale_frames == 1
    queue.capture(sequence=1, capture_time_s=0.0, payload=b"y", encoding="ranges")
    assert queue.pop_due(1.0, 2)[0].metadata.reset_epoch == 2


def test_pause_disconnect_and_reset_freeze_follower() -> None:
    buffer = AuthoritativeSnapshotBuffer()
    follower = ReadOnlyGazeboFollower(buffer)
    follower.ingest(snapshot())
    assert follower.update(1.0) is not None
    follower.pause()
    assert follower.update(1.01) == follower.last_applied
    follower.resume()
    follower.disconnect()
    assert follower.update(1.01) == follower.last_applied
    assert not follower.diagnostics.follower_valid
    follower.reset(2, HASH)
    assert follower.paused
    assert follower.last_applied is None

    delayed = ReadOnlyGazeboFollower(
        AuthoritativeSnapshotBuffer(maximum_state_age_s=0.02)
    )
    delayed.ingest(snapshot())
    applied = delayed.update(1.0)
    assert delayed.update(1.021) == applied
    assert delayed.diagnostics.follower_frozen
    assert not delayed.diagnostics.follower_valid
    assert delayed.diagnostics.critical_reason == (
        "authoritative follower state is stale"
    )


def test_configuration_rejects_gazebo_dynamics_and_actuation() -> None:
    scenario = yaml.safe_load(
        (ROOT / "configs/scenarios/gazebo_follower_full_lap.yaml").read_text(encoding="utf-8")
    )
    result = validate_follower_configuration(
        scenario, ROOT / "simulation/models/autokart/model.sdf"
    )
    assert result["double_physics_detected"] is False
    scenario["gazebo"]["role"] = "dynamics_authority"
    scenario["gazebo"]["vehicle_dynamics_authority"] = True
    with pytest.raises(SimulationError, match="visual_sensor_follower"):
        validate_follower_configuration(
            scenario, ROOT / "simulation/models/autokart/model.sdf"
        )
