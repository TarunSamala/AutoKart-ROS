"""Deterministic, one-way internal-physics to Gazebo follower contracts.

The classes in this module are transport neutral.  Native ROS/Gazebo adapters
may consume their immutable snapshots, but no type in this module exposes an
actuator-command path back into the plant.
"""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass, field, replace
from hashlib import sha256
from math import asin, atan2, cos, isfinite, pi, sin, sqrt
from pathlib import Path
from random import Random
from typing import Any, Iterable, Literal
from xml.etree import ElementTree as ET

import yaml

from .types import SimulationError


def normalize_angle(angle_rad: float) -> float:
    """Normalize a pose angle to ``[-pi, pi)``."""
    return (angle_rad + pi) % (2.0 * pi) - pi


def quaternion_from_rpy(roll_rad: float, pitch_rad: float, yaw_rad: float) -> tuple[float, float, float, float]:
    """Return a normalized ROS/Gazebo quaternion in x, y, z, w order."""
    cr, sr = cos(roll_rad / 2.0), sin(roll_rad / 2.0)
    cp, sp = cos(pitch_rad / 2.0), sin(pitch_rad / 2.0)
    cy, sy = cos(yaw_rad / 2.0), sin(yaw_rad / 2.0)
    quaternion = (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
    norm = sqrt(sum(value * value for value in quaternion))
    if norm < 1e-12:
        raise SimulationError("orientation produced a zero-length quaternion")
    return tuple(value / norm for value in quaternion)


def rpy_from_quaternion(rotation_xyzw: tuple[float, float, float, float]) -> tuple[float, float, float]:
    """Convert an x, y, z, w quaternion to normalized roll, pitch, yaw."""
    if len(rotation_xyzw) != 4 or not all(isfinite(value) for value in rotation_xyzw):
        raise SimulationError("quaternion must contain four finite values")
    norm = sqrt(sum(value * value for value in rotation_xyzw))
    if norm < 1e-12:
        raise SimulationError("quaternion has zero length")
    x, y, z, w = (value / norm for value in rotation_xyzw)
    roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = asin(pitch_term)
    yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return normalize_angle(roll), normalize_angle(pitch), normalize_angle(yaw)


def configuration_digest(paths: Iterable[str | Path]) -> str:
    """Hash ordered configuration bytes for snapshot/run consistency checks."""
    digest = sha256()
    for source in paths:
        path = Path(source)
        digest.update(str(path.as_posix()).encode("utf-8"))
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


@dataclass(frozen=True)
class AuthoritativeVehicleSnapshot:
    simulation_time_s: float
    sequence: int
    position_x_m: float
    position_y_m: float
    position_z_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    linear_velocity_body_mps: tuple[float, float, float]
    angular_velocity_body_radps: tuple[float, float, float]
    front_left_steering_rad: float
    front_right_steering_rad: float
    front_left_wheel_rotation_rad: float
    front_right_wheel_rotation_rad: float
    rear_left_wheel_rotation_rad: float
    rear_right_wheel_rotation_rad: float
    front_left_wheel_speed_radps: float
    front_right_wheel_speed_radps: float
    rear_left_wheel_speed_radps: float
    rear_right_wheel_speed_radps: float
    rear_axle_rotation_rad: float
    rear_axle_speed_radps: float
    valid: bool
    reset_epoch: int
    configuration_hash: str

    def validated(self) -> "AuthoritativeVehicleSnapshot":
        scalar_values = (
            self.simulation_time_s,
            self.position_x_m,
            self.position_y_m,
            self.position_z_m,
            self.roll_rad,
            self.pitch_rad,
            self.yaw_rad,
            self.front_left_steering_rad,
            self.front_right_steering_rad,
            self.front_left_wheel_rotation_rad,
            self.front_right_wheel_rotation_rad,
            self.rear_left_wheel_rotation_rad,
            self.rear_right_wheel_rotation_rad,
            self.front_left_wheel_speed_radps,
            self.front_right_wheel_speed_radps,
            self.rear_left_wheel_speed_radps,
            self.rear_right_wheel_speed_radps,
            self.rear_axle_rotation_rad,
            self.rear_axle_speed_radps,
            *self.linear_velocity_body_mps,
            *self.angular_velocity_body_radps,
        )
        if not all(isfinite(value) for value in scalar_values):
            raise SimulationError("authoritative snapshot contains NaN or infinity")
        if self.simulation_time_s < 0.0 or self.sequence < 0 or self.reset_epoch < 0:
            raise SimulationError("snapshot time, sequence, and reset epoch must be non-negative")
        if len(self.linear_velocity_body_mps) != 3 or len(self.angular_velocity_body_radps) != 3:
            raise SimulationError("snapshot velocity vectors must contain three components")
        if len(self.configuration_hash) != 64 or any(character not in "0123456789abcdef" for character in self.configuration_hash):
            raise SimulationError("snapshot configuration hash must be lower-case SHA-256")
        for angle in (self.roll_rad, self.pitch_rad, self.yaw_rad):
            if not -pi - 1e-12 <= angle <= pi + 1e-12:
                raise SimulationError("snapshot pose angles must be normalized")
        return self


@dataclass
class WheelRotationAccumulator:
    """Integrate measured wheel speed solely for follower joint visualization."""

    front_left_rad: float = 0.0
    front_right_rad: float = 0.0
    rear_left_rad: float = 0.0
    rear_right_rad: float = 0.0
    rear_axle_rad: float = 0.0

    def advance(
        self,
        wheel_speed_radps: tuple[float, float, float, float],
        rear_axle_speed_radps: float,
        dt_s: float,
    ) -> None:
        if dt_s < 0.0 or not all(isfinite(value) for value in (*wheel_speed_radps, rear_axle_speed_radps, dt_s)):
            raise SimulationError("wheel rotation integration received invalid values")
        self.front_left_rad += wheel_speed_radps[0] * dt_s
        self.front_right_rad += wheel_speed_radps[1] * dt_s
        self.rear_left_rad += wheel_speed_radps[2] * dt_s
        self.rear_right_rad += wheel_speed_radps[3] * dt_s
        self.rear_axle_rad += rear_axle_speed_radps * dt_s

    def reset(self) -> None:
        self.front_left_rad = self.front_right_rad = 0.0
        self.rear_left_rad = self.rear_right_rad = self.rear_axle_rad = 0.0


class AuthoritativeSnapshotFactory:
    """Build snapshots from completed, immutable telemetry records."""

    def __init__(self, configuration_hash: str, reset_epoch: int = 0) -> None:
        if len(configuration_hash) != 64:
            raise SimulationError("snapshot factory requires a SHA-256 configuration hash")
        self.configuration_hash = configuration_hash
        self.reset_epoch = reset_epoch
        self.sequence = 0
        self.rotations = WheelRotationAccumulator()
        self._last_time_s: float | None = None

    def reset(self, reset_epoch: int) -> None:
        if reset_epoch <= self.reset_epoch:
            raise SimulationError("snapshot reset epoch must increase")
        self.reset_epoch = reset_epoch
        self.sequence = 0
        self.rotations.reset()
        self._last_time_s = None

    def from_record(self, record: dict[str, Any]) -> AuthoritativeVehicleSnapshot:
        state = record["state"]
        wheels = record["subsystems"]["four_wheel"]["wheels"]
        diagnostics = record["subsystems"]["four_wheel"]["diagnostics"]
        time_s = float(state["time_s"])
        dt_s = 0.0 if self._last_time_s is None else time_s - self._last_time_s
        if dt_s < -1e-12:
            raise SimulationError("authoritative telemetry time moved backwards")
        speeds = tuple(float(wheels[name]["angular_speed_radps"]) for name in (
            "front_left", "front_right", "rear_left", "rear_right"
        ))
        rear_speed = float(diagnostics["rear_axle_angular_speed_radps"])
        self.rotations.advance(speeds, rear_speed, max(0.0, dt_s))
        self.sequence += 1
        self._last_time_s = time_s
        snapshot = AuthoritativeVehicleSnapshot(
            simulation_time_s=time_s,
            sequence=self.sequence,
            position_x_m=float(state["x_m"]),
            position_y_m=float(state["y_m"]),
            position_z_m=float(state.get("z_m", 0.0)),
            roll_rad=normalize_angle(float(state.get("roll_rad", 0.0))),
            pitch_rad=normalize_angle(float(state.get("pitch_rad", 0.0))),
            yaw_rad=normalize_angle(float(state["yaw_rad"])),
            linear_velocity_body_mps=(
                float(state["u_mps"]),
                float(state["v_mps"]),
                0.0,
            ),
            angular_velocity_body_radps=(0.0, 0.0, float(state["yaw_rate_radps"])),
            front_left_steering_rad=float(wheels["front_left"]["steering_angle_rad"]),
            front_right_steering_rad=float(wheels["front_right"]["steering_angle_rad"]),
            front_left_wheel_rotation_rad=self.rotations.front_left_rad,
            front_right_wheel_rotation_rad=self.rotations.front_right_rad,
            rear_left_wheel_rotation_rad=self.rotations.rear_left_rad,
            rear_right_wheel_rotation_rad=self.rotations.rear_right_rad,
            front_left_wheel_speed_radps=speeds[0],
            front_right_wheel_speed_radps=speeds[1],
            rear_left_wheel_speed_radps=speeds[2],
            rear_right_wheel_speed_radps=speeds[3],
            rear_axle_rotation_rad=self.rotations.rear_axle_rad,
            rear_axle_speed_radps=rear_speed,
            valid=True,
            reset_epoch=self.reset_epoch,
            configuration_hash=self.configuration_hash,
        )
        return snapshot.validated()

    def from_plant(
        self,
        *,
        state: Any,
        wheels: dict[str, Any],
        rear_axle_speed_radps: float,
    ) -> AuthoritativeVehicleSnapshot:
        """Build directly from a completed plant step without telemetry copies.

        ``state`` and wheel objects are read synchronously by the physics-step
        observer.  Their values are copied into the frozen snapshot before the
        next plant step can begin.
        """
        self.advance_from_plant(
            state=state,
            wheels=wheels,
            rear_axle_speed_radps=rear_axle_speed_radps,
        )
        return self.snapshot_from_advanced_plant(
            state=state,
            wheels=wheels,
            rear_axle_speed_radps=rear_axle_speed_radps,
        )

    def advance_from_plant(
        self,
        *,
        state: Any,
        wheels: dict[str, Any],
        rear_axle_speed_radps: float,
    ) -> None:
        """Advance sequence and measured wheel rotations for one plant step.

        This deliberately performs no snapshot allocation.  High-rate physics
        recorders can advance the exact joint state at the plant rate and only
        allocate an immutable transport snapshot at the configured follower
        rate.
        """
        time_s = float(state.time_s)
        dt_s = 0.0 if self._last_time_s is None else time_s - self._last_time_s
        if dt_s < -1e-12:
            raise SimulationError("authoritative plant time moved backwards")
        speeds = tuple(float(wheels[name].angular_speed_radps) for name in (
            "front_left", "front_right", "rear_left", "rear_right"
        ))
        rear_speed = float(rear_axle_speed_radps)
        self.rotations.advance(speeds, rear_speed, max(0.0, dt_s))
        self.sequence += 1
        self._last_time_s = time_s

    def snapshot_from_advanced_plant(
        self,
        *,
        state: Any,
        wheels: dict[str, Any],
        rear_axle_speed_radps: float,
    ) -> AuthoritativeVehicleSnapshot:
        """Copy the already-advanced plant state into one frozen snapshot."""
        time_s = float(state.time_s)
        if self._last_time_s is None or abs(time_s - self._last_time_s) > 1e-12:
            raise SimulationError(
                "snapshot requested before advancing the current plant state"
            )
        ordered = tuple(wheels[name] for name in (
            "front_left", "front_right", "rear_left", "rear_right"
        ))
        speeds = tuple(float(wheel.angular_speed_radps) for wheel in ordered)
        rear_speed = float(rear_axle_speed_radps)
        return AuthoritativeVehicleSnapshot(
            simulation_time_s=time_s,
            sequence=self.sequence,
            position_x_m=float(state.x_m),
            position_y_m=float(state.y_m),
            position_z_m=float(getattr(state, "z_m", 0.0)),
            roll_rad=normalize_angle(float(getattr(state, "roll_rad", 0.0))),
            pitch_rad=normalize_angle(float(getattr(state, "pitch_rad", 0.0))),
            yaw_rad=normalize_angle(float(state.yaw_rad)),
            linear_velocity_body_mps=(
                float(state.u_mps),
                float(state.v_mps),
                0.0,
            ),
            angular_velocity_body_radps=(
                0.0,
                0.0,
                float(state.yaw_rate_radps),
            ),
            front_left_steering_rad=float(ordered[0].steering_angle_rad),
            front_right_steering_rad=float(ordered[1].steering_angle_rad),
            front_left_wheel_rotation_rad=self.rotations.front_left_rad,
            front_right_wheel_rotation_rad=self.rotations.front_right_rad,
            rear_left_wheel_rotation_rad=self.rotations.rear_left_rad,
            rear_right_wheel_rotation_rad=self.rotations.rear_right_rad,
            front_left_wheel_speed_radps=speeds[0],
            front_right_wheel_speed_radps=speeds[1],
            rear_left_wheel_speed_radps=speeds[2],
            rear_right_wheel_speed_radps=speeds[3],
            rear_axle_rotation_rad=self.rotations.rear_axle_rad,
            rear_axle_speed_radps=rear_speed,
            valid=True,
            reset_epoch=self.reset_epoch,
            configuration_hash=self.configuration_hash,
        ).validated()


def _lerp(left: float, right: float, fraction: float) -> float:
    return left + (right - left) * fraction


def _lerp_angle(left: float, right: float, fraction: float) -> float:
    return normalize_angle(left + normalize_angle(right - left) * fraction)


def interpolate_snapshots(
    left: AuthoritativeVehicleSnapshot,
    right: AuthoritativeVehicleSnapshot,
    target_time_s: float,
    *,
    maximum_interval_s: float,
) -> AuthoritativeVehicleSnapshot:
    """Time-consistently interpolate two snapshots without extrapolation."""
    left.validated()
    right.validated()
    if left.reset_epoch != right.reset_epoch or left.configuration_hash != right.configuration_hash:
        raise SimulationError("cannot interpolate across reset/configuration boundaries")
    interval = right.simulation_time_s - left.simulation_time_s
    if interval <= 0.0 or interval > maximum_interval_s:
        raise SimulationError("snapshot interpolation interval is invalid or too large")
    if not left.simulation_time_s <= target_time_s <= right.simulation_time_s:
        raise SimulationError("snapshot interpolation cannot extrapolate")
    fraction = (target_time_s - left.simulation_time_s) / interval
    scalar_linear = {
        field_name: _lerp(getattr(left, field_name), getattr(right, field_name), fraction)
        for field_name in (
            "position_x_m", "position_y_m", "position_z_m",
            "front_left_steering_rad", "front_right_steering_rad",
            "front_left_wheel_rotation_rad", "front_right_wheel_rotation_rad",
            "rear_left_wheel_rotation_rad", "rear_right_wheel_rotation_rad",
            "front_left_wheel_speed_radps", "front_right_wheel_speed_radps",
            "rear_left_wheel_speed_radps", "rear_right_wheel_speed_radps",
            "rear_axle_rotation_rad", "rear_axle_speed_radps",
        )
    }
    return AuthoritativeVehicleSnapshot(
        simulation_time_s=target_time_s,
        sequence=right.sequence,
        roll_rad=_lerp_angle(left.roll_rad, right.roll_rad, fraction),
        pitch_rad=_lerp_angle(left.pitch_rad, right.pitch_rad, fraction),
        yaw_rad=_lerp_angle(left.yaw_rad, right.yaw_rad, fraction),
        linear_velocity_body_mps=tuple(
            _lerp(a, b, fraction) for a, b in zip(left.linear_velocity_body_mps, right.linear_velocity_body_mps)
        ),
        angular_velocity_body_radps=tuple(
            _lerp(a, b, fraction) for a, b in zip(left.angular_velocity_body_radps, right.angular_velocity_body_radps)
        ),
        valid=left.valid and right.valid,
        reset_epoch=left.reset_epoch,
        configuration_hash=left.configuration_hash,
        **scalar_linear,
    ).validated()


@dataclass
class AuthoritativeSnapshotBuffer:
    """Bounded monotonic bridge queue with epoch and staleness protection."""

    maximum_size: int = 256
    maximum_interpolation_s: float = 0.02
    maximum_state_age_s: float = 0.02
    _snapshots: deque[AuthoritativeVehicleSnapshot] = field(default_factory=deque)
    configuration_hash: str | None = None
    reset_epoch: int | None = None
    dropped_snapshots: int = 0

    def __post_init__(self) -> None:
        if self.maximum_size < 2 or self.maximum_interpolation_s <= 0.0 or self.maximum_state_age_s <= 0.0:
            raise SimulationError("snapshot buffer limits must be positive")

    def push(self, snapshot: AuthoritativeVehicleSnapshot) -> None:
        snapshot.validated()
        if not snapshot.valid:
            raise SimulationError("invalid authoritative snapshot cannot enter follower queue")
        if self.configuration_hash is None:
            self.configuration_hash = snapshot.configuration_hash
            self.reset_epoch = snapshot.reset_epoch
        if snapshot.configuration_hash != self.configuration_hash:
            raise SimulationError("snapshot configuration changed during a run")
        if snapshot.reset_epoch != self.reset_epoch:
            raise SimulationError("snapshot epoch changed without explicit follower reset")
        if self._snapshots:
            previous = self._snapshots[-1]
            if snapshot.sequence <= previous.sequence:
                raise SimulationError("snapshot sequence must increase within an epoch")
            if snapshot.simulation_time_s <= previous.simulation_time_s:
                raise SimulationError("snapshot simulation time must increase within an epoch")
        if len(self._snapshots) >= self.maximum_size:
            self._snapshots.popleft()
            self.dropped_snapshots += 1
        self._snapshots.append(snapshot)

    def sample(self, target_time_s: float) -> AuthoritativeVehicleSnapshot:
        if not self._snapshots:
            raise SimulationError("follower has no authoritative state")
        if target_time_s < self._snapshots[0].simulation_time_s:
            raise SimulationError("requested follower time predates buffered authority state")
        # Once the requested time has passed a bracket, retain only the newest
        # state at/before the target. This keeps the live queue bounded by rate
        # mismatch rather than total run duration.
        while (
            len(self._snapshots) >= 2 and
            self._snapshots[1].simulation_time_s <= target_time_s + 1e-12
        ):
            self._snapshots.popleft()
        snapshots = tuple(self._snapshots)
        for left, right in zip(snapshots, snapshots[1:]):
            if left.simulation_time_s <= target_time_s <= right.simulation_time_s:
                if abs(target_time_s - left.simulation_time_s) <= 1e-12:
                    return left
                if abs(target_time_s - right.simulation_time_s) <= 1e-12:
                    return right
                return interpolate_snapshots(
                    left,
                    right,
                    target_time_s,
                    maximum_interval_s=self.maximum_interpolation_s,
                )
        latest = snapshots[-1]
        state_age_s = target_time_s - latest.simulation_time_s
        if state_age_s > self.maximum_state_age_s + 1e-12:
            raise SimulationError("authoritative follower state is stale")
        # Holding the latest measured pose for a bounded age is allowed.
        return latest

    def reset(self, reset_epoch: int, configuration_hash: str) -> None:
        if self.reset_epoch is not None and reset_epoch <= self.reset_epoch:
            raise SimulationError("follower reset epoch must increase")
        if len(configuration_hash) != 64:
            raise SimulationError("follower reset requires configuration SHA-256")
        self._snapshots.clear()
        self.configuration_hash = configuration_hash
        self.reset_epoch = reset_epoch

    @property
    def depth(self) -> int:
        return len(self._snapshots)


@dataclass(frozen=True)
class GazeboMappedState:
    simulation_time_s: float
    reset_epoch: int
    sequence: int
    pose_xyz_quaternion: tuple[float, float, float, float, float, float, float]
    linear_velocity_world_mps: tuple[float, float, float]
    angular_velocity_world_radps: tuple[float, float, float]
    joint_positions_rad: dict[str, float]
    joint_velocities_radps: dict[str, float]


def snapshot_to_gazebo(snapshot: AuthoritativeVehicleSnapshot) -> GazeboMappedState:
    """Map internal ENU/base coordinates to Gazebo ENU and SDF joint signs."""
    snapshot.validated()
    quaternion = quaternion_from_rpy(snapshot.roll_rad, snapshot.pitch_rad, snapshot.yaw_rad)
    u_mps, v_mps, w_mps = snapshot.linear_velocity_body_mps
    cy, sy = cos(snapshot.yaw_rad), sin(snapshot.yaw_rad)
    world_velocity = (cy * u_mps - sy * v_mps, sy * u_mps + cy * v_mps, w_mps)
    # Internal wheel speed is positive for forward travel.  SDF wheel axes are
    # +Y, for which forward rolling is negative by the right-hand rule.
    joints = {
        "front_left_steering_joint": snapshot.front_left_steering_rad,
        "front_right_steering_joint": snapshot.front_right_steering_rad,
        "front_left_wheel_joint": -snapshot.front_left_wheel_rotation_rad,
        "front_right_wheel_joint": -snapshot.front_right_wheel_rotation_rad,
        "rear_solid_axle_joint": -snapshot.rear_axle_rotation_rad,
    }
    velocities = {
        "front_left_wheel_joint": -snapshot.front_left_wheel_speed_radps,
        "front_right_wheel_joint": -snapshot.front_right_wheel_speed_radps,
        "rear_solid_axle_joint": -snapshot.rear_axle_speed_radps,
    }
    return GazeboMappedState(
        snapshot.simulation_time_s,
        snapshot.reset_epoch,
        snapshot.sequence,
        (
            snapshot.position_x_m,
            snapshot.position_y_m,
            snapshot.position_z_m,
            *quaternion,
        ),
        world_velocity,
        snapshot.angular_velocity_body_radps,
        joints,
        velocities,
    )


@dataclass(frozen=True)
class FollowerAcceptanceThresholds:
    maximum_position_error_m: float = 0.005
    maximum_orientation_error_rad: float = 0.005
    maximum_steering_joint_error_rad: float = 0.005
    maximum_wheel_rotation_error_rad: float = 0.02
    maximum_clock_offset_s: float = 0.002
    maximum_state_age_s: float = 0.02

    def validate(self) -> None:
        if not all(isfinite(value) and value > 0.0 for value in asdict(self).values()):
            raise SimulationError("follower acceptance thresholds must be finite and positive")


def load_follower_thresholds(path: str | Path) -> FollowerAcceptanceThresholds:
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    root = payload["gazebo_follower_acceptance"]
    thresholds = FollowerAcceptanceThresholds(**{
        key: float(value) for key, value in root.items() if key in FollowerAcceptanceThresholds.__annotations__
    })
    thresholds.validate()
    return thresholds


@dataclass
class TimeSyncMonitor:
    thresholds: FollowerAcceptanceThresholds = field(default_factory=FollowerAcceptanceThresholds)
    maximum_clock_offset_s: float = 0.0
    samples: int = 0
    violations: int = 0

    def observe(self, internal_time_s: float, gazebo_time_s: float) -> float:
        if not all(isfinite(value) and value >= 0.0 for value in (internal_time_s, gazebo_time_s)):
            raise SimulationError("time-sync sample is invalid")
        offset = gazebo_time_s - internal_time_s
        self.maximum_clock_offset_s = max(self.maximum_clock_offset_s, abs(offset))
        self.samples += 1
        if abs(offset) > self.thresholds.maximum_clock_offset_s:
            self.violations += 1
        return offset


@dataclass(frozen=True)
class SensorFrameMetadata:
    sensor_id: str
    frame_id: str
    sequence: int
    reset_epoch: int
    capture_time_s: float
    processing_completion_time_s: float
    delivery_time_s: float
    calibration_version: str
    quality: str
    fault_state: str
    payload_present: bool
    payload_size: int
    encoding: str
    width: int = 0
    height: int = 0

    def validate(self) -> None:
        if not self.sensor_id or not self.frame_id or self.sequence < 0 or self.reset_epoch < 0:
            raise SimulationError("sensor metadata has invalid identity")
        timestamps = (self.capture_time_s, self.processing_completion_time_s, self.delivery_time_s)
        if not all(isfinite(value) and value >= 0.0 for value in timestamps):
            raise SimulationError("sensor metadata contains invalid timestamps")
        if not self.capture_time_s <= self.processing_completion_time_s <= self.delivery_time_s:
            raise SimulationError("sensor timing order is invalid")
        if self.payload_present != (self.payload_size > 0):
            raise SimulationError("sensor payload presence and size disagree")
        if self.width < 0 or self.height < 0:
            raise SimulationError("sensor dimensions cannot be negative")


@dataclass(frozen=True)
class SensorFrame:
    metadata: SensorFrameMetadata
    payload: bytes

    def validate(self) -> None:
        self.metadata.validate()
        if len(self.payload) != self.metadata.payload_size:
            raise SimulationError("sensor payload size does not match metadata")


@dataclass(frozen=True)
class SensorQueueConfiguration:
    sensor_id: str
    frame_id: str
    capacity: int
    processing_latency_s: float = 0.0
    transport_latency_s: float = 0.0
    jitter_s: float = 0.0
    dropout_probability: float = 0.0
    overflow_policy: Literal["drop_oldest", "drop_newest"] = "drop_oldest"
    calibration_version: str = "SIM-CAL-1"
    seed: int = 1

    def validate(self) -> None:
        if not self.sensor_id or not self.frame_id or self.capacity <= 0:
            raise SimulationError("sensor queue identity/capacity is invalid")
        if min(self.processing_latency_s, self.transport_latency_s, self.jitter_s) < 0.0:
            raise SimulationError("sensor queue latencies cannot be negative")
        if not 0.0 <= self.dropout_probability <= 1.0:
            raise SimulationError("sensor dropout probability must be in [0, 1]")


@dataclass
class SensorQueueMetrics:
    captured_frames: int = 0
    delivered_frames: int = 0
    dropped_frames: int = 0
    stale_frames: int = 0
    maximum_queue_depth: int = 0
    maximum_latency_s: float = 0.0
    delivery_latencies_s: list[float] = field(default_factory=list)


class SensorDeliveryQueue:
    """Bounded, non-blocking, capture-time-preserving per-sensor queue."""

    def __init__(self, configuration: SensorQueueConfiguration, *, reset_epoch: int = 0) -> None:
        configuration.validate()
        self.configuration = configuration
        self.reset_epoch = reset_epoch
        self.metrics = SensorQueueMetrics()
        self._queue: deque[SensorFrame] = deque()
        self._random = Random(configuration.seed)
        self._last_capture_time_s: float | None = None
        self._last_delivery_time_s: float = 0.0

    def capture(
        self,
        *,
        sequence: int,
        capture_time_s: float,
        payload: bytes,
        encoding: str,
        width: int = 0,
        height: int = 0,
        quality: str = "VALID",
        fault_state: str = "NONE",
    ) -> SensorFrame | None:
        if self._last_capture_time_s is not None and capture_time_s <= self._last_capture_time_s:
            raise SimulationError("sensor capture timestamps must increase within an epoch")
        self._last_capture_time_s = capture_time_s
        self.metrics.captured_frames += 1
        if self._random.random() < self.configuration.dropout_probability or fault_state == "DROPOUT":
            self.metrics.dropped_frames += 1
            return None
        jitter = self._random.uniform(-self.configuration.jitter_s, self.configuration.jitter_s)
        processing = capture_time_s + self.configuration.processing_latency_s
        requested_delivery = processing + self.configuration.transport_latency_s + jitter
        delivery = max(processing, self._last_delivery_time_s, requested_delivery)
        self._last_delivery_time_s = delivery
        metadata = SensorFrameMetadata(
            self.configuration.sensor_id,
            self.configuration.frame_id,
            sequence,
            self.reset_epoch,
            capture_time_s,
            processing,
            delivery,
            self.configuration.calibration_version,
            quality,
            fault_state,
            bool(payload),
            len(payload),
            encoding,
            width,
            height,
        )
        frame = SensorFrame(metadata, bytes(payload))
        frame.validate()
        if len(self._queue) >= self.configuration.capacity:
            self.metrics.dropped_frames += 1
            if self.configuration.overflow_policy == "drop_newest":
                return None
            self._queue.popleft()
        self._queue.append(frame)
        self.metrics.maximum_queue_depth = max(self.metrics.maximum_queue_depth, len(self._queue))
        return frame

    def pop_due(self, now_s: float, reset_epoch: int) -> tuple[SensorFrame, ...]:
        due: list[SensorFrame] = []
        while self._queue:
            frame = self._queue[0]
            if frame.metadata.reset_epoch != reset_epoch:
                self._queue.popleft()
                self.metrics.stale_frames += 1
                continue
            if frame.metadata.delivery_time_s > now_s + 1e-12:
                break
            frame = self._queue.popleft()
            latency = frame.metadata.delivery_time_s - frame.metadata.capture_time_s
            self.metrics.delivered_frames += 1
            self.metrics.maximum_latency_s = max(self.metrics.maximum_latency_s, latency)
            self.metrics.delivery_latencies_s.append(latency)
            due.append(frame)
        return tuple(due)

    def reset(self, reset_epoch: int) -> None:
        if reset_epoch <= self.reset_epoch:
            raise SimulationError("sensor queue reset epoch must increase")
        self.metrics.stale_frames += len(self._queue)
        self._queue.clear()
        self.reset_epoch = reset_epoch
        self._last_capture_time_s = None
        self._last_delivery_time_s = 0.0

    @property
    def depth(self) -> int:
        return len(self._queue)


@dataclass
class FollowerDiagnostics:
    physics_authority: str = "internal"
    gazebo_role: str = "visual_sensor_follower"
    double_physics_detected: bool = False
    pose_source: str = "authoritative_internal"
    joint_source: str = "authoritative_internal"
    follower_valid: bool = False
    follower_frozen: bool = True
    last_sequence: int = -1
    last_state_time_s: float = 0.0
    state_age_s: float = 0.0
    reset_epoch: int = 0
    critical_reason: str = ""
    gazebo_contacts_visual_only: int = 0

    def critical(self, reason: str, *, double_physics: bool = False) -> None:
        self.follower_valid = False
        self.follower_frozen = True
        self.critical_reason = reason
        self.double_physics_detected = self.double_physics_detected or double_physics


class ReadOnlyGazeboFollower:
    """Apply snapshots to a kinematic mirror; there is deliberately no command API."""

    def __init__(self, snapshot_buffer: AuthoritativeSnapshotBuffer) -> None:
        self.buffer = snapshot_buffer
        self.diagnostics = FollowerDiagnostics()
        self.last_applied: GazeboMappedState | None = None
        self.paused = False
        self.connected = True

    def ingest(self, snapshot: AuthoritativeVehicleSnapshot) -> None:
        if not self.connected:
            raise SimulationError("Gazebo follower bridge is disconnected")
        self.buffer.push(snapshot)

    def update(self, gazebo_time_s: float) -> GazeboMappedState | None:
        if self.paused or not self.connected:
            self.diagnostics.follower_frozen = True
            return self.last_applied
        try:
            snapshot = self.buffer.sample(gazebo_time_s)
        except SimulationError as exc:
            self.diagnostics.critical(str(exc))
            return self.last_applied
        mapped = snapshot_to_gazebo(snapshot)
        self.last_applied = mapped
        self.diagnostics.follower_valid = True
        self.diagnostics.follower_frozen = False
        self.diagnostics.last_sequence = snapshot.sequence
        self.diagnostics.last_state_time_s = snapshot.simulation_time_s
        self.diagnostics.state_age_s = max(0.0, gazebo_time_s - snapshot.simulation_time_s)
        self.diagnostics.reset_epoch = snapshot.reset_epoch
        self.diagnostics.critical_reason = ""
        return mapped

    def pause(self) -> None:
        self.paused = True
        self.diagnostics.follower_frozen = True

    def resume(self) -> None:
        if not self.connected:
            raise SimulationError("cannot resume a disconnected follower")
        self.paused = False

    def disconnect(self) -> None:
        self.connected = False
        self.diagnostics.critical("follower bridge disconnected")

    def reset(self, reset_epoch: int, configuration_hash: str) -> None:
        self.buffer.reset(reset_epoch, configuration_hash)
        self.last_applied = None
        self.paused = True
        self.connected = True
        contacts = self.diagnostics.gazebo_contacts_visual_only
        self.diagnostics = FollowerDiagnostics(reset_epoch=reset_epoch)
        self.diagnostics.gazebo_contacts_visual_only = contacts

    def record_visual_contact(self) -> None:
        self.diagnostics.gazebo_contacts_visual_only += 1


FORBIDDEN_DYNAMICS_PLUGIN_TOKENS = (
    "diff_drive",
    "ackermann",
    "motor",
    "steering_controller",
    "brake_controller",
    "gazebo_ros2_control",
    "vehicle_dynamics",
)


def validate_follower_configuration(configuration: dict[str, Any], model_sdf: str | Path) -> dict[str, Any]:
    """Reject every known internal/Gazebo dual-authority configuration."""
    simulation = configuration.get("simulation", {})
    gazebo = configuration.get("gazebo", {})
    backend = str(simulation.get("physics_backend", ""))
    role = str(gazebo.get("role", ""))
    comparison_mode = bool(configuration.get("comparison_mode", {}).get("enabled", False))
    if backend != "internal":
        raise SimulationError("this follower task requires internal physics authority")
    if role not in {"visual_sensor_follower", "visual_follower"}:
        if role == "dynamics_authority" and comparison_mode:
            raise SimulationError("comparison mode is not implemented for this follower")
        raise SimulationError("internal physics requires Gazebo visual_sensor_follower role")
    if bool(gazebo.get("vehicle_dynamics_authority", False)):
        raise SimulationError("Gazebo vehicle dynamics authority must be false in follower mode")
    if bool(gazebo.get("actuation_authority", False)):
        raise SimulationError("Gazebo actuation authority must be disabled in follower mode")
    root = ET.parse(model_sdf).getroot()
    model = root.find(".//model")
    if model is None:
        raise SimulationError("Gazebo follower SDF has no model")
    static = model.findtext("static", default="false").strip().lower()
    if static != "true":
        raise SimulationError("Gazebo follower chassis must be static/kinematic")
    plugins: list[str] = []
    for plugin in model.findall(".//plugin"):
        descriptor = f"{plugin.get('name', '')} {plugin.get('filename', '')}".lower()
        plugins.append(descriptor.strip())
        if any(token in descriptor for token in FORBIDDEN_DYNAMICS_PLUGIN_TOKENS):
            raise SimulationError(f"prohibited Gazebo dynamics plugin in follower model: {descriptor}")
    return {
        "physics_authority": "internal",
        "gazebo_role": "visual_sensor_follower",
        "vehicle_dynamics_authority": False,
        "gazebo_actuation_authority": False,
        "model_static": True,
        "plugins": tuple(plugins),
        "double_physics_detected": False,
    }
