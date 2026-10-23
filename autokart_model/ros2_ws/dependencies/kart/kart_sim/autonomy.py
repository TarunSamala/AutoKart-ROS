"""ROS-independent autonomy contracts and deterministic baseline controllers.

This module deliberately stops at *high-level* :class:`MotionCommand` output.
It never produces a :class:`~kart_sim.types.VehicleCommand`, so a controller or
ROS adapter cannot bypass VCU integrity, freshness, state, or safety-envelope
checks.  ROS 2 adapters convert their transport messages into these contracts.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import atan2, cos, hypot, isfinite, sin
from typing import Iterable

from .types import PlanarKartState, SimulationError
from .vcu import MotionCommand


def normalize_angle(angle_rad: float) -> float:
    """Return an angle in ``[-pi, pi)`` without a dependency on NumPy."""
    from math import pi

    return (angle_rad + pi) % (2.0 * pi) - pi


@dataclass(frozen=True)
class MessageHeader:
    """Transport-neutral metadata retained when a message is bridged to ROS."""

    stamp_s: float
    frame_id: str
    sequence: int
    source: str
    valid: bool = True
    quality: str = "VALID"
    configuration_hash: str = ""
    calibration_version: str = ""

    def validate(self) -> None:
        if not isfinite(self.stamp_s) or self.stamp_s < 0.0:
            raise SimulationError("message stamp must be a finite non-negative time")
        if not self.frame_id or not self.source or self.sequence < 0:
            raise SimulationError("message header has an invalid frame, source, or sequence")


@dataclass(frozen=True)
class SensorEnvelope:
    """Sensor payload metadata with distinct capture and delivery times."""

    header: MessageHeader
    capture_start_s: float
    capture_end_s: float
    measurement_timestamp_s: float
    delivery_timestamp_s: float
    payload: object

    def validate(self) -> None:
        self.header.validate()
        values = (self.capture_start_s, self.capture_end_s, self.measurement_timestamp_s, self.delivery_timestamp_s)
        if not all(isfinite(value) and value >= 0.0 for value in values):
            raise SimulationError("sensor envelope contains invalid timestamps")
        if self.capture_end_s < self.capture_start_s:
            raise SimulationError("sensor capture end precedes capture start")
        if not self.capture_start_s <= self.measurement_timestamp_s <= self.capture_end_s:
            raise SimulationError("sensor measurement timestamp must lie in its capture interval")
        if self.delivery_timestamp_s < self.measurement_timestamp_s:
            raise SimulationError("sensor delivery cannot precede its measurement")


@dataclass(frozen=True)
class TrajectoryPoint:
    """A time-parameterized map-frame target expressed in SI units."""

    time_s: float
    x_m: float
    y_m: float
    z_m: float = 0.0
    heading_rad: float = 0.0
    speed_mps: float = 0.0
    acceleration_mps2: float = 0.0
    curvature_1pm: float = 0.0
    jerk_mps3: float | None = None
    track_s_m: float | None = None

    def validate(self, *, maximum_speed_mps: float, maximum_acceleration_mps2: float, maximum_curvature_1pm: float) -> None:
        values = (self.time_s, self.x_m, self.y_m, self.z_m, self.heading_rad, self.speed_mps,
                  self.acceleration_mps2, self.curvature_1pm)
        if self.jerk_mps3 is not None:
            values += (self.jerk_mps3,)
        if self.track_s_m is not None:
            values += (self.track_s_m,)
        if not all(isfinite(value) for value in values):
            raise SimulationError("trajectory point contains NaN or infinity")
        if self.speed_mps < 0.0 or self.speed_mps > maximum_speed_mps:
            raise SimulationError("trajectory speed is outside its configured range")
        if abs(self.acceleration_mps2) > maximum_acceleration_mps2:
            raise SimulationError("trajectory acceleration is outside its configured range")
        if abs(self.curvature_1pm) > maximum_curvature_1pm:
            raise SimulationError("trajectory curvature is outside its configured range")


@dataclass(frozen=True)
class Trajectory:
    """Validated temporal trajectory, not an ambiguous spatial-only path."""

    header: MessageHeader
    points: tuple[TrajectoryPoint, ...]
    valid_until_s: float

    def validate(
        self,
        *,
        current_time_s: float,
        expected_frame: str = "map",
        minimum_horizon_s: float = 0.2,
        maximum_speed_mps: float = 15.0,
        maximum_acceleration_mps2: float = 12.0,
        maximum_curvature_1pm: float = 4.0,
    ) -> None:
        self.header.validate()
        if self.header.frame_id != expected_frame:
            raise SimulationError("trajectory frame does not match the configured planning frame")
        if not isfinite(self.valid_until_s) or self.valid_until_s < current_time_s:
            raise SimulationError("trajectory is stale")
        if len(self.points) < 2:
            raise SimulationError("trajectory needs at least two timed points")
        previous = None
        for point in self.points:
            point.validate(
                maximum_speed_mps=maximum_speed_mps,
                maximum_acceleration_mps2=maximum_acceleration_mps2,
                maximum_curvature_1pm=maximum_curvature_1pm,
            )
            if previous is not None and point.time_s <= previous:
                raise SimulationError("trajectory point times must be strictly increasing")
            previous = point.time_s
        if self.points[-1].time_s - self.points[0].time_s < minimum_horizon_s:
            raise SimulationError("trajectory horizon is too short")


@dataclass(frozen=True)
class LocalizationContract:
    header: MessageHeader
    x_m: float
    y_m: float
    yaw_rad: float
    speed_mps: float
    yaw_rate_radps: float
    position_std_m: float
    heading_std_rad: float
    track_projection_valid: bool
    quality: str
    innovation: float

    def validate(self) -> None:
        self.header.validate()
        values = (self.x_m, self.y_m, self.yaw_rad, self.speed_mps, self.yaw_rate_radps,
                  self.position_std_m, self.heading_std_rad, self.innovation)
        if not all(isfinite(value) for value in values) or self.position_std_m < 0.0 or self.heading_std_rad < 0.0:
            raise SimulationError("localization contract contains invalid values")


@dataclass(frozen=True)
class PerceptionContract:
    header: MessageHeader
    clear_path_distance_m: float
    unknown_space_distance_m: float
    coverage_fraction: float
    processing_latency_s: float
    fault_flags: tuple[str, ...] = ()

    def validate(self) -> None:
        self.header.validate()
        if not all(isfinite(value) for value in (self.clear_path_distance_m, self.unknown_space_distance_m,
                                                  self.coverage_fraction, self.processing_latency_s)):
            raise SimulationError("perception contract contains non-finite values")
        if self.clear_path_distance_m < 0.0 or self.unknown_space_distance_m < 0.0:
            raise SimulationError("perception distance cannot be negative")
        if not 0.0 <= self.coverage_fraction <= 1.0 or self.processing_latency_s < 0.0:
            raise SimulationError("perception coverage or latency is invalid")


@dataclass(frozen=True)
class ControllerOutput:
    target_speed_mps: float
    steering_rad: float
    cross_track_error_m: float = 0.0
    heading_error_rad: float = 0.0
    limiting_reason: str = ""
    target_xy_m: tuple[float, float] | None = None


def _nearest_index(state: PlanarKartState, points: Iterable[TrajectoryPoint]) -> int:
    candidates = tuple(points)
    return min(range(len(candidates)), key=lambda index: (candidates[index].x_m - state.x_m) ** 2 + (candidates[index].y_m - state.y_m) ** 2)


@dataclass
class PurePursuitController:
    lookahead_m: float = 1.5
    speed_lookahead_gain_s: float = 0.15
    wheelbase_m: float = 1.0
    maximum_steering_rad: float = 0.6
    maximum_steering_rate_radps: float = 3.0
    _last_steering_rad: float = 0.0

    def __post_init__(self) -> None:
        if not isfinite(self.wheelbase_m) or self.wheelbase_m <= 0.0:
            raise SimulationError("controller wheelbase must be positive and finite")

    def compute(self, state: PlanarKartState, trajectory: Trajectory, dt_s: float) -> ControllerOutput:
        if dt_s <= 0.0:
            raise SimulationError("controller dt must be positive")
        points = trajectory.points
        nearest = _nearest_index(state, points)
        lookahead = max(0.1, self.lookahead_m + self.speed_lookahead_gain_s * max(0.0, state.u_mps))
        target = points[-1]
        for point in points[nearest:]:
            if hypot(point.x_m - state.x_m, point.y_m - state.y_m) >= lookahead:
                target = point
                break
        dx, dy = target.x_m - state.x_m, target.y_m - state.y_m
        x_local = cos(state.yaw_rad) * dx + sin(state.yaw_rad) * dy
        y_local = -sin(state.yaw_rad) * dx + cos(state.yaw_rad) * dy
        distance = max(0.1, hypot(x_local, y_local))
        requested = atan2(2.0 * self.wheelbase_m * y_local, distance * distance)
        rate = self.maximum_steering_rate_radps * dt_s
        steering = max(self._last_steering_rad - rate, min(self._last_steering_rad + rate, requested))
        steering = max(-self.maximum_steering_rad, min(self.maximum_steering_rad, steering))
        self._last_steering_rad = steering
        return ControllerOutput(target.speed_mps, steering, y_local,
                                normalize_angle(target.heading_rad - state.yaw_rad),
                                target_xy_m=(target.x_m, target.y_m))


@dataclass
class StanleyController:
    gain: float = 1.0
    softening_mps: float = 0.5
    maximum_steering_rad: float = 0.6
    maximum_steering_rate_radps: float = 3.0
    _last_steering_rad: float = 0.0

    def compute(self, state: PlanarKartState, trajectory: Trajectory, dt_s: float) -> ControllerOutput:
        if dt_s <= 0.0 or self.softening_mps <= 0.0:
            raise SimulationError("Stanley controller requires positive dt and low-speed softening")
        point = trajectory.points[_nearest_index(state, trajectory.points)]
        # Positive cross-track error means the vehicle is right of the path
        # direction, so the positive steering correction turns left toward it.
        dx, dy = state.x_m - point.x_m, state.y_m - point.y_m
        cross_track = sin(point.heading_rad) * dx - cos(point.heading_rad) * dy
        heading_error = normalize_angle(point.heading_rad - state.yaw_rad)
        requested = heading_error + atan2(self.gain * cross_track, abs(state.u_mps) + self.softening_mps)
        rate = self.maximum_steering_rate_radps * dt_s
        steering = max(self._last_steering_rad - rate, min(self._last_steering_rad + rate, requested))
        steering = max(-self.maximum_steering_rad, min(self.maximum_steering_rad, steering))
        self._last_steering_rad = steering
        return ControllerOutput(point.speed_mps, steering, cross_track, heading_error)


@dataclass(frozen=True)
class LongitudinalOutput:
    drive: float
    brake: float
    target_speed_mps: float
    error_mps: float


@dataclass
class LongitudinalPidController:
    kp: float = 0.35
    ki: float = 0.12
    kd: float = 0.0
    integral_limit: float = 4.0
    _integral: float = 0.0
    _previous_error: float = 0.0

    def reset(self) -> None:
        self._integral = 0.0
        self._previous_error = 0.0

    def compute(self, target_speed_mps: float, current_speed_mps: float, dt_s: float, *, safety_speed_limit_mps: float,
                brake_stuck: bool = False) -> LongitudinalOutput:
        if dt_s <= 0.0 or not all(isfinite(value) for value in (target_speed_mps, current_speed_mps, safety_speed_limit_mps)):
            raise SimulationError("longitudinal PID received invalid timing or values")
        target = max(0.0, min(target_speed_mps, safety_speed_limit_mps))
        error = target - current_speed_mps
        proposed_integral = max(-self.integral_limit, min(self.integral_limit, self._integral + error * dt_s))
        derivative = (error - self._previous_error) / dt_s
        raw = self.kp * error + self.ki * proposed_integral + self.kd * derivative
        output = max(-1.0, min(1.0, raw))
        # Conditional integration is simple deterministic anti-windup.
        if output == raw or (output > 0.0 and error < 0.0) or (output < 0.0 and error > 0.0):
            self._integral = proposed_integral
        self._previous_error = error
        drive = max(0.0, output)
        brake = max(0.0, -output)
        if brake_stuck:
            drive = 0.0
        return LongitudinalOutput(drive, brake, target, error)


@dataclass
class ControllerManager:
    lateral_controller: PurePursuitController | StanleyController
    longitudinal_controller: LongitudinalPidController = field(default_factory=LongitudinalPidController)
    source_id: str = "baseline_controller"
    validity_s: float = 0.1
    _sequence: int = 0

    def command(self, state: PlanarKartState, trajectory: Trajectory, timestamp_s: float, dt_s: float, *,
                safety_speed_limit_mps: float, brake_stuck: bool = False) -> MotionCommand:
        lateral = self.lateral_controller.compute(state, trajectory, dt_s)
        longitudinal = self.longitudinal_controller.compute(
            lateral.target_speed_mps, state.u_mps, dt_s,
            safety_speed_limit_mps=safety_speed_limit_mps, brake_stuck=brake_stuck,
        )
        self._sequence += 1
        # VCU's high-level speed command is intentionally separate from its final VehicleCommand.
        return MotionCommand(
            self.source_id, self._sequence, timestamp_s, timestamp_s + self.validity_s,
            longitudinal.target_speed_mps, lateral.steering_rad, longitudinal.brake,
        ).signed()
