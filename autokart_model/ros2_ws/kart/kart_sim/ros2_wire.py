"""Transport-neutral representations matching the optional ROS message files."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite

from .types import SimulationError
from .vcu import MotionCommand


@dataclass(frozen=True)
class RosTime:
    sec: int
    nanosec: int

    @classmethod
    def from_seconds(cls, value: float) -> "RosTime":
        if not isfinite(value) or value < 0.0:
            raise SimulationError("ROS time must be finite and nonnegative")
        sec = int(value)
        nanosec = int(round((value - sec) * 1_000_000_000))
        if nanosec == 1_000_000_000:
            sec, nanosec = sec + 1, 0
        return cls(sec, nanosec)

    def seconds(self) -> float:
        return self.sec + self.nanosec * 1e-9


@dataclass(frozen=True)
class MotionCommandWire:
    source_id: str
    sequence: int
    measurement_time: RosTime
    valid_until: RosTime
    speed_mps: float
    steering_rad: float
    brake: float
    emergency_stop: bool
    crc16: int
    mode: str = "speed_steering"
    acceleration_limit_mps2: float = 3.0
    deceleration_limit_mps2: float = 5.0
    valid: bool = True
    reset_epoch: int = 0


def motion_command_to_wire(command: MotionCommand) -> MotionCommandWire:
    return MotionCommandWire(
        command.source,
        command.sequence,
        RosTime.from_seconds(command.timestamp_s),
        RosTime.from_seconds(command.valid_until_s),
        command.speed_mps,
        command.steering_rad,
        command.brake,
        command.estop,
        command.checksum,
        command.mode,
        command.acceleration_limit_mps2,
        command.deceleration_limit_mps2,
        command.valid,
        command.reset_epoch,
    )


def motion_command_from_wire(message: MotionCommandWire) -> MotionCommand:
    command = MotionCommand(
        message.source_id,
        message.sequence,
        message.measurement_time.seconds(),
        message.valid_until.seconds(),
        message.speed_mps,
        message.steering_rad,
        message.brake,
        message.emergency_stop,
        message.crc16,
        message.mode,
        message.acceleration_limit_mps2,
        message.deceleration_limit_mps2,
        message.valid,
        message.reset_epoch,
    )
    if command.crc() != command.checksum:
        raise SimulationError("ROS motion command CRC mismatch")
    return command
