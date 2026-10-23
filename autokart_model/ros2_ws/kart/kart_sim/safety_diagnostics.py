"""DTC metadata and bounded pre/post-fault snapshot storage."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class DtcDefinition:
    code: str
    description: str
    severity: str
    response: str
    related_test: str


DTC_REGISTRY = {
    "BRAKE-003": DtcDefinition(
        "BRAKE-003",
        "Brake mechanical-transmission response missing",
        "CRITICAL",
        "EMERGENCY_STOP",
        "BC2",
    ),
    "MOTOR-003": DtcDefinition(
        "MOTOR-003",
        "Uncommanded positive motor torque",
        "CRITICAL",
        "EMERGENCY_STOP",
        "TR-ST2",
    ),
    "POWER-004": DtcDefinition(
        "POWER-004",
        "Traction isolation command/feedback mismatch",
        "CRITICAL",
        "FAULT_LOCKOUT",
        "TR-ST4",
    ),
}


@dataclass(frozen=True)
class FaultSnapshot:
    timestamp_s: float
    state: str
    fault_codes: tuple[str, ...]
    vehicle_speed_mps: float
    motor_torque_nm: float
    motor_current_a: float
    brake_pressure_pa: float
    steering_angle_rad: float


class FaultSnapshotBuffer:
    def __init__(self, capacity: int = 32) -> None:
        if capacity <= 0:
            raise ValueError("snapshot capacity must be positive")
        self._records: deque[FaultSnapshot] = deque(maxlen=capacity)

    def append(self, snapshot: FaultSnapshot) -> None:
        self._records.append(snapshot)

    def records(self) -> tuple[FaultSnapshot, ...]:
        return tuple(self._records)
