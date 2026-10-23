"""Structured deterministic DTC records with latching and retained history."""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum


class FaultStatus(str, Enum):
    NOT_PRESENT = "NOT_PRESENT"
    PENDING = "PENDING"
    ACTIVE = "ACTIVE"
    RECOVERED = "RECOVERED"
    LATCHED = "LATCHED"
    HISTORICAL = "HISTORICAL"


@dataclass
class FaultRecord:
    code: str
    severity: str
    first_occurrence_s: float
    latest_occurrence_s: float
    occurrence_count: int = 1
    status: FaultStatus = FaultStatus.ACTIVE
    latched: bool = True
    required_response: str = "CONTROLLED_STOP"
    reset_requirement: str = "operator_reset"


class FaultManager:
    def __init__(self) -> None:
        self.active: dict[str, FaultRecord] = {}
        self.history: dict[str, FaultRecord] = {}

    def confirm(self, code: str, timestamp_s: float, *, severity: str, response: str, latched: bool = True) -> FaultRecord:
        record = self.active.get(code)
        if record is None:
            record = FaultRecord(code, severity, timestamp_s, timestamp_s, latched=latched, required_response=response)
            self.active[code] = record
            self.history.setdefault(code, record)
        else:
            record.latest_occurrence_s = timestamp_s
            record.occurrence_count += 1
        return record

    def recover(self, code: str) -> None:
        record = self.active.get(code)
        if record and not record.latched:
            record.status = FaultStatus.RECOVERED
            del self.active[code]

    def reset_latched(self, allowed: bool) -> tuple[str, ...]:
        if not allowed:
            return tuple(self.active)
        cleared = tuple(self.active)
        for record in self.active.values():
            record.status = FaultStatus.HISTORICAL
        self.active.clear()
        return cleared
