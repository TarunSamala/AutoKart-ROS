"""Deterministic task watchdogs; heartbeat payload health is deliberately separate."""
from __future__ import annotations
from dataclasses import dataclass
from .types import SimulationError


@dataclass(frozen=True)
class WatchdogStatus:
    healthy: bool
    fault: str | None
    deadline_misses: int


class WindowedTaskWatchdog:
    def __init__(self, minimum_period_s: float, maximum_period_s: float, misses_to_fault: int = 1) -> None:
        if not 0 <= minimum_period_s < maximum_period_s or misses_to_fault <= 0:
            raise SimulationError("invalid windowed watchdog configuration")
        self.minimum_period_s = minimum_period_s
        self.maximum_period_s = maximum_period_s
        self.misses_to_fault = misses_to_fault
        self.last_timestamp_s: float | None = None
        self.last_sequence: int | None = None
        self.deadline_misses = 0
        self.fault: str | None = None

    def observe(self, timestamp_s: float, sequence: int) -> WatchdogStatus:
        if self.last_timestamp_s is not None:
            period = timestamp_s - self.last_timestamp_s
            if period < self.minimum_period_s:
                self.fault = "WATCHDOG_TOO_FAST"
            elif period > self.maximum_period_s:
                self.deadline_misses += 1
                if self.deadline_misses >= self.misses_to_fault:
                    self.fault = "WATCHDOG_DEADLINE"
            else:
                self.deadline_misses = 0
            if self.last_sequence == sequence:
                self.fault = "WATCHDOG_SEQUENCE_STUCK"
        self.last_timestamp_s = timestamp_s
        self.last_sequence = sequence
        return self.status()

    def check_expired(self, timestamp_s: float) -> WatchdogStatus:
        if self.last_timestamp_s is None or timestamp_s - self.last_timestamp_s > self.maximum_period_s:
            self.fault = "WATCHDOG_EXPIRED"
        return self.status()

    def status(self) -> WatchdogStatus:
        return WatchdogStatus(self.fault is None, self.fault, self.deadline_misses)
