"""Fixed-step deterministic scheduler independent of rendering wall clock."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

from .types import SimulationError

StepCallback = Callable[[float, float], None]


@dataclass
class FixedStepScheduler:
    dt_s: float
    time_s: float = 0.0
    step_count: int = 0
    _callbacks: list[StepCallback] = field(default_factory=list)

    def __post_init__(self) -> None:
        if not 0 < self.dt_s <= 0.02:
            raise SimulationError("dt_s must be in (0, 0.02] seconds")

    def add_callback(self, callback: StepCallback) -> None:
        self._callbacks.append(callback)

    def step(self, steps: int = 1) -> None:
        if steps < 0:
            raise SimulationError("steps must not be negative")
        for _ in range(steps):
            for callback in self._callbacks:
                callback(self.time_s, self.dt_s)
            self.step_count += 1
            self.time_s = self.step_count * self.dt_s

    def run_until(self, end_time_s: float) -> None:
        if end_time_s < self.time_s:
            raise SimulationError("scheduler cannot run backwards")
        while self.time_s + self.dt_s <= end_time_s + 1e-12:
            self.step()
