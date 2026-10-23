"""Real-time scheduling and bounded task-timing diagnostics.

The scheduler never skips physics, VCU, or independent-safety work.  It may
report lateness, while sensor producers apply their own explicit drop policy.
"""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import asdict, dataclass
from math import ceil
from statistics import fmean
from time import perf_counter, sleep
from typing import Literal

from .types import SimulationError

SchedulerMode = Literal["deterministic_offline", "realtime_synchronized", "step"]


@dataclass(frozen=True)
class TaskBudget:
    rate_hz: float
    average_budget_s: float
    deadline_s: float
    severity: Literal["info", "warning", "error", "critical"]

    def validated(self) -> "TaskBudget":
        if self.rate_hz <= 0.0 or self.average_budget_s <= 0.0 or self.deadline_s <= 0.0:
            raise SimulationError("task timing budgets must be positive")
        if self.average_budget_s > self.deadline_s:
            raise SimulationError("average timing budget cannot exceed the deadline")
        return self


@dataclass(frozen=True)
class TaskTimingStatistics:
    samples: int
    retained_samples: int
    mean_s: float
    p95_s: float
    p99_s: float
    maximum_s: float
    deadline_s: float
    deadline_misses: int
    consecutive_misses: int
    maximum_consecutive_misses: int
    mean_jitter_s: float
    severity: str


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, ceil(percentile * len(ordered)) - 1))
    return ordered[index]


class TaskTimingMonitor:
    """Collect bounded execution samples and deadline diagnostics by task."""

    def __init__(
        self,
        budgets: dict[str, TaskBudget],
        *,
        maximum_samples_per_task: int = 200_000,
    ) -> None:
        if maximum_samples_per_task <= 0:
            raise SimulationError("timing sample capacity must be positive")
        self.budgets = {
            name: budget.validated() for name, budget in budgets.items()
        }
        self.maximum_samples_per_task = maximum_samples_per_task
        self._samples: dict[str, deque[float]] = {
            name: deque(maxlen=maximum_samples_per_task) for name in budgets
        }
        self._deadline_misses: dict[str, int] = defaultdict(int)
        self._total_samples: dict[str, int] = defaultdict(int)
        self._consecutive_misses: dict[str, int] = defaultdict(int)
        self._maximum_consecutive_misses: dict[str, int] = defaultdict(int)

    def record(self, task: str, elapsed_s: float) -> None:
        if task not in self.budgets:
            raise SimulationError(f"no timing budget configured for task: {task}")
        if elapsed_s < 0.0:
            raise SimulationError("task elapsed time cannot be negative")
        self._total_samples[task] += 1
        self._samples[task].append(float(elapsed_s))
        if elapsed_s > self.budgets[task].deadline_s:
            self._deadline_misses[task] += 1
            self._consecutive_misses[task] += 1
            self._maximum_consecutive_misses[task] = max(
                self._maximum_consecutive_misses[task],
                self._consecutive_misses[task],
            )
        else:
            self._consecutive_misses[task] = 0

    def statistics(self, task: str) -> TaskTimingStatistics:
        if task not in self.budgets:
            raise SimulationError(f"unknown timed task: {task}")
        values = list(self._samples[task])
        mean = fmean(values) if values else 0.0
        jitter = fmean(abs(value - mean) for value in values) if values else 0.0
        budget = self.budgets[task]
        return TaskTimingStatistics(
            samples=self._total_samples[task],
            retained_samples=len(values),
            mean_s=mean,
            p95_s=_percentile(values, 0.95),
            p99_s=_percentile(values, 0.99),
            maximum_s=max(values, default=0.0),
            deadline_s=budget.deadline_s,
            deadline_misses=self._deadline_misses[task],
            consecutive_misses=self._consecutive_misses[task],
            maximum_consecutive_misses=self._maximum_consecutive_misses[task],
            mean_jitter_s=jitter,
            severity=budget.severity,
        )

    def report(self) -> dict[str, dict]:
        return {
            task: asdict(self.statistics(task))
            for task in sorted(self.budgets)
        }


@dataclass(frozen=True)
class SynchronizationStatus:
    mode: SchedulerMode
    simulation_time_s: float
    wall_elapsed_s: float
    clock_drift_s: float
    behind_wall_time_s: float
    deadline_misses: int


class RealTimeSynchronizer:
    """Pace simulation time against wall time without dropping core cycles."""

    def __init__(
        self,
        mode: SchedulerMode,
        *,
        maximum_clock_drift_s: float = 0.020,
    ) -> None:
        if mode not in {"deterministic_offline", "realtime_synchronized", "step"}:
            raise SimulationError(f"unsupported scheduler mode: {mode}")
        if maximum_clock_drift_s <= 0.0:
            raise SimulationError("maximum clock drift must be positive")
        self.mode = mode
        self.maximum_clock_drift_s = maximum_clock_drift_s
        self._wall_start_s = perf_counter()
        self._simulation_start_s = 0.0
        self._deadline_misses = 0
        self._last_simulation_time_s = 0.0

    def reset(self, simulation_time_s: float = 0.0) -> None:
        if simulation_time_s < 0.0:
            raise SimulationError("synchronizer reset time cannot be negative")
        self._wall_start_s = perf_counter()
        self._simulation_start_s = simulation_time_s
        self._last_simulation_time_s = simulation_time_s
        self._deadline_misses = 0

    def synchronize(self, simulation_time_s: float) -> SynchronizationStatus:
        if simulation_time_s + 1e-12 < self._last_simulation_time_s:
            raise SimulationError("real-time synchronizer prohibits rewind")
        self._last_simulation_time_s = simulation_time_s
        target_elapsed = simulation_time_s - self._simulation_start_s
        wall_elapsed = perf_counter() - self._wall_start_s
        if self.mode == "realtime_synchronized" and target_elapsed > wall_elapsed:
            sleep(target_elapsed - wall_elapsed)
            wall_elapsed = perf_counter() - self._wall_start_s
        drift = target_elapsed - wall_elapsed
        behind = max(0.0, -drift)
        if self.mode == "realtime_synchronized" and behind > self.maximum_clock_drift_s:
            self._deadline_misses += 1
        return SynchronizationStatus(
            mode=self.mode,
            simulation_time_s=simulation_time_s,
            wall_elapsed_s=wall_elapsed,
            clock_drift_s=drift,
            behind_wall_time_s=behind,
            deadline_misses=self._deadline_misses,
        )
