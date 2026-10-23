"""Configurable Formula Student-style event scoring without compliance claims."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any

import yaml

from .types import SimulationError


class EventType(str, Enum):
    ACCELERATION = "acceleration"
    SKIDPAD = "skidpad"
    AUTOCROSS = "autocross"
    TRACKDRIVE = "trackdrive"
    EMERGENCY_BRAKE_TEST = "emergency_brake_test"


@dataclass(frozen=True)
class EventConfiguration:
    event_type: EventType
    track: str
    cone_penalty_s: float = 0.0
    timed_distance_m: float | None = None
    laps: int = 1
    trigger_s_m: float | None = None
    stop_boundary_s_m: float | None = None


def load_event(path: str | Path) -> EventConfiguration:
    payload: dict[str, Any] = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    event = payload["event"]
    event_type = EventType(event["type"])
    laps = int(event.get("laps", event.get("timed_laps", 1)))
    configuration = EventConfiguration(
        event_type,
        str(event["track"]),
        float(event.get("cone_penalty_s", 0.0)),
        None if "timed_distance_m" not in event else float(event["timed_distance_m"]),
        laps,
        None if "trigger_s_m" not in event else float(event["trigger_s_m"]),
        None if "stop_boundary_s_m" not in event else float(event["stop_boundary_s_m"]),
    )
    if configuration.laps <= 0 or configuration.cone_penalty_s < 0:
        raise SimulationError("invalid event lap or penalty configuration")
    if event_type == EventType.EMERGENCY_BRAKE_TEST and (
        configuration.trigger_s_m is None
        or configuration.stop_boundary_s_m is None
        or configuration.stop_boundary_s_m <= configuration.trigger_s_m
    ):
        raise SimulationError("EBS test requires ordered trigger and stop boundary")
    return configuration


@dataclass
class EventResult:
    event_type: str
    started_s: float | None = None
    finished_s: float | None = None
    raw_time_s: float | None = None
    penalty_time_s: float = 0.0
    scored_time_s: float | None = None
    cone_hits: int = 0
    track_limit_violations: int = 0
    maximum_speed_mps: float = 0.0
    energy_wh: float = 0.0
    stopping_distance_m: float | None = None
    stop_time_s: float | None = None
    boundary_violation: bool = False
    completed: bool = False
    notes: list[str] = field(default_factory=list)


class EventEvaluator:
    def __init__(self, configuration: EventConfiguration) -> None:
        self.configuration = configuration
        self.result = EventResult(configuration.event_type.value)
        self._previous_s_m: float | None = None
        self._lap_count = 0
        self._ebs_trigger_time_s: float | None = None
        self._ebs_trigger_s_m: float | None = None

    def update(
        self,
        *,
        timestamp_s: float,
        track_s_m: float,
        track_length_m: float,
        speed_mps: float,
        battery_power_w: float = 0.0,
        dt_s: float = 0.0,
        cone_hits: int = 0,
        track_limit_violation: bool = False,
        estop_active: bool = False,
    ) -> EventResult:
        result = self.result
        result.maximum_speed_mps = max(result.maximum_speed_mps, abs(speed_mps))
        result.energy_wh += max(0.0, battery_power_w) * max(0.0, dt_s) / 3600.0
        if result.started_s is None and speed_mps > 0.05:
            result.started_s = timestamp_s
        if cone_hits > result.cone_hits:
            new_hits = cone_hits - result.cone_hits
            result.cone_hits = cone_hits
            result.penalty_time_s += new_hits * self.configuration.cone_penalty_s
        if track_limit_violation:
            result.track_limit_violations += 1
        if self._previous_s_m is not None and self._previous_s_m > 0.8 * track_length_m and track_s_m < 0.2 * track_length_m:
            self._lap_count += 1
        self._previous_s_m = track_s_m
        if self.configuration.event_type == EventType.ACCELERATION:
            if self.configuration.timed_distance_m is not None and track_s_m >= self.configuration.timed_distance_m:
                self._finish(timestamp_s)
        elif self.configuration.event_type in {EventType.SKIDPAD, EventType.AUTOCROSS, EventType.TRACKDRIVE}:
            if self._lap_count >= self.configuration.laps:
                self._finish(timestamp_s)
        else:
            trigger = self.configuration.trigger_s_m
            boundary = self.configuration.stop_boundary_s_m
            assert trigger is not None and boundary is not None
            if self._ebs_trigger_time_s is None and track_s_m >= trigger:
                self._ebs_trigger_time_s = timestamp_s
                self._ebs_trigger_s_m = track_s_m
                if not estop_active:
                    result.notes.append("E-stop was not active at configured trigger")
            if self._ebs_trigger_time_s is not None and abs(speed_mps) < 0.05:
                result.stopping_distance_m = max(0.0, track_s_m - (self._ebs_trigger_s_m or track_s_m))
                result.stop_time_s = timestamp_s - self._ebs_trigger_time_s
                result.boundary_violation = track_s_m > boundary
                self._finish(timestamp_s)
            elif track_s_m > boundary:
                result.boundary_violation = True
        return result

    def _finish(self, timestamp_s: float) -> None:
        result = self.result
        if result.completed:
            return
        result.finished_s = timestamp_s
        result.raw_time_s = None if result.started_s is None else timestamp_s - result.started_s
        result.scored_time_s = None if result.raw_time_s is None else result.raw_time_s + result.penalty_time_s
        result.completed = True

