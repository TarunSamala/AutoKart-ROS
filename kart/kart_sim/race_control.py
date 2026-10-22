"""Deterministic race-control state, timing and penalty accounting."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from .types import SimulationError


class SessionState(str, Enum):
    SETUP = "SETUP"
    GRID = "GRID"
    READY = "READY"
    COUNTDOWN = "COUNTDOWN"
    RUNNING = "RUNNING"
    YELLOW = "YELLOW"
    RED = "RED"
    FINISHED = "FINISHED"
    ABORTED = "ABORTED"


@dataclass
class KartTiming:
    namespace: str
    lap_count: int = 0
    lap_started_s: float | None = None
    current_lap_s: float | None = None
    best_lap_s: float | None = None
    sector_times_s: list[float] = field(default_factory=list)
    cone_penalties: int = 0
    track_limit_penalties: int = 0
    collisions: int = 0
    dnf: bool = False
    estop: bool = False
    safety_state: str = "SAFE"
    faults: tuple[str, ...] = ()


class RaceControl:
    _allowed = {
        SessionState.SETUP: {SessionState.GRID, SessionState.ABORTED},
        SessionState.GRID: {SessionState.READY, SessionState.ABORTED},
        SessionState.READY: {SessionState.COUNTDOWN, SessionState.ABORTED},
        SessionState.COUNTDOWN: {SessionState.RUNNING, SessionState.ABORTED},
        SessionState.RUNNING: {SessionState.YELLOW, SessionState.RED, SessionState.FINISHED, SessionState.ABORTED},
        SessionState.YELLOW: {SessionState.RUNNING, SessionState.RED, SessionState.FINISHED, SessionState.ABORTED},
        SessionState.RED: {SessionState.ABORTED, SessionState.FINISHED},
        SessionState.FINISHED: set(),
        SessionState.ABORTED: set(),
    }

    def __init__(self, namespaces: tuple[str, ...]) -> None:
        if not namespaces or len(set(namespaces)) != len(namespaces):
            raise SimulationError("race control requires unique kart namespaces")
        self.state = SessionState.SETUP
        self.karts = {namespace: KartTiming(namespace) for namespace in namespaces}
        self.events: list[dict[str, object]] = []

    def transition(self, state: SessionState, timestamp_s: float, reason: str = "") -> None:
        if state not in self._allowed[self.state]:
            raise SimulationError(f"invalid race-control transition {self.state.value}->{state.value}")
        self.events.append({"time_s": timestamp_s, "from": self.state.value, "to": state.value, "reason": reason})
        self.state = state

    def lap_crossing(self, namespace: str, timestamp_s: float) -> None:
        kart = self.karts[namespace]
        if self.state not in {SessionState.RUNNING, SessionState.YELLOW}:
            return
        if kart.lap_started_s is not None:
            kart.current_lap_s = timestamp_s - kart.lap_started_s
            kart.best_lap_s = kart.current_lap_s if kart.best_lap_s is None else min(kart.best_lap_s, kart.current_lap_s)
            kart.lap_count += 1
        kart.lap_started_s = timestamp_s

    def penalty(self, namespace: str, kind: str, timestamp_s: float) -> None:
        kart = self.karts[namespace]
        if kind == "cone":
            kart.cone_penalties += 1
        elif kind == "track_limit":
            kart.track_limit_penalties += 1
        elif kind == "collision":
            kart.collisions += 1
        else:
            raise SimulationError("unknown penalty kind")
        self.events.append({"time_s": timestamp_s, "namespace": namespace, "penalty": kind})

    def update_safety(self, namespace: str, state: str, faults: tuple[str, ...], estop: bool) -> None:
        kart = self.karts[namespace]
        kart.safety_state, kart.faults, kart.estop = state, faults, estop
        if estop:
            kart.dnf = True

    def snapshot(self) -> dict[str, object]:
        return {
            "session_state": self.state.value,
            "karts": {
                name: {
                    "lap_count": kart.lap_count,
                    "current_lap_s": kart.current_lap_s,
                    "best_lap_s": kart.best_lap_s,
                    "sector_times_s": list(kart.sector_times_s),
                    "cone_penalties": kart.cone_penalties,
                    "track_limit_penalties": kart.track_limit_penalties,
                    "collisions": kart.collisions,
                    "dnf": kart.dnf,
                    "estop": kart.estop,
                    "safety_state": kart.safety_state,
                    "faults": list(kart.faults),
                }
                for name, kart in self.karts.items()
            },
        }

