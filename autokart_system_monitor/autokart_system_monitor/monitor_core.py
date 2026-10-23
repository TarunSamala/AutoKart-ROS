"""Pure logic for autokart_system_monitor -- no ROS dependency, so it is
directly unit-testable and mirrors the "pure logic / ROS wrapper" split
every other node in this library uses (e.g. autokart_vcu_safety's core.py).

This node REPORTS health; it has NO safety authority and cannot gate, stop,
or authorize anything. Contrast with autokart_safety (node 10), which IS an
arming authority. See CONTRACT.md.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

from .resource_backends import ResourceSample

# Mirrors diagnostic_msgs/DiagnosticStatus levels (byte constants) without
# importing diagnostic_msgs here, keeping this module ROS-free.
OK, WARN, ERROR, STALE = 0, 1, 2, 3
LEVEL_NAME = {OK: "OK", WARN: "WARN", ERROR: "ERROR", STALE: "STALE"}


@dataclass(frozen=True)
class ComponentStatus:
    name: str
    level: int
    message: str
    values: Dict[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class HealthReport:
    overall_level: int
    components: List[ComponentStatus]


def _heartbeat_status(name: str, age_s: Optional[float], timeout_s: float) -> ComponentStatus:
    if age_s is None:
        return ComponentStatus(name, STALE, "never seen", {"age_s": "null", "timeout_s": f"{timeout_s:.2f}"})
    if age_s > timeout_s:
        return ComponentStatus(name, STALE, "stale: no message within timeout",
                                {"age_s": f"{age_s:.3f}", "timeout_s": f"{timeout_s:.2f}"})
    return ComponentStatus(name, OK, "fresh", {"age_s": f"{age_s:.3f}", "timeout_s": f"{timeout_s:.2f}"})


def _resource_status(sample: ResourceSample, cpu_warn_pct: float, ram_warn_pct: float) -> ComponentStatus:
    if not sample.available:
        return ComponentStatus("resources", WARN, f"resource sampling unavailable: {sample.reason}",
                                {"cpu_percent": "null", "ram_percent": "null"})
    level = OK
    reasons = []
    if sample.cpu_percent is not None and sample.cpu_percent >= cpu_warn_pct:
        level = WARN
        reasons.append(f"cpu {sample.cpu_percent:.1f}% >= {cpu_warn_pct:.1f}%")
    if sample.ram_percent is not None and sample.ram_percent >= ram_warn_pct:
        level = WARN
        reasons.append(f"ram {sample.ram_percent:.1f}% >= {ram_warn_pct:.1f}%")
    message = "within limits" if not reasons else "; ".join(reasons)
    return ComponentStatus("resources", level, message, {
        "cpu_percent": f"{sample.cpu_percent:.1f}" if sample.cpu_percent is not None else "null",
        "ram_percent": f"{sample.ram_percent:.1f}" if sample.ram_percent is not None else "null",
    })


def evaluate_health(heartbeat_ages: Dict[str, Optional[float]], timeouts: Dict[str, float],
                     resource_sample: ResourceSample, cpu_warn_pct: float = 90.0,
                     ram_warn_pct: float = 90.0) -> HealthReport:
    """Pure function: given freshness ages for every monitored heartbeat key
    and one resource sample, produce a full component breakdown plus an
    overall rollup level (the worst of any component). Never raises, never
    needs a live ROS graph -- directly unit-testable."""
    components = [
        _heartbeat_status(name, heartbeat_ages.get(name), timeouts.get(name, 1.0))
        for name in sorted(heartbeat_ages)
    ]
    components.append(_resource_status(resource_sample, cpu_warn_pct, ram_warn_pct))
    overall = max((c.level for c in components), default=OK)
    return HealthReport(overall_level=overall, components=components)


class StartupLatch:
    """Tracks, per monitored key, whether it has EVER been seen fresh at
    least once since this node started -- distinct from "is it fresh right
    now" (that is what /system/health answers). Once a key latches OK it
    never regresses back to "never seen" even if the upstream node later
    goes stale or restarts -- this answers "did the system finish booting",
    not "is it currently healthy". A monitored key that has never published
    at all (the node isn't running, or hasn't published yet) correctly stays
    NOT_SEEN forever until it does -- this node degrades gracefully rather
    than erroring when a dependency simply isn't running yet.
    """

    def __init__(self, required_keys: List[str]):
        self.required_keys = list(required_keys)
        self._ever_seen: Dict[str, bool] = {key: False for key in required_keys}

    def observe(self, heartbeat_ages: Dict[str, Optional[float]]) -> None:
        for key in self.required_keys:
            if heartbeat_ages.get(key) is not None:
                self._ever_seen[key] = True

    def status(self) -> HealthReport:
        components = [
            ComponentStatus(key, OK if seen else STALE,
                             "seen at least once" if seen else "never seen since startup", {})
            for key, seen in sorted(self._ever_seen.items())
        ]
        overall = OK if all(self._ever_seen.values()) else STALE
        return HealthReport(overall_level=overall, components=components)

    @property
    def fully_started(self) -> bool:
        return all(self._ever_seen.values())
