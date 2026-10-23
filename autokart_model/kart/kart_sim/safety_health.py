"""Function-oriented health aggregation for VCU entry and degradation guards."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite


class HealthQuality(str, Enum):
    VALID = "VALID"
    DEGRADED = "DEGRADED"
    INVALID = "INVALID"
    STALE = "STALE"


@dataclass(frozen=True)
class SignalHealth:
    signal_id: str
    timestamp_s: float
    valid: bool = True
    quality: HealthQuality = HealthQuality.VALID
    confidence: float = 1.0

    def usable(self, now_s: float, maximum_age_s: float) -> bool:
        return (
            self.valid
            and self.quality not in (HealthQuality.INVALID, HealthQuality.STALE)
            and isfinite(self.timestamp_s)
            and 0.0 <= now_s - self.timestamp_s <= maximum_age_s
            and 0.0 <= self.confidence <= 1.0
        )


@dataclass(frozen=True)
class FunctionHealth:
    function_id: str
    quality: HealthQuality
    usable_sources: tuple[str, ...]
    required_sources: int

    @property
    def valid(self) -> bool:
        return self.quality in (HealthQuality.VALID, HealthQuality.DEGRADED)


class SensorFunctionHealthManager:
    """Aggregate device signals by required vehicle function."""

    def evaluate(
        self,
        function_id: str,
        signals: tuple[SignalHealth, ...],
        *,
        now_s: float,
        maximum_age_s: float,
        required_sources: int = 1,
        nominal_sources: int | None = None,
    ) -> FunctionHealth:
        usable = tuple(signal.signal_id for signal in signals if signal.usable(now_s, maximum_age_s))
        nominal = len(signals) if nominal_sources is None else nominal_sources
        if len(usable) < required_sources:
            quality = HealthQuality.INVALID
        elif len(usable) < nominal:
            quality = HealthQuality.DEGRADED
        else:
            quality = HealthQuality.VALID
        return FunctionHealth(function_id, quality, usable, required_sources)


@dataclass(frozen=True)
class LocalizationHealth:
    timestamp_s: float
    position_covariance_m2: float
    heading_covariance_rad2: float
    track_projection_valid: bool

    def valid(self, now_s: float, maximum_age_s: float = 0.1) -> bool:
        return (
            0.0 <= now_s - self.timestamp_s <= maximum_age_s
            and self.track_projection_valid
            and 0.0 <= self.position_covariance_m2 <= 1.0
            and 0.0 <= self.heading_covariance_rad2 <= 0.1
        )


@dataclass(frozen=True)
class PerceptionHealth:
    timestamp_s: float
    clear_path_distance_m: float
    unknown_space_distance_m: float
    coverage_fraction: float

    def valid(self, now_s: float, maximum_age_s: float = 0.1) -> bool:
        return (
            0.0 <= now_s - self.timestamp_s <= maximum_age_s
            and self.clear_path_distance_m >= 0.0
            and self.unknown_space_distance_m >= 0.0
            and 0.0 <= self.coverage_fraction <= 1.0
        )
