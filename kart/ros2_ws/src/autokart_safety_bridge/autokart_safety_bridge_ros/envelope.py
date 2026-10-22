"""Pure classification logic for the AutoKart front safety envelope monitor.

Diagnostic-only: this module computes a zone classification and numeric
distances. Nothing here has a path to a vehicle command topic -- callers
that want to keep that property must not wire the result into one.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from autokart_vehicle_geometry.geometry import GEOMETRY

STATE_CLEAR = "CLEAR"
STATE_BRAKING = "BRAKING"
STATE_CRITICAL = "CRITICAL"
STATE_UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class EnvelopeParameters:
    """Front safety envelope constants.

    bumper_setback_m and corridor_half_width_m are the user-measured camera-to-bumper offset and half bumper
    width -- see kart/ros2_ws/src/autokart_bringup/config/vehicle_geometry.yaml, the single source for both
    (autokart_vehicle_geometry/test/test_vehicle_geometry.py fails if this drifts from it). response_latency_s,
    guaranteed_deceleration_mps2 and distance_margin_m are still PROVISIONAL: they come from an illustrative
    calculation, not a measured braking test. Treat outputs derived from them as diagnostic, not an approved
    operating distance, until replaced with measured values.
    """

    minimum_bumper_clearance_m: float = 1.0
    bumper_setback_m: float = GEOMETRY.camera_to_front_bumper_horizontal_m
    corridor_half_width_m: float = GEOMETRY.half_bumper_width_m
    response_latency_s: float = 0.25
    guaranteed_deceleration_mps2: float = 2.0
    distance_margin_m: float = 0.3


def braking_trigger_distance_m(speed_mps: float, params: EnvelopeParameters) -> float:
    """Required clearance-from-bumper at which braking must begin, at the given speed.

    D_trigger = minimum_clearance + v*T + v^2/(2a) + M
    """
    speed = max(0.0, speed_mps)
    return (
        params.minimum_bumper_clearance_m
        + speed * params.response_latency_s
        + (speed * speed) / (2.0 * params.guaranteed_deceleration_mps2)
        + params.distance_margin_m
    )


def nearest_obstacle_clearance_m(
    obstacle_positions_xy: list[tuple[float, float]], params: EnvelopeParameters
) -> Optional[float]:
    """Nearest clearance from the bumper among obstacles inside the monitored corridor.

    Positions are (x_forward_m, y_left_m) in base_link. Returns None when no
    obstacle is in the corridor ahead of the bumper (a legitimate CLEAR
    reading, distinct from no data having arrived at all).
    """
    candidates = [
        x - params.bumper_setback_m
        for x, y in obstacle_positions_xy
        if x > params.bumper_setback_m and abs(y) <= params.corridor_half_width_m
    ]
    return min(candidates) if candidates else None


@dataclass(frozen=True)
class ClassificationResult:
    state: str
    clearance_m: Optional[float]
    trigger_distance_m: Optional[float]
    valid: bool
    reason: str


def classify(
    *,
    speed_mps: Optional[float],
    odometry_fresh: bool,
    obstacle_feed_fresh: bool,
    obstacle_positions_xy: list[tuple[float, float]],
    params: EnvelopeParameters,
) -> ClassificationResult:
    """Classify the front safety zone. Any missing/stale input yields UNKNOWN, never CLEAR."""
    if not odometry_fresh or speed_mps is None:
        return ClassificationResult(STATE_UNKNOWN, None, None, False, "ODOMETRY_STALE_OR_MISSING")
    if not obstacle_feed_fresh:
        return ClassificationResult(STATE_UNKNOWN, None, None, False, "OBSTACLE_FEED_STALE_OR_MISSING")

    trigger = braking_trigger_distance_m(speed_mps, params)
    clearance = nearest_obstacle_clearance_m(obstacle_positions_xy, params)

    if clearance is None:
        return ClassificationResult(STATE_CLEAR, None, trigger, True, "NO_OBSTACLE_IN_MONITORED_CORRIDOR")
    if clearance <= params.minimum_bumper_clearance_m:
        return ClassificationResult(STATE_CRITICAL, clearance, trigger, True, "OBSTACLE_WITHIN_MINIMUM_CLEARANCE")
    if clearance <= trigger:
        return ClassificationResult(STATE_BRAKING, clearance, trigger, True, "OBSTACLE_WITHIN_BRAKING_ENVELOPE")
    return ClassificationResult(STATE_CLEAR, clearance, trigger, True, "OBSTACLE_OUTSIDE_BRAKING_ENVELOPE")
