"""Detects a cone obstructing the planned corridor and computes the stopping
clearance needed to react to it.

Reuses autokart_perception.shared_core.BrakeCapability.stopping_distance(),
the same physics-based formula (speed*latency + speed^2/(2*decel) + margin)
the production shared algorithm core uses for its own BRAKING_TEST mission,
instead of a second copy. BrakeCapability.valid_for() is also reused: with the
placeholder zero deceleration/latency this harness ships with, it correctly
refuses to call the stopping distance "qualified", matching the project's
rule that 15 cm (or any number) is not a target until it is measured.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

PERCEPTION_SRC = (Path(__file__).resolve().parents[2] /
                   "autonomy_stack_phase1/phase_01_perception/src")
if str(PERCEPTION_SRC) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_SRC))

from autokart_perception.shared_core import BrakeCapability  # noqa: E402


@dataclass
class ObstacleStatus:
    blocked: bool
    nearest_forward_m: float | None
    required_clearance_m: float | None
    clearance_qualified: bool
    reason: str


def _project_onto_corridor(cone_xy, corridor_half_width_m: float, lookahead_m: float) -> bool:
    x_forward, y_left = cone_xy
    return 0.0 < x_forward <= lookahead_m and abs(y_left) <= corridor_half_width_m


def check_obstacle(result, cfg: dict, current_speed_mps: float) -> ObstacleStatus:
    """cfg is the 'planning'+'stopping' section of test_config.yaml (merged dict)."""
    corridor_half_width_m = cfg["corridor_half_width_m"]
    lookahead_m = cfg["obstacle_lookahead_m"]

    blocking = [c for c in result.cones
                if _project_onto_corridor(c.xy, corridor_half_width_m, lookahead_m)]
    if not blocking:
        return ObstacleStatus(False, None, None, False, "NO_CONE_IN_CORRIDOR")

    nearest = min(blocking, key=lambda c: c.xy[0])
    nearest_forward_m = float(nearest.xy[0])

    if not cfg["measured"] or cfg["brake_min_mps2"] <= 0:
        # No physics-based number is meaningful without a measured deceleration;
        # reporting a huge or zero value here would look like a real distance.
        return ObstacleStatus(
            blocked=True, nearest_forward_m=nearest_forward_m,
            required_clearance_m=None, clearance_qualified=False,
            reason="CONE_IN_PATH_CLEARANCE_UNQUALIFIED_NO_MEASURED_BRAKING",
        )

    brake = BrakeCapability(
        minimum_deceleration_mps2=cfg["brake_min_mps2"],
        maximum_delay_s=cfg["t_latency_s"],
        provenance="MEASURED",
        margin_m=cfg["margin_m"],
    )
    qualified = brake.valid_for("LIVE")
    required_clearance_m = brake.stopping_distance(max(current_speed_mps, 0.0)) if qualified else None

    return ObstacleStatus(
        blocked=True,
        nearest_forward_m=nearest_forward_m,
        required_clearance_m=required_clearance_m,
        clearance_qualified=qualified,
        reason="CONE_IN_PATH" if qualified else "CONE_IN_PATH_CLEARANCE_UNQUALIFIED",
    )
