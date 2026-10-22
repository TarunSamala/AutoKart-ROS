"""Formats the geometry engine's already-computed centerline/boundaries/gates
into a corridor summary for the planner/dashboard. Generates no new geometry:
if autokart_perception didn't produce a centerline, this reports NO_PATH, it
never invents one.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Corridor:
    status: str                      # mirrors result.diagnostics['centerline_status'], or NO_PATH
    centerline_xy: list[tuple[float, float]]
    left_cone_ids: tuple[int, ...]
    right_cone_ids: tuple[int, ...]
    gate_widths_m: list[float]
    confidence: float | None


def build_corridor(result) -> Corridor:
    boundaries = result.boundaries
    centerline_status = result.diagnostics.get("centerline_status", "UNKNOWN")
    centerline = list(result.centerline) if centerline_status == "PROVISIONAL" else []
    accepted_gates = [g for g in result.gates if g.status == "CANDIDATE"]
    return Corridor(
        status=centerline_status if centerline else "NO_PATH",
        centerline_xy=centerline,
        left_cone_ids=boundaries.left_ids if boundaries else (),
        right_cone_ids=boundaries.right_ids if boundaries else (),
        gate_widths_m=[g.width for g in accepted_gates],
        confidence=boundaries.confidence if boundaries else None,
    )
