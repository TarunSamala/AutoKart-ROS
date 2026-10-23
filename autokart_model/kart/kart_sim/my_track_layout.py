"""Loads docs/my_track_spec/my_track_layout.json (Rev 7) as ground-truth cones
for the my_track scenario. The algorithm (my_track_algorithm.py) never reads
the `role` field below -- it must discover boundary vs. obstacle vs. finish
from geometry alone, the same way the real perception stack would. `role` is
kept only for scoring/ground-truth checks (e.g. "did a collision happen with
a cone tagged obstacle").
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from .cones import Cone, ConeClass
from .types import SimulationError

_COLOUR_TO_CLASS = {
    "BLUE": ConeClass.BLUE,
    "YELLOW": ConeClass.YELLOW,
    "ORANGE": ConeClass.LARGE_ORANGE,
}


@dataclass(frozen=True)
class MyTrackLayout:
    revision: int
    corridor_m: float
    length_m: float
    spacing_m: float
    sections: dict[str, tuple[float, float]]
    cones: tuple[Cone, ...]
    roles: dict[str, str]
    start_pose: tuple[float, float, float]  # x_m, y_m, heading_rad


def load_my_track_layout(path: str | Path) -> MyTrackLayout:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    cones: list[Cone] = []
    roles: dict[str, str] = {}
    for entry in payload["cones"]:
        cone_class = _COLOUR_TO_CLASS.get(entry["color"])
        if cone_class is None:
            raise SimulationError(f"unknown cone colour {entry['color']!r} in my_track layout")
        cones.append(Cone(entry["id"], float(entry["x"]), float(entry["y"]), 0.0, cone_class))
        roles[entry["id"]] = entry["role"]
    sections = {name: (float(bounds[0]), float(bounds[1])) for name, bounds in payload["sections"].items()}
    start = payload["start_pose"]
    layout = MyTrackLayout(
        int(payload["rev"]), float(payload["corridor_m"]), float(payload["length_m"]), float(payload["spacing_m"]),
        sections, tuple(cones), roles,
        (float(start["x"]), float(start["y"]), _radians(float(start["heading_deg"]))),
    )
    _validate(layout)
    return layout


def _radians(degrees: float) -> float:
    from math import radians
    return radians(degrees)


def _validate(layout: MyTrackLayout) -> None:
    if len(layout.cones) != 120:
        raise SimulationError(f"my_track layout must have exactly 120 cones, found {len(layout.cones)}")
    counts = {"blue": 0, "yellow": 0, "large_orange": 0}
    for cone in layout.cones:
        counts[cone.semantic_class.value] += 1
    expected = {"blue": 20, "yellow": 20, "large_orange": 80}
    if counts != expected:
        raise SimulationError(f"my_track layout colour counts {counts} do not match {expected}")
