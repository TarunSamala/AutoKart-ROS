"""Exports a my_track run as a single JSON file for the replay viewer
(kart/viewer/index.html). Step 7 (D3) scope note: this exports what the
current telemetry actually carries -- kart pose/gauges/state and the static
cone layout. It does NOT yet carry gates, Delaunay/Gabriel edges, the planned
path, or swept footprint per frame (MissionCommand doesn't expose those
internals out to the runner yet), so the viewer's corresponding overlays are
stubs. See docs/MY_TRACK.md "what is NOT done" for the real list.
"""

from __future__ import annotations

import json
from pathlib import Path

from .my_track_layout import MyTrackLayout
from .my_track_runner import RunResult


def export_replay(result: RunResult, layout: MyTrackLayout, *, profile: str, seed: int,
                   front_extent_m: float, rear_overhang_m: float, half_width_m: float,
                   camera_hfov_deg: float, camera_range_m: float, path: Path) -> Path:
    payload = {
        "scenario": "my_track",
        "profile": profile,
        "seed": seed,
        "metrics": {
            "outcome": result.metrics.outcome,
            "collisions": result.metrics.collisions,
            "min_clearance_m": result.metrics.min_clearance_m,
            "max_speed_mps": result.metrics.max_speed_mps,
            "max_steer_deg": result.metrics.max_steer_deg,
            "stop_gap_to_finish_m": result.metrics.stop_gap_to_finish_m,
        },
        "kart_geometry": {
            "front_extent_m": front_extent_m, "rear_overhang_m": rear_overhang_m, "half_width_m": half_width_m,
        },
        "camera": {"hfov_deg": camera_hfov_deg, "range_m": camera_range_m},
        "cones": [
            {"id": cone.cone_id, "x": cone.x_m, "y": cone.y_m, "colour": cone.semantic_class.value,
             "role": layout.roles.get(cone.cone_id, "")}
            for cone in layout.cones
        ],
        "events": [{"t": t, "kind": kind, "detail": detail} for t, kind, detail in result.events],
        "frames": result.telemetry_rows,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path
