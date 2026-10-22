from pathlib import Path

import pytest

from kart_sim.my_track_layout import load_my_track_layout
from kart_sim.types import SimulationError

ROOT = Path(__file__).parents[2]
LAYOUT_PATH = ROOT.parents[0] / "docs/my_track_spec/my_track_layout.json"


def test_layout_loads_with_120_cones_in_the_specified_colour_split() -> None:
    layout = load_my_track_layout(LAYOUT_PATH)
    assert layout.revision == 7
    assert layout.corridor_m == 3.0
    assert layout.length_m == 50.0
    assert layout.spacing_m == 1.0
    assert len(layout.cones) == 120
    counts = {"blue": 0, "yellow": 0, "large_orange": 0}
    for cone in layout.cones:
        counts[cone.semantic_class.value] += 1
    assert counts == {"blue": 20, "yellow": 20, "large_orange": 80}


def test_layout_has_obstacle_clusters_near_stations_18_and_38() -> None:
    layout = load_my_track_layout(LAYOUT_PATH)
    obstacle_ids = [cone_id for cone_id, role in layout.roles.items() if role == "obstacle"]
    assert len(obstacle_ids) == 12
    by_id = {cone.cone_id: cone for cone in layout.cones}
    stations_18 = [by_id[cid].x_m for cid in obstacle_ids if cid.startswith("OB1-")]
    stations_38 = [by_id[cid].x_m for cid in obstacle_ids if cid.startswith("OB2-")]
    assert len(stations_18) == 6 and len(stations_38) == 6
    assert all(17.0 < x < 20.0 for x in stations_18)
    assert all(37.0 < x < 40.0 for x in stations_38)


def test_layout_rejects_wrong_cone_count(tmp_path: Path) -> None:
    import json
    payload = json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))
    payload["cones"].pop()
    target = tmp_path / "bad_layout.json"
    target.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(SimulationError, match="120 cones"):
        load_my_track_layout(target)
