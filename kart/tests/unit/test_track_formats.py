from json import loads
from pathlib import Path

from kart_sim.track import Track
from kart_sim.track_formats import (
    cone_layout,
    dashboard_geometry,
    detect_self_intersections,
    gazebo_world_sdf,
    occupancy_map,
    validate_track_source,
)


ROOT = Path(__file__).parents[2]


def test_one_source_track_conversions_are_deterministic_and_safe() -> None:
    path = ROOT / "configs/tracks/cone_track.yaml"
    track = Track.from_yaml(path)
    report = validate_track_source(path)
    assert not report["issues"] and not detect_self_intersections(track)
    assert dashboard_geometry(track) == dashboard_geometry(track)
    assert cone_layout(track)["ground_truth_only"]
    pgm, metadata = occupancy_map(track, resolution_m=0.5)
    assert pgm.startswith("P2\n") and not metadata["unknown_is_free"]
    world = gazebo_world_sdf(track)
    assert "<world" in world and "diff_drive" not in world and "ackermann" not in world

