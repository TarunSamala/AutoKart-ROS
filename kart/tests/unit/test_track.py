from pathlib import Path
import pytest
from kart_sim.track import Track, segment_corners

ROOT=Path(__file__).parents[2]
def test_circle_arc_curvature_projection_and_surface() -> None:
    track=Track.from_yaml(ROOT/"configs/tracks/research_circle.yaml")
    assert track.length_m == pytest.approx(2*3.14159265*10, rel=.03)
    assert abs(track.sample(10).curvature_1pm) == pytest.approx(.1, rel=.2)
    world=track.world_from_track(8.0,1.0); projection=track.project(*world)
    # Eight-chord reference circle: projection remains bounded but is not spline-exact.
    assert projection.s_m == pytest.approx(8.0, abs=.9) and projection.lateral_m == pytest.approx(1.0,abs=.06)
    assert track.surface_at(8,-1).material.id == "wet_asphalt"
    assert track.surface_at(8,1).material.id == "dry_asphalt"
def test_split_friction_and_boundaries() -> None:
    track=Track.from_yaml(ROOT/"configs/tracks/straight_split_friction.yaml")
    assert track.surface_at(10,1).material.id == "dry_asphalt"
    assert track.surface_at(10,-1).material.id == "wet_asphalt"
    p=track.project(10,2.1); assert p.left_margin_m < 0
def test_corner_segmentation_is_deterministic() -> None:
    track=Track.from_yaml(ROOT/"configs/tracks/research_circle.yaml")
    assert segment_corners(track)==segment_corners(track)
