from pathlib import Path

import pytest

from kart_sim.config import load_kart_config
from kart_sim.robot_description import autokart_frame_tree, validate_sdf, validate_urdf


ROOT = Path(__file__).parents[2]


def test_urdf_matches_config_and_has_positive_reviewed_geometry() -> None:
    report = validate_urdf(
        ROOT / "ros2_ws/src/autokart_description/urdf/autokart.urdf",
        load_kart_config(ROOT / "configs/karts/prototype_dynamic.yaml"),
    )
    assert not report.issues
    assert report.total_mass_kg == pytest.approx(145.0)
    assert report.link_count >= 22 and report.joint_count >= 20


def test_sdf_model_and_all_worlds_validate_structurally() -> None:
    model = validate_sdf(ROOT / "simulation/models/autokart/model.sdf")
    assert not model.issues
    worlds = sorted((ROOT / "simulation/worlds").glob("*.sdf"))
    assert len(worlds) == 7
    assert any(path.name == "autokart_gazebo_follower_world.sdf" for path in worlds)
    assert all(not validate_sdf(world).issues for world in worlds)


def test_tf_tree_has_namespaced_optical_and_dynamic_frames() -> None:
    tree = autokart_frame_tree("kart_01", 1.0)
    tree.validate()
    assert tree.transforms["kart_01/camera_optical_frame"].parent == "kart_01/camera_link"
    assert not tree.transforms["kart_01/base_link"].static
    assert not tree.transforms["kart_01/front_left_wheel_link"].static
