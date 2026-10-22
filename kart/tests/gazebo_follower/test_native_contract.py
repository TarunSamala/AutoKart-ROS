from pathlib import Path
from xml.etree import ElementTree as ET


ROOT = Path(__file__).parents[2]


def test_native_follower_plugins_have_no_command_or_dynamics_subscription() -> None:
    source = (
        ROOT / "ros2_ws/src/autokart_gazebo/src/gazebo_follower_plugin.cpp"
    ).read_text(encoding="utf-8")
    assert "/autokart/follower/authoritative_snapshot" in source
    assert "SetWorldPose" in source
    assert "SetPosition" in source
    assert "create_subscription<Snapshot>" in source
    assert "MotionCommand" not in source
    assert "SetForce" not in source
    assert "SetParam(\"fmax\"" not in source


def test_sdf_is_static_and_contains_only_read_only_model_plugin() -> None:
    root = ET.parse(ROOT / "simulation/models/autokart/model.sdf").getroot()
    model = root.find("model")
    assert model is not None
    assert model.findtext("static") == "true"
    model_plugins = model.findall("plugin")
    assert [plugin.get("filename") for plugin in model_plugins] == [
        "libautokart_gazebo_follower.so"
    ]
    assert model.find(".//plugin[@name='autokart_rgb_delivery']") is not None
    assert model.find(".//plugin[@name='autokart_depth_delivery']") is not None
    assert model.find(".//plugin[@name='autokart_lidar_delivery']") is not None


def test_gazebo_bridge_does_not_duplicate_internal_clock() -> None:
    bridge = (
        ROOT / "ros2_ws/src/autokart_gazebo/config/bridge.yaml"
    ).read_text(encoding="utf-8")
    assert "gz_topic_name: /clock" not in bridge
    follower = (
        ROOT / "ros2_ws/src/autokart_gazebo/src/gazebo_follower_plugin.cpp"
    ).read_text(encoding="utf-8")
    assert '"/clock"' in follower
    assert 'Transform("odom", "base_link"' in follower
