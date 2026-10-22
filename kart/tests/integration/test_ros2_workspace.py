from pathlib import Path
import xml.etree.ElementTree as ET
from collections import defaultdict


ROOT = Path(__file__).parents[2]


def test_required_ros_packages_messages_launches_and_rviz_assets_exist() -> None:
    source = ROOT / "ros2_ws/src"
    packages = {
        "autokart_msgs", "autokart_description", "autokart_bringup", "autokart_gazebo",
        "autokart_sim_bridge", "autokart_control", "autokart_localization",
        "autokart_planning", "autokart_safety_bridge", "autokart_diagnostics",
        "autokart_scenarios", "autokart_mapping", "autokart_planner", "autokart_controller",
    }
    grouped_packages = {"autokart_perception", "autokart_mapping", "autokart_planner",
                        "autokart_controller"}
    package_paths = {package: source / package for package in packages - grouped_packages}
    package_paths.update({package: source / "autokart_nodes" / package
                          for package in grouped_packages})
    assert all((path / "package.xml").is_file() for path in package_paths.values())
    motion = (source / "autokart_msgs/msg/MotionCommand.msg").read_text()
    for field in (
        "source", "sequence", "timestamp", "expiry", "mode", "target_speed_mps",
        "steering_angle_rad", "curvature_1pm", "acceleration_limit_mps2",
        "deceleration_limit_mps2", "valid", "integrity",
    ):
        assert field in motion
    assert len(list((source / "autokart_description/rviz").glob("*.rviz"))) == 9
    launches = {path.name for path in (source / "autokart_bringup/launch").glob("*.launch.py")}
    assert {"gazebo_sim.launch.py", "cone_track.launch.py", "fast_sim.launch.py",
            "skidpad.launch.py", "full_stack.launch.py",
            "rc03_rc05_diagnostics.launch.py"}.issubset(launches)


def test_autokart_nodes_grouping_discovers_unique_ros_packages() -> None:
    source = ROOT / "ros2_ws/src"
    grouping = source / "autokart_nodes"
    assert grouping.is_dir()
    assert not (grouping / "package.xml").exists()
    assert not (source / "autokart_perception").exists()

    names: dict[str, list[Path]] = defaultdict(list)
    for package_xml in source.rglob("package.xml"):
        name = ET.parse(package_xml).getroot().findtext("name")
        assert name
        names[name].append(package_xml)
    assert all(len(paths) == 1 for paths in names.values())
    assert {"autokart_perception", "autokart_mapping", "autokart_planner",
            "autokart_controller"}.issubset(names)

    perception_cmake = (grouping / "autokart_perception/CMakeLists.txt").read_text()
    assert "ament_python_install_package(autokart_perception_ros)" in perception_cmake
    assert (grouping / "autokart_perception/autokart_perception_ros/__init__.py").is_file()


def test_ros_and_gazebo_bridges_have_no_direct_actuator_command_path() -> None:
    bridge = (ROOT / "ros2_ws/src/autokart_sim_bridge/autokart_sim_bridge/vcu_bridge.py").read_text()
    gazebo_bridge = (ROOT / "ros2_ws/src/autokart_gazebo/config/bridge.yaml").read_text()
    assert "direct_actuator_commands" in bridge and "prohibited" in bridge
    assert "motor_effort" not in gazebo_bridge
    assert "brake_effort" not in gazebo_bridge
    assert "steering_effort" not in gazebo_bridge

