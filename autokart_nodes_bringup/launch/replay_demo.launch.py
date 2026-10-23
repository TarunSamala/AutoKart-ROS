"""replay_demo.launch.py — OPTIONAL end-to-end replay demo (phase 6, step 4).

Identical downstream graph to ``disarmed_sim.launch.py`` (06 mapping -> [bridge] -> 08 planner ->
09 controller -> 10 safety/04 vcu_bridge, still permanently DISARMED), but node 01
(``autokart_hp60c``) is replaced by ``replay_publisher`` (see that module's docstring for why: node
01 itself has no "replay a recording" backend) feeding real recorded HP60C frames, and node 02
(``autokart_perception_adapter``) runs its real ``algorithm`` backend (the actual
``autokart_perception`` Delaunay/YOLO/qualification pipeline, ``source_label="REPLAY"``) instead of
the fixed synthetic 4-cone layout. Everything downstream is unchanged and still synthetic/builtin.

Labelled REPLAY end to end: 01(replay) -> 02(REPLAY) -> 06(REPLAY) -> 07(REPLAY) -> 08 -> 09 -> STOP
candidate, still never armed.

Default recording: ``autonomy_stack_phase1/phase_01_perception/recordings/hp60c-bench-20261008-03``
(20 numbered frame/metadata pairs, the only 1-indexed-contiguous recording available in this
environment that the mission statement's recordings/ directory listing points at).
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

AUTOKART_ROOT = Path(__file__).resolve().parents[2]
AUTONOMY_PERCEPTION_SRC = AUTOKART_ROOT.parent / "autonomy_stack_phase1" / "phase_01_perception" / "src"
DEFAULT_RECORDING = (AUTOKART_ROOT.parent / "autonomy_stack_phase1" / "phase_01_perception" /
                      "recordings" / "hp60c-bench-20261008-03")
KART_WS_SRC = AUTOKART_ROOT.parent / "kart" / "ros2_ws" / "src"


def node_config(package: str, filename: str) -> str:
    return str(AUTOKART_ROOT / package / "config" / filename)


def generate_launch_description():
    algorithm_package_path_arg = DeclareLaunchArgument(
        "algorithm_package_path", default_value=str(AUTONOMY_PERCEPTION_SRC),
    )
    recording_path_arg = DeclareLaunchArgument(
        "recording_path", default_value=str(DEFAULT_RECORDING),
        description="Directory of numbered <NNNNNN>.json/.npz frame pairs "
                    "(autokart_perception.recording.Replay format) to replay as node 01's output.",
    )

    nodes = [
        Node(
            package="autokart_nodes_bringup", executable="replay_publisher", name="replay_publisher",
            output="screen",
            parameters=[{
                "algorithm_package_path": LaunchConfiguration("algorithm_package_path"),
                "recording_path": LaunchConfiguration("recording_path"),
                "rate_hz": 2.0,
            }],
        ),
        Node(
            package="autokart_perception_adapter", executable="perception_adapter_node",
            name="autokart_perception_adapter", output="screen",
            parameters=[{
                "backend": "algorithm",
                "input_mode": "topics",
                "source_label": "REPLAY",
                "algorithm_package_path": LaunchConfiguration("algorithm_package_path"),
            }],
        ),
        Node(
            package="autokart_imu", executable="imu_node", name="autokart_imu", output="screen",
            parameters=[node_config("autokart_imu", "imu_synthetic.yaml")],
        ),
        Node(
            package="autokart_vcu_bridge", executable="vcu_bridge_node", name="autokart_vcu_bridge",
            output="screen",
            parameters=[
                node_config("autokart_vcu_bridge", "vcu_bridge_synthetic.yaml"),
                {
                    "algorithm_package_path": str(KART_WS_SRC / "autokart_vehicle_bridge"),
                    "candidate_topic": "/autokart_controller/motion_command_candidate",
                },
            ],
        ),
        Node(
            package="autokart_odometry", executable="odometry_node", name="autokart_odometry",
            output="screen",
            parameters=[
                node_config("autokart_odometry", "odometry_synthetic.yaml"),
                {
                    "source": "actuator_feedback",
                    "algorithm_package_path": str(KART_WS_SRC / "autokart_localization"),
                    "counts_per_m": 0.0,
                },
            ],
            remappings=[("/odom", "/autokart/odom"), ("/odom/status", "/autokart/odometry/status")],
        ),
        Node(
            package="autokart_mapping", executable="mapping_node", name="autokart_mapping",
            output="screen",
            parameters=[
                node_config("autokart_mapping", "mapping.yaml"),
                {
                    "algorithm_package_path": LaunchConfiguration("algorithm_package_path"),
                    "source_mode": "REPLAY",
                },
            ],
            remappings=[("/autokart_mapping/cones_in", "/perception/cones")],
        ),
        Node(
            package="autokart_mission", executable="mission_node", name="autokart_mission",
            output="screen",
            parameters=[
                node_config("autokart_mission", "mission_skidpad.yaml"),
                {
                    "algorithm_package_path": LaunchConfiguration("algorithm_package_path"),
                    "vehicle_geometry_package_path": str(KART_WS_SRC / "autokart_vehicle_geometry"),
                    "planner_package_path": str(KART_WS_SRC / "autokart_nodes" / "autokart_planner"),
                    "source": "REPLAY",
                },
            ],
        ),
        Node(
            package="autokart_nodes_bringup", executable="mapping_path_bridge",
            name="mapping_path_bridge", output="screen",
        ),
        Node(
            package="autokart_planner", executable="planner_node", name="autokart_planner",
            output="screen",
            parameters=[
                node_config("autokart_planner", "planner_builtin.yaml"),
                {"a_brake_min_mps2": 2.0, "t_total_latency_s": 0.1, "source": "REPLAY"},
            ],
        ),
        Node(
            package="autokart_controller", executable="controller_node", name="autokart_controller",
            output="screen",
            parameters=[
                node_config("autokart_controller", "controller_builtin.yaml"),
                {"source": "REPLAY"},
            ],
        ),
        Node(
            package="autokart_safety", executable="safety_node", name="autokart_safety",
            output="screen",
            parameters=[
                node_config("autokart_safety", "safety_synthetic.yaml"),
                {"vcu_safety_package_path": str(KART_WS_SRC / "autokart_vcu_safety")},
            ],
            remappings=[("/autokart/mapping/status", "/autokart_mapping/status")],
        ),
        Node(
            package="autokart_nodes_bringup", executable="safety_armed_bridge",
            name="safety_armed_bridge", output="screen",
        ),
        Node(
            package="autokart_system_monitor", executable="system_monitor_node",
            name="autokart_system_monitor", output="screen",
            parameters=[
                node_config("autokart_system_monitor", "system_monitor_synthetic.yaml"),
                {
                    "heartbeat_topics": [
                        "/hp60c/status", "/perception/status", "/autokart_mapping/status",
                        "/autokart_mission/status", "/autokart_planner/safety_envelope",
                        "/autokart_controller/diagnostics", "/autokart/safety/status",
                    ],
                },
            ],
        ),
    ]
    return LaunchDescription([algorithm_package_path_arg, recording_path_arg, *nodes])
