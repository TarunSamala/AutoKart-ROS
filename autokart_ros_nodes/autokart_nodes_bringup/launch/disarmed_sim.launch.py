"""disarmed_sim.launch.py — full-graph bringup for the autokart_ros_nodes library.

Starts all 12 node packages plus the two small adapters this integration package adds
(``safety_armed_bridge``, ``mapping_path_bridge``), wired per each node's own CONTRACT.md so that
real outputs feed real inputs end to end:

    01 hp60c -> 02 perception_adapter -> 06 mapping -+-> 07 mission (diagnostic, parallel)
                                                       `-> [mapping_path_bridge] -> 08 planner
    08 planner -> 09 controller -> 04 vcu_bridge
    10 safety --[safety_armed_bridge]--> 04 vcu_bridge (safety_armed Bool)
    03 imu: standalone (CONTRACT.md: autokart_odometry explicitly does not fuse IMU data; there is
            no topic to wire it into any other node in this library)
    04 feedback -> 05 odometry
    11 dashboard_gateway, 12 system_monitor: subscribe broadly, watching everything

Every backend is synthetic/standalone/builtin -- no camera, no I2C, no serial port, no network
socket beyond localhost is opened. ``calibration_verified``/``vehicle_config_verified``/
``braking_verified`` are left at their honest default ``false`` (node 10's own config), and nothing
in this launch file ever publishes an ``arm`` request on ``/autokart/safety/request`` -- the graph
is DISARMED by construction and stays that way for the life of this launch, per node 10's own
"default-DISARMED, never-auto-arm" contract. See README.md in this package for the two genuine,
pre-existing integration gaps this launch file does NOT paper over (node 10's perception/planner/
controller/localization freshness inputs, which cannot be wired without inventing data no
node actually computes).
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

AUTOKART_ROOT = Path(__file__).resolve().parents[2]
AUTONOMY_PERCEPTION_SRC = AUTOKART_ROOT.parent / "autonomy_stack_phase1" / "phase_01_perception" / "src"
KART_WS_SRC = AUTOKART_ROOT.parent / "kart" / "ros2_ws" / "src"


def node_config(package: str, filename: str) -> str:
    return str(AUTOKART_ROOT / package / "config" / filename)


def generate_launch_description():
    algorithm_package_path_arg = DeclareLaunchArgument(
        "algorithm_package_path",
        default_value=str(AUTONOMY_PERCEPTION_SRC),
        description="Path to the autokart_perception algorithm package's src/ directory "
                    "(used by mapping and mission for real Delaunay/Gabriel/centerline/skidpad "
                    "geometry instead of a dependency-unavailable fault).",
    )

    nodes = [
        # --- 01 hp60c: synthetic camera, standalone ---
        Node(
            package="autokart_hp60c", executable="hp60c_node", name="autokart_hp60c",
            output="screen",
            parameters=[node_config("autokart_hp60c", "hp60c_synthetic.yaml")],
        ),

        # --- 02 perception_adapter: synthetic backend, but wired to node 01's real topics
        #     (input_mode=topics, not standalone_timer) to actually exercise 01 -> 02 ---
        Node(
            package="autokart_perception_adapter", executable="perception_adapter_node",
            name="autokart_perception_adapter", output="screen",
            parameters=[
                node_config("autokart_perception_adapter", "synthetic.yaml"),
                {"input_mode": "topics"},
            ],
        ),

        # --- 03 imu: standalone. No topic in this library accepts IMU data (see module docstring)
        Node(
            package="autokart_imu", executable="imu_node", name="autokart_imu", output="screen",
            parameters=[node_config("autokart_imu", "imu_synthetic.yaml")],
        ),

        # --- 04 vcu_bridge: synthetic VCU, real reused control-law core. safety_armed_topic is
        #     left at its documented default (the safety bridge below publishes to exactly that
        #     name). candidate_topic's own default does NOT match node 09's real output -- see
        #     the override below. ---
        Node(
            package="autokart_vcu_bridge", executable="vcu_bridge_node", name="autokart_vcu_bridge",
            output="screen",
            parameters=[
                node_config("autokart_vcu_bridge", "vcu_bridge_synthetic.yaml"),
                {
                    "algorithm_package_path": str(KART_WS_SRC / "autokart_vehicle_bridge"),
                    # candidate_topic's own default ("/autokart/controller/motion_command_candidate")
                    # does NOT match node 09's actual, hardcoded output topic
                    # ("/autokart_controller/motion_command_candidate") -- found while wiring this
                    # graph, not previously documented in either node's CONTRACT.md.
                    "candidate_topic": "/autokart_controller/motion_command_candidate",
                },
            ],
        ),

        # --- 05 odometry: wired to 04's real actuator feedback (not the internal synthetic
        #     generator). Publishes on the hardcoded /odom; remapped to /autokart/odom, the
        #     odom_topic default every downstream node (06/07/08/09) already expects. ---
        Node(
            package="autokart_odometry", executable="odometry_node", name="autokart_odometry",
            output="screen",
            parameters=[
                node_config("autokart_odometry", "odometry_synthetic.yaml"),
                {
                    "source": "actuator_feedback",
                    "algorithm_package_path": str(KART_WS_SRC / "autokart_localization"),
                    # ActuatorFeedback carries no raw axle counts (see CONTRACT.md "Displacement
                    # semantics"); counts_per_m must be 0 (speed mode) for this source, overriding
                    # the synthetic profile's counts-mode test value of 100.0.
                    "counts_per_m": 0.0,
                },
            ],
            remappings=[
                ("/odom", "/autokart/odom"),
                ("/odom/status", "/autokart/odometry/status"),
            ],
        ),

        # --- 06 mapping: cones_topic remapped from its own default (/autokart_mapping/cones_in)
        #     to node 02's real output (/perception/cones). odom_topic default already matches
        #     05's remapped output, no remap needed there. ---
        Node(
            package="autokart_mapping", executable="mapping_node", name="autokart_mapping",
            output="screen",
            parameters=[
                node_config("autokart_mapping", "mapping.yaml"),
                {
                    "algorithm_package_path": LaunchConfiguration("algorithm_package_path"),
                    "source_mode": "SIMULATED",
                },
            ],
            remappings=[("/autokart_mapping/cones_in", "/perception/cones")],
        ),

        # --- 07 mission: every default topic (mapping_status_topic, odom_topic, feedback_topic)
        #     already matches an upstream node's real output name -- no remap needed. Runs as a
        #     parallel diagnostic consumer of 06 (see module docstring: it has no path output to
        #     feed forward to 08, by its own CONTRACT.md). ---
        Node(
            package="autokart_mission", executable="mission_node", name="autokart_mission",
            output="screen",
            parameters=[
                node_config("autokart_mission", "mission_skidpad.yaml"),
                {
                    "algorithm_package_path": LaunchConfiguration("algorithm_package_path"),
                    "vehicle_geometry_package_path": str(KART_WS_SRC / "autokart_vehicle_geometry"),
                    "planner_package_path": str(KART_WS_SRC / "autokart_nodes" / "autokart_planner"),
                },
            ],
        ),

        # --- [mapping_path_bridge]: 06's centerline_odom geometry -> 08's reference_path_topic.
        #     See mapping_path_bridge.py module docstring for why this adapter exists. ---
        Node(
            package="autokart_nodes_bringup", executable="mapping_path_bridge",
            name="mapping_path_bridge", output="screen",
        ),

        # --- 08 planner: builtin backend (zero dependency). odom_topic and reference_path_topic
        #     defaults already match 05's remap and the bridge's output -- no remap needed.
        #     a_brake_min_mps2/t_total_latency_s are given small, explicitly-labelled SIMULATION
        #     TEST VALUES (not measured) so the stop-envelope logic actually runs instead of
        #     failing closed on "unconfigured", per CONTRACT.md's own documented default. ---
        Node(
            package="autokart_planner", executable="planner_node", name="autokart_planner",
            output="screen",
            parameters=[
                node_config("autokart_planner", "planner_builtin.yaml"),
                {
                    # SIMULATION TEST VALUES, not measured on the real kart -- see CONTRACT.md's
                    # own "Physical constants" table, which fails closed on the real 0.0 defaults.
                    "a_brake_min_mps2": 2.0,
                    "t_total_latency_s": 0.1,
                },
            ],
        ),

        # --- 09 controller: builtin pursuit backend. Its subscribed-topic defaults match 08's
        #     and 04's real outputs; its own output topic is fixed (not a parameter) and is
        #     pointed at by 04's candidate_topic override above -- no remap needed on this node
        #     itself. ---
        Node(
            package="autokart_controller", executable="controller_node", name="autokart_controller",
            output="screen",
            parameters=[node_config("autokart_controller", "controller_builtin.yaml")],
        ),

        # --- 10 safety: real reused autokart_vcu_safety core. Only the mapping status input can
        #     genuinely be wired (same MissionDebug type); perception/planner/controller/
        #     localization inputs have no type-matching publisher anywhere in this graph -- see
        #     README.md "Known, unresolved gaps". Never receives an "arm" request: stays DISARMED.
        Node(
            package="autokart_safety", executable="safety_node", name="autokart_safety",
            output="screen",
            parameters=[
                node_config("autokart_safety", "safety_synthetic.yaml"),
                {"vcu_safety_package_path": str(KART_WS_SRC / "autokart_vcu_safety")},
            ],
            remappings=[("/autokart/mapping/status", "/autokart_mapping/status")],
        ),

        # --- [safety_armed_bridge]: 10's MissionDebug ARMED state -> 04's safety_armed Bool.
        #     See safety_armed_bridge.py module docstring for why this adapter exists. ---
        Node(
            package="autokart_nodes_bringup", executable="safety_armed_bridge",
            name="safety_armed_bridge", output="screen",
        ),

        # --- 11 dashboard_gateway: live mode (so it actually subscribes and "watches"), but
        #     display_only=true inhibits BOTH of its outbound request sinks -- it can observe
        #     the whole graph and can never influence it. ---
        Node(
            package="autokart_dashboard_gateway", executable="dashboard_gateway_node",
            name="autokart_dashboard_gateway", output="screen",
            parameters=[
                node_config("autokart_dashboard_gateway", "dashboard_gateway_synthetic.yaml"),
                {
                    "mode": "live",
                    "display_only": True,
                    "gateway_package_path": str(KART_WS_SRC / "autokart_dashboard_gateway"),
                    "perception_package_path": LaunchConfiguration("algorithm_package_path"),
                    "port": 8766,
                },
            ],
        ),

        # --- 12 system_monitor: heartbeat_topics lists every topic this launch file expects to
        #     be flowing, exactly the set the integration test checks via /system/health. ---
        Node(
            package="autokart_system_monitor", executable="system_monitor_node",
            name="autokart_system_monitor", output="screen",
            parameters=[
                node_config("autokart_system_monitor", "system_monitor_synthetic.yaml"),
                {
                    "heartbeat_topics": [
                        "/hp60c/status",
                        "/perception/status",
                        "/imu/status",
                        "/vcu_bridge/status",
                        "/autokart/odometry/status",
                        "/autokart_mapping/status",
                        "/autokart_mission/status",
                        "/autokart_planner/safety_envelope",
                        "/autokart_controller/diagnostics",
                        "/autokart/safety/status",
                        "/vcu/state",
                        "/vehicle/actuator_feedback",
                    ],
                    "heartbeat_timeout_s": 2.0,
                },
            ],
        ),
    ]

    return LaunchDescription([algorithm_package_path_arg, *nodes])
