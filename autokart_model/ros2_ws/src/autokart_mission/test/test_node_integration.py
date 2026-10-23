"""Integration test: real rclpy node, real pub/sub, real reused PlannerEngine.

Requires a sourced ROS 2 Humble environment (rclpy, nav_msgs, autokart_msgs) and the full
kart/ros2_ws + autonomy_stack_phase1 checkout (the three external lazy-import dependencies; see
DEPENDENCIES.md). Run via: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
"""
import json
import math
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from nav_msgs.msg import Odometry
from std_msgs.msg import String  # noqa: E402
from autokart_msgs.msg import MissionDebug  # noqa: E402

from autokart_mission.node import MissionNode, ros_time  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[3]
ALGORITHM_PATH = REPO_ROOT / "autonomy_stack_phase1" / "phase_01_perception" / "src"
VEHICLE_GEOMETRY_PATH = REPO_ROOT / "kart" / "ros2_ws" / "src" / "autokart_vehicle_geometry"
PLANNER_PATH = REPO_ROOT / "kart" / "ros2_ws" / "src" / "autokart_nodes" / "autokart_planner"
ALL_PRESENT = ALGORITHM_PATH.is_dir() and VEHICLE_GEOMETRY_PATH.is_dir() and PLANNER_PATH.is_dir()

pytestmark = pytest.mark.skipif(not ALL_PRESENT, reason="full kart/autonomy_stack_phase1 checkout not present")


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _wait_for(predicate, timeout_s=5.0, step_s=0.05):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(step_s)
    return False


def _skidpad_topology():
    import sys
    sys.path.insert(0, str(ALGORITHM_PATH))
    from autokart_perception.shared_core import fit_skidpad_topology
    cones = []
    for cx, blue, yellow in ((-9.125, 7.625, 10.625), (9.125, 10.625, 7.625)):
        for label, radius in (("blue", blue), ("yellow", yellow)):
            for index in range(24):
                angle = 2 * math.pi * index / 24
                cones.append({"x_m": cx + radius * math.cos(angle),
                              "y_m": radius * math.sin(angle), "label": label, "sigma_m": 0.05})
    topology = fit_skidpad_topology(cones)
    assert topology["valid"]
    return topology


def test_mission_node_publishes_path_candidate_on_the_right_loop(ros_context):
    node = MissionNode(parameter_overrides=[
        Parameter('algorithm_package_path', Parameter.Type.STRING, str(ALGORITHM_PATH)),
        Parameter('vehicle_geometry_package_path', Parameter.Type.STRING, str(VEHICLE_GEOMETRY_PATH)),
        Parameter('planner_package_path', Parameter.Type.STRING, str(PLANNER_PATH)),
        Parameter('mission', Parameter.Type.STRING, 'SKIDPAD'),
        Parameter('brake_min_mps2', Parameter.Type.DOUBLE, 3.0),
        Parameter('brake_delay_s', Parameter.Type.DOUBLE, 0.2),
        Parameter('brake_provenance', Parameter.Type.STRING, 'SIMULATED'),
    ])
    assert node.algorithm is not None, node.startup_error

    received = {"status": None}
    sub_node = rclpy.create_node("test_subscriber_mission")
    sub_node.create_subscription(
        MissionDebug, "/autokart_mission/status", lambda m: received.__setitem__("status", m), 10)
    odom_pub = sub_node.create_publisher(Odometry, "/odom", qos_profile_sensor_data)
    odom_status_pub = sub_node.create_publisher(String, "/vehicle/odometry_status", 10)
    mapping_pub = sub_node.create_publisher(MissionDebug, "/autokart_mapping/status", 10)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        stamp = node.get_clock().now().nanoseconds
        angle = math.pi
        x, y = 9.125 + 9.125 * math.cos(angle), 9.125 * math.sin(angle)
        heading = angle - math.pi / 2

        odom = Odometry()
        odom.header.stamp = ros_time(stamp)
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z = math.sin(heading / 2)
        odom.pose.pose.orientation.w = math.cos(heading / 2)
        odom.pose.covariance[0] = odom.pose.covariance[7] = 0.0025
        odom.twist.twist.linear.x = 1.0

        mapping_status = MissionDebug()
        mapping_status.measurement_time = ros_time(stamp)
        mapping_status.frame_id = "odom"
        mapping_status.stage = "MAPPING"
        payload = {"schema_version": 1, "source": "SIMULATED", "valid": True, "frame_id": "odom",
                   "sample_time_ns": stamp, "expires_ns":stamp+250_000_000, "tracked_count": 96,
                   "geometry": {"skidpad_topology": _skidpad_topology()}}
        mapping_status.payload_json = json.dumps(payload)

        assert _wait_for(lambda: sub_node.count_subscribers("/odom") >= 1, timeout_s=3.0)
        odom_pub.publish(odom)
        odom_status_pub.publish(String(data=__import__("json").dumps(dict(schema_version=1,source="SIMULATED",valid=True,continuity_lost=False,sample_time_ns=stamp,expires_ns=stamp+250_000_000))))
        time.sleep(0.1)
        mapping_pub.publish(mapping_status)

        assert _wait_for(lambda: received["status"] is not None, timeout_s=5.0)
        result = json.loads(received["status"].payload_json)
        assert result["mission"] == "SKIDPAD"
        assert result["valid"] is True, result
        assert result["decision"] == "PATH_CANDIDATE"
        assert result["mission_constraints"]["allowable_branch"] == "RIGHT_1"
        assert result["mission_constraints"]["speed_cap_mps"] > 0
        assert result["vehicle_authorized"] is False
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()
