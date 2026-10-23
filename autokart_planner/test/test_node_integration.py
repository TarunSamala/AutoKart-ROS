"""Integration test: real rclpy node, real pub/sub, builtin algorithm backend.

Requires a sourced ROS 2 Humble environment (rclpy, nav_msgs, geometry_msgs, autokart_msgs).
"""
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from nav_msgs.msg import Odometry, Path  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from autokart_msgs.msg import SafetyEnvelope  # noqa: E402

from autokart_planner.node import PlannerNode, ros_time  # noqa: E402


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


def _straight_path_msg(stamp_ns, n=11, step=1.0):
    msg = Path()
    msg.header.stamp = ros_time(stamp_ns)
    msg.header.frame_id = "odom"
    for i in range(n):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = float(i) * step
        pose.pose.position.y = 0.0
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
    return msg


def _odom_msg(stamp_ns, speed_mps=0.5):
    msg = Odometry()
    msg.header.stamp = ros_time(stamp_ns)
    msg.header.frame_id = "odom"
    msg.child_frame_id = "base_link"
    msg.pose.pose.position.x = 0.0
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = speed_mps
    return msg


def test_planner_node_publishes_path_envelope_and_debug_for_a_clear_straight_path(ros_context):
    from rclpy.parameter import Parameter

    node = PlannerNode(parameter_overrides=[
        Parameter('a_brake_min_mps2', Parameter.Type.DOUBLE, 2.0),
        Parameter('t_total_latency_s', Parameter.Type.DOUBLE, 0.2),
        Parameter('speed_limit_mps', Parameter.Type.DOUBLE, 1.0),
    ])

    received = {"path": None, "envelope": None, "debug": None}
    sub_node = rclpy.create_node("test_subscriber_planner")
    sub_node.create_subscription(Path, "/autokart_planner/path", lambda m: received.__setitem__("path", m), 10)
    sub_node.create_subscription(SafetyEnvelope, "/autokart_planner/safety_envelope",
                                 lambda m: received.__setitem__("envelope", m), 10)
    sub_node.create_subscription(String, "/autokart_planner/speed_profile",
                                 lambda m: received.__setitem__("debug", m), 10)
    odom_pub = sub_node.create_publisher(Odometry, "/autokart/odom", 10)
    path_pub = sub_node.create_publisher(Path, "/autokart_planner/reference_path", 10)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        stamp_ns = node.get_clock().now().nanoseconds
        odom_pub.publish(_odom_msg(stamp_ns, speed_mps=0.5))
        time.sleep(0.1)
        path_pub.publish(_straight_path_msg(stamp_ns))

        assert _wait_for(lambda: received["envelope"] is not None and received["envelope"].valid)
        assert len(received["path"].poses) == 11
        assert received["envelope"].stopping_distance_m > 0.0
        import json
        payload = json.loads(received["debug"].data)
        assert payload["stage"] == "PLANNING"
        assert payload["valid"] is True
        assert len(payload["speed_profile_mps"]) == 11
        assert payload["vehicle_authorized"] is False
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()
