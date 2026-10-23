"""Integration test: real rclpy node, real pub/sub, builtin pursuit backend.

Requires a sourced ROS 2 Humble environment (rclpy, nav_msgs, std_msgs, autokart_msgs).
Feeds the node exactly the message shapes autokart_planner (node 08) publishes.
"""
import json
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from nav_msgs.msg import Odometry, Path  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from autokart_msgs.msg import MotionCommand, SafetyEnvelope  # noqa: E402

from autokart_controller.node import ControllerNode, ros_time  # noqa: E402


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


def path_msg(stamp_ns, points):
    msg = Path()
    msg.header.stamp = ros_time(stamp_ns)
    msg.header.frame_id = "odom"
    for x, y in points:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
    return msg


def envelope_msg(stamp_ns, valid=True, max_speed=1.0):
    msg = SafetyEnvelope()
    msg.timestamp = ros_time(stamp_ns)
    msg.frame_id = "odom"
    msg.valid = valid
    msg.maximum_speed_mps = max_speed
    return msg


def odom_msg(stamp_ns, x=0.0, y=0.0, speed=0.5):
    msg = Odometry()
    msg.header.stamp = ros_time(stamp_ns)
    msg.header.frame_id = "odom"
    msg.child_frame_id = "base_link"
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = speed
    return msg


def test_controller_publishes_valid_candidate_for_fresh_planner_output(ros_context):
    node = ControllerNode()
    received = {"cmd": None, "diag": None}
    sub = rclpy.create_node("test_sub_controller")
    sub.create_subscription(MotionCommand, "/autokart_controller/motion_command_candidate",
                            lambda m: received.__setitem__("cmd", m), 10)
    sub.create_subscription(String, "/autokart_controller/diagnostics",
                            lambda m: received.__setitem__("diag", m), 10)
    odom_pub = sub.create_publisher(Odometry, "/autokart/odom", 10)
    env_pub = sub.create_publisher(SafetyEnvelope, "/autokart_planner/safety_envelope", 10)
    prof_pub = sub.create_publisher(String, "/autokart_planner/speed_profile", 10)
    path_pub = sub.create_publisher(Path, "/autokart_planner/path", 10)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        time.sleep(0.3)  # discovery
        stamp = node.get_clock().now().nanoseconds
        points = [(float(i), 0.0) for i in range(11)]
        odom_pub.publish(odom_msg(stamp, y=-0.5))
        env_pub.publish(envelope_msg(stamp, valid=True, max_speed=1.0))
        prof_pub.publish(String(data=json.dumps({"sample_time_ns": stamp,
                                                 "speed_profile_mps": [0.8] * 11})))
        time.sleep(0.2)
        # Odometry streams continuously in reality; re-publish so it is inside the 150 ms
        # pose-freshness gate when the path arrives (a single odom 0.2 s earlier is correctly
        # rejected as stale -- that was this test's own first-draft bug, not a node bug).
        odom_pub.publish(odom_msg(stamp, y=-0.5))
        time.sleep(0.02)
        path_pub.publish(path_msg(stamp, points))
        assert _wait_for(lambda: received["cmd"] is not None and received["cmd"].valid)
        cmd = received["cmd"]
        assert cmd.steering_angle_rad > 0.0  # path is to the left of the offset kart
        assert 0.0 < cmd.target_speed_mps <= 0.8 + 1e-9
        assert cmd.source.startswith("autokart_controller/")
        assert _wait_for(lambda: received["diag"] is not None and json.loads(received["diag"].data)["valid"])
        diag = json.loads(received["diag"].data)
        assert diag["vehicle_authorized"] is False and diag["published_to_vcu"] is False
        assert diag["cross_track_error_m"] == pytest.approx(0.5, abs=1e-6)
        assert diag["heading_error_rad"] is not None
        assert diag["speed_tracking_error_mps"] is not None
    finally:
        executor.shutdown()
        node.destroy_node()
        sub.destroy_node()
