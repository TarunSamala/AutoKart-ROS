"""Fault-injection tests: missing/stale odom, invalid path, and an unavailable algorithm
dependency must all produce a STOP/invalid candidate -- this node must never publish a stale or
fabricated-valid path, speed profile, or safety envelope.
"""
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from nav_msgs.msg import Path  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
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


def _path_msg(stamp_ns, points):
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


def _spin(node, extra_nodes=()):
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    for n in extra_nodes:
        executor.add_node(n)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return executor, thread


def test_missing_odom_never_publishes_a_valid_envelope(ros_context):
    from rclpy.parameter import Parameter
    node = PlannerNode(parameter_overrides=[
        Parameter('a_brake_min_mps2', Parameter.Type.DOUBLE, 2.0),
    ])
    received = {"envelope": None}
    sub = rclpy.create_node("test_sub_missing_odom")
    sub.create_subscription(SafetyEnvelope, "/autokart_planner/safety_envelope",
                            lambda m: received.__setitem__("envelope", m), 10)
    path_pub = sub.create_publisher(Path, "/autokart_planner/reference_path", 10)
    executor, thread = _spin(node, [sub])
    try:
        stamp_ns = node.get_clock().now().nanoseconds
        path_pub.publish(_path_msg(stamp_ns, [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]))
        assert _wait_for(lambda: received["envelope"] is not None)
        assert received["envelope"].valid is False
    finally:
        executor.shutdown()
        node.destroy_node()
        sub.destroy_node()


def test_stale_odom_rejects_even_after_an_earlier_valid_cycle(ros_context):
    from rclpy.parameter import Parameter
    from nav_msgs.msg import Odometry
    node = PlannerNode(parameter_overrides=[
        Parameter('a_brake_min_mps2', Parameter.Type.DOUBLE, 2.0),
        Parameter('odom_max_age_s', Parameter.Type.DOUBLE, 0.1),
    ])
    received = []
    sub = rclpy.create_node("test_sub_stale_odom")
    sub.create_subscription(SafetyEnvelope, "/autokart_planner/safety_envelope", received.append, 10)
    odom_pub = sub.create_publisher(Odometry, "/autokart/odom", 10)
    path_pub = sub.create_publisher(Path, "/autokart_planner/reference_path", 10)
    executor, thread = _spin(node, [sub])
    try:
        stamp_ns = node.get_clock().now().nanoseconds
        odom = Odometry()
        odom.header.stamp = ros_time(stamp_ns)
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)
        path_pub.publish(_path_msg(stamp_ns, [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]))
        assert _wait_for(lambda: len(received) >= 1 and received[-1].valid)

        time.sleep(0.3)  # let the odom go stale past odom_max_age_s=0.1
        path_pub.publish(_path_msg(node.get_clock().now().nanoseconds,
                                   [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]))
        assert _wait_for(lambda: len(received) >= 2 and not received[-1].valid)
        assert "ODOM" in received[-1].limiting_reason
    finally:
        executor.shutdown()
        node.destroy_node()
        sub.destroy_node()


def test_too_short_path_rejects(ros_context):
    from rclpy.parameter import Parameter
    from nav_msgs.msg import Odometry
    node = PlannerNode(parameter_overrides=[
        Parameter('a_brake_min_mps2', Parameter.Type.DOUBLE, 2.0),
    ])
    received = {"envelope": None}
    sub = rclpy.create_node("test_sub_short_path")
    sub.create_subscription(SafetyEnvelope, "/autokart_planner/safety_envelope",
                            lambda m: received.__setitem__("envelope", m), 10)
    odom_pub = sub.create_publisher(Odometry, "/autokart/odom", 10)
    path_pub = sub.create_publisher(Path, "/autokart_planner/reference_path", 10)
    executor, thread = _spin(node, [sub])
    try:
        stamp_ns = node.get_clock().now().nanoseconds
        odom = Odometry()
        odom.header.stamp = ros_time(stamp_ns)
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)
        time.sleep(0.1)
        path_pub.publish(_path_msg(node.get_clock().now().nanoseconds, [(0.0, 0.0)]))
        assert _wait_for(lambda: received["envelope"] is not None)
        assert received["envelope"].valid is False
        assert "PATH" in received["envelope"].limiting_reason
    finally:
        executor.shutdown()
        node.destroy_node()
        sub.destroy_node()


def test_no_new_path_within_timeout_keeps_reporting_stale_not_just_once(ros_context):
    from rclpy.parameter import Parameter
    from nav_msgs.msg import Odometry
    node = PlannerNode(parameter_overrides=[
        Parameter('a_brake_min_mps2', Parameter.Type.DOUBLE, 2.0),
        Parameter('path_max_age_s', Parameter.Type.DOUBLE, 0.15),
    ])
    received = []
    sub = rclpy.create_node("test_sub_path_timeout")
    sub.create_subscription(SafetyEnvelope, "/autokart_planner/safety_envelope", received.append, 10)
    odom_pub = sub.create_publisher(Odometry, "/autokart/odom", 10)
    path_pub = sub.create_publisher(Path, "/autokart_planner/reference_path", 10)
    executor, thread = _spin(node, [sub])
    try:
        stamp_ns = node.get_clock().now().nanoseconds
        odom = Odometry()
        odom.header.stamp = ros_time(stamp_ns)
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)
        time.sleep(0.1)
        path_pub.publish(_path_msg(node.get_clock().now().nanoseconds,
                                   [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]))
        assert _wait_for(lambda: len(received) >= 1 and received[-1].valid)
        # Stop publishing new paths and wait past path_max_age_s: the node's own on_timer must
        # keep repeating an invalid status, never freeze at the last valid one.
        assert _wait_for(lambda: len(received) >= 2 and not received[-1].valid, timeout_s=3.0)
        count_after_first_invalid = len(received)
        time.sleep(0.5)
        assert len(received) > count_after_first_invalid, \
            "expected repeated PATH_MISSING_OR_STALE reports, not a single one followed by silence"
        assert all(not m.valid for m in received[-3:])
    finally:
        executor.shutdown()
        node.destroy_node()
        sub.destroy_node()


def test_never_receiving_any_path_still_reports_stale_after_startup(ros_context):
    # Regression test: before this review, last_path_wall_s started as None and the staleness
    # timer only began counting once the FIRST path message arrived -- so a planner that never
    # received any reference path at all (e.g. upstream node never started) stayed completely
    # silent on /autokart_planner/safety_envelope forever, which a downstream consumer cannot
    # distinguish from "not started yet." Fixed by starting the staleness clock at construction.
    from rclpy.parameter import Parameter
    node = PlannerNode(parameter_overrides=[
        Parameter('a_brake_min_mps2', Parameter.Type.DOUBLE, 2.0),
        Parameter('path_max_age_s', Parameter.Type.DOUBLE, 0.15),
    ])
    received = []
    sub = rclpy.create_node("test_sub_never_any_path")
    sub.create_subscription(SafetyEnvelope, "/autokart_planner/safety_envelope", received.append, 10)
    executor, thread = _spin(node, [sub])
    try:
        assert _wait_for(lambda: len(received) >= 1, timeout_s=3.0), \
            "expected the node to report PATH_MISSING_OR_STALE on its own, never having " \
            "received any reference path"
        assert all(not m.valid for m in received)
    finally:
        executor.shutdown()
        node.destroy_node()
        sub.destroy_node()


def test_missing_algorithm_dependency_keeps_reporting_fault_not_just_once(ros_context, tmp_path):
    from rclpy.parameter import Parameter
    missing_path = str(tmp_path / "does_not_exist")
    node = PlannerNode(parameter_overrides=[
        Parameter('algorithm_backend', Parameter.Type.STRING, 'reused'),
        Parameter('algorithm_package_path', Parameter.Type.STRING, missing_path),
    ])
    assert node.engine is None
    assert node.startup_error is not None and "DEPENDENCY_PATH_NOT_FOUND" in node.startup_error
    received = []
    sub = rclpy.create_node("test_sub_missing_dep")
    sub.create_subscription(SafetyEnvelope, "/autokart_planner/safety_envelope", received.append, 10)
    executor, thread = _spin(node, [sub])
    try:
        assert _wait_for(lambda: len(received) >= 2, timeout_s=5.0), \
            "expected repeated fault status, not a single message followed by silence"
        assert all(not m.valid for m in received)
    finally:
        executor.shutdown()
        node.destroy_node()
        sub.destroy_node()
