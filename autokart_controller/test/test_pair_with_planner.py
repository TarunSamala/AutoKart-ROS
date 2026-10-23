"""End-to-end: the REAL autokart_planner node (08) feeding this controller node (09).

This is the test that would have caught the cross-topic ordering bug (planner publishes path before
envelope before speed profile). Skipped, not failed, when autokart_planner is not importable -- e.g.
in a standalone controller export bundle, which deliberately does not include node 08.
"""
import sys
import threading
import time
from pathlib import Path as FsPath

import pytest

rclpy = pytest.importorskip("rclpy")

_sibling = FsPath(__file__).resolve().parents[2] / "autokart_planner"
if _sibling.is_dir() and str(_sibling) not in sys.path:
    sys.path.append(str(_sibling))
planner_node = pytest.importorskip("autokart_planner.node")

from nav_msgs.msg import Odometry, Path  # noqa: E402
from autokart_msgs.msg import MotionCommand  # noqa: E402

from autokart_controller.node import ControllerNode  # noqa: E402
from test_node_integration import odom_msg, path_msg  # noqa: E402


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


def test_real_planner_output_drives_a_valid_controller_candidate(ros_context):
    from rclpy.parameter import Parameter
    planner = planner_node.PlannerNode(parameter_overrides=[
        Parameter('a_brake_min_mps2', Parameter.Type.DOUBLE, 2.0),   # test value, not a measurement
        Parameter('t_total_latency_s', Parameter.Type.DOUBLE, 0.1),
        Parameter('speed_limit_mps', Parameter.Type.DOUBLE, 0.8),
    ])
    controller = ControllerNode()
    commands = []
    sub = rclpy.create_node("test_sub_pair")
    sub.create_subscription(MotionCommand, "/autokart_controller/motion_command_candidate", commands.append, 10)
    odom_pub = sub.create_publisher(Odometry, "/autokart/odom", 10)
    ref_pub = sub.create_publisher(Path, "/autokart_planner/reference_path", 10)
    executor = rclpy.executors.MultiThreadedExecutor()
    for n in (planner, controller, sub):
        executor.add_node(n)
    threading.Thread(target=executor.spin, daemon=True).start()
    stop = threading.Event()

    def stream():
        while not stop.is_set():
            now = planner.get_clock().now().nanoseconds
            odom_pub.publish(odom_msg(now, y=-0.3, speed=0.4))
            ref_pub.publish(path_msg(now, [(float(i), 0.0) for i in range(11)]))
            stop.wait(0.05)
    threading.Thread(target=stream, daemon=True).start()
    try:
        assert _wait_for(lambda: any(c.valid for c in commands), timeout_s=5.0), \
            "controller never produced a valid candidate from the real planner's output"
        valid = [c for c in commands if c.valid]
        assert all(0.0 < c.target_speed_mps <= 0.8 + 1e-9 for c in valid)
        assert all(c.steering_angle_rad > 0.0 for c in valid[-3:])  # path is left of the kart
    finally:
        stop.set()
        executor.shutdown()
        for n in (planner, controller, sub):
            n.destroy_node()
