"""Fault-injection tests: missing/stale odom, invalid planner envelope, mismatched stamps, expired
path, never-received path and a missing pursuit dependency must all produce a STOP candidate
(valid=false, zero speed, zero steering) -- never a stale or reused command.
"""
import json
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from nav_msgs.msg import Odometry, Path  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from autokart_msgs.msg import MotionCommand, SafetyEnvelope  # noqa: E402

from autokart_controller.node import ControllerNode  # noqa: E402
from test_node_integration import envelope_msg, odom_msg, path_msg  # noqa: E402

POINTS = [(float(i), 0.0) for i in range(11)]


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


class Harness:
    def __init__(self, node):
        self.node = node
        self.commands = []
        self.diags = []
        self.sub = rclpy.create_node(f"test_sub_{id(self)}")
        self.sub.create_subscription(MotionCommand, "/autokart_controller/motion_command_candidate",
                                     self.commands.append, 10)
        self.sub.create_subscription(String, "/autokart_controller/diagnostics",
                                     lambda m: self.diags.append(json.loads(m.data)), 10)
        self.odom = self.sub.create_publisher(Odometry, "/autokart/odom", 10)
        self.env = self.sub.create_publisher(SafetyEnvelope, "/autokart_planner/safety_envelope", 10)
        self.prof = self.sub.create_publisher(String, "/autokart_planner/speed_profile", 10)
        self.path = self.sub.create_publisher(Path, "/autokart_planner/path", 10)
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(node)
        self.executor.add_node(self.sub)
        threading.Thread(target=self.executor.spin, daemon=True).start()
        self._odom_stop = threading.Event()
        self._odom_thread = None
        time.sleep(0.3)

    def now(self):
        return self.node.get_clock().now().nanoseconds

    def start_odom(self, y=0.0, speed=0.5, rate_hz=50.0):
        """Stream odometry continuously, as a real odometry node does. A single odom message sent
        once beforehand is (correctly) stale by the time a stamp-paired path is processed."""
        self.stop_odom()
        self._odom_stop.clear()

        def run():
            while not self._odom_stop.is_set():
                self.odom.publish(odom_msg(self.now(), y=y, speed=speed))
                self._odom_stop.wait(1.0 / rate_hz)
        self._odom_thread = threading.Thread(target=run, daemon=True)
        self._odom_thread.start()
        time.sleep(0.05)

    def stop_odom(self):
        self._odom_stop.set()
        if self._odom_thread is not None:
            self._odom_thread.join(timeout=1.0)
            self._odom_thread = None

    def close(self):
        self.stop_odom()
        self.executor.shutdown()
        self.node.destroy_node()
        self.sub.destroy_node()


def _assert_stop(cmd):
    assert cmd.valid is False
    assert cmd.target_speed_mps == 0.0
    assert cmd.steering_angle_rad == 0.0


def test_missing_odom_produces_stop_candidate(ros_context):
    h = Harness(ControllerNode())
    try:
        stamp = h.now()
        h.env.publish(envelope_msg(stamp))
        time.sleep(0.1)
        h.path.publish(path_msg(stamp, POINTS))
        assert _wait_for(lambda: any("POSE_MISSING_OR_STALE" in d["reason_codes"] for d in h.diags))
        _assert_stop(h.commands[-1])
    finally:
        h.close()


def test_stale_odom_after_a_valid_cycle_produces_stop_not_the_old_command(ros_context):
    from rclpy.parameter import Parameter
    h = Harness(ControllerNode(parameter_overrides=[
        Parameter('pose_max_age_s', Parameter.Type.DOUBLE, 0.1)]))
    try:
        h.start_odom(y=-0.5)
        stamp = h.now()
        h.env.publish(envelope_msg(stamp))
        time.sleep(0.05)
        h.path.publish(path_msg(stamp, POINTS))
        assert _wait_for(lambda: h.commands and h.commands[-1].valid)
        h.stop_odom()
        time.sleep(0.3)  # odom goes stale
        stamp2 = h.now()
        h.env.publish(envelope_msg(stamp2))
        time.sleep(0.05)
        h.path.publish(path_msg(stamp2, POINTS))
        assert _wait_for(lambda: any("POSE_MISSING_OR_STALE" in d["reason_codes"] for d in h.diags))
        _assert_stop(h.commands[-1])
    finally:
        h.close()


def test_invalid_planner_envelope_produces_stop(ros_context):
    h = Harness(ControllerNode())
    try:
        stamp = h.now()
        h.odom.publish(odom_msg(stamp))
        h.env.publish(envelope_msg(stamp, valid=False))
        time.sleep(0.1)
        h.path.publish(path_msg(stamp, POINTS))
        assert _wait_for(lambda: any("PLANNER_INVALID" in d["reason_codes"] for d in h.diags))
        _assert_stop(h.commands[-1])
    finally:
        h.close()


def test_envelope_for_a_different_stamp_is_not_trusted(ros_context):
    h = Harness(ControllerNode())
    try:
        h.start_odom()
        stamp = h.now()
        h.env.publish(envelope_msg(stamp - 50_000_000, valid=True))  # valid, but for an older path
        time.sleep(0.1)
        h.path.publish(path_msg(stamp, POINTS))
        # No envelope exists for THIS path's stamp: after pair_timeout_s it is rejected, never
        # paired with the other cycle's (valid) envelope.
        assert _wait_for(lambda: any("PLANNER_STATUS_MISSING" in d["reason_codes"] for d in h.diags))
        assert not any(c.valid for c in h.commands)
    finally:
        h.close()


def test_expired_path_produces_stop(ros_context):
    h = Harness(ControllerNode())
    try:
        h.start_odom()
        stamp = h.now() - 2_000_000_000  # path stamped 2 s ago, path_max_age_s=0.5
        h.env.publish(envelope_msg(stamp))
        time.sleep(0.1)
        h.path.publish(path_msg(stamp, POINTS))
        assert _wait_for(lambda: any("PATH_EXPIRED" in d["reason_codes"] for d in h.diags))
        _assert_stop(h.commands[-1])
    finally:
        h.close()


def test_no_new_path_keeps_reporting_stop_not_just_once(ros_context):
    from rclpy.parameter import Parameter
    h = Harness(ControllerNode(parameter_overrides=[
        Parameter('path_max_age_s', Parameter.Type.DOUBLE, 0.15)]))
    try:
        h.start_odom()
        stamp = h.now()
        h.env.publish(envelope_msg(stamp))
        h.prof.publish(String(data=json.dumps({"sample_time_ns": stamp, "speed_profile_mps": [1.0] * 11})))
        time.sleep(0.05)
        h.path.publish(path_msg(stamp, POINTS))
        assert _wait_for(lambda: h.commands and h.commands[-1].valid)
        assert _wait_for(lambda: not h.commands[-1].valid, timeout_s=3.0)
        n = len(h.commands)
        time.sleep(0.5)
        assert len(h.commands) > n, "expected repeated stop candidates, not one then silence"
        for cmd in h.commands[-3:]:
            _assert_stop(cmd)
    finally:
        h.close()


def test_never_receiving_any_path_still_reports_stop(ros_context):
    from rclpy.parameter import Parameter
    h = Harness(ControllerNode(parameter_overrides=[
        Parameter('path_max_age_s', Parameter.Type.DOUBLE, 0.15)]))
    try:
        assert _wait_for(lambda: len(h.commands) >= 2, timeout_s=3.0)
        for cmd in h.commands:
            _assert_stop(cmd)
    finally:
        h.close()


def test_malformed_speed_profile_does_not_crash_and_falls_back_to_envelope_cap(ros_context):
    h = Harness(ControllerNode())
    try:
        h.start_odom()
        stamp = h.now()
        h.env.publish(envelope_msg(stamp, max_speed=0.4))
        h.prof.publish(String(data="{not json"))
        time.sleep(0.1)
        h.path.publish(path_msg(stamp, POINTS))
        assert _wait_for(lambda: h.commands and h.commands[-1].valid)
        assert h.commands[-1].target_speed_mps <= 0.4 + 1e-9
    finally:
        h.close()


def test_missing_pursuit_dependency_keeps_reporting_fault(ros_context, tmp_path):
    from rclpy.parameter import Parameter
    node = ControllerNode(parameter_overrides=[
        Parameter('pursuit_backend', Parameter.Type.STRING, 'reused'),
        Parameter('kart_sim_package_path', Parameter.Type.STRING, str(tmp_path / "missing")),
    ])
    assert node.engine is None and "DEPENDENCY_PATH_NOT_FOUND" in node.startup_error
    h = Harness(node)
    try:
        assert _wait_for(lambda: len(h.commands) >= 2, timeout_s=3.0)
        for cmd in h.commands:
            _assert_stop(cmd)
    finally:
        h.close()


def test_unqualified_wheelbase_fails_closed_at_startup(ros_context):
    from rclpy.parameter import Parameter
    node = ControllerNode(parameter_overrides=[
        Parameter('wheelbase_m', Parameter.Type.DOUBLE, 0.0)])
    assert node.engine is None and "WHEELBASE_UNQUALIFIED" in node.startup_error
    h = Harness(node)
    try:
        assert _wait_for(lambda: len(h.commands) >= 1, timeout_s=3.0)
        _assert_stop(h.commands[-1])
    finally:
        h.close()


def test_path_arriving_before_its_envelope_is_still_processed(ros_context):
    # Regression test (review finding): autokart_planner publishes path, THEN envelope, THEN speed
    # profile for the same stamp, and ROS gives no cross-topic ordering guarantee anyway. The first
    # draft processed on path arrival only and required an already-received envelope with the same
    # stamp -- so wired to the real planner it would have emitted STOP (PLANNER_INVALID) forever.
    h = Harness(ControllerNode())
    try:
        h.start_odom(y=-0.5)
        stamp = h.now()
        h.path.publish(path_msg(stamp, POINTS))          # planner's actual order: path first
        time.sleep(0.02)
        h.env.publish(envelope_msg(stamp, valid=True, max_speed=1.0))
        h.prof.publish(String(data=json.dumps({"sample_time_ns": stamp, "speed_profile_mps": [0.6] * 11})))
        assert _wait_for(lambda: h.commands and h.commands[-1].valid, timeout_s=3.0), \
            [d["reason_codes"] for d in h.diags]
        assert h.commands[-1].target_speed_mps <= 0.6 + 1e-9  # per-point profile actually used
    finally:
        h.close()


def test_path_whose_envelope_never_arrives_is_rejected_not_dropped(ros_context):
    h = Harness(ControllerNode())
    try:
        stamp = h.now()
        h.odom.publish(odom_msg(stamp))
        h.path.publish(path_msg(stamp, POINTS))
        assert _wait_for(lambda: any("PLANNER_STATUS_MISSING" in d["reason_codes"] for d in h.diags),
                         timeout_s=3.0)
        _assert_stop(h.commands[-1])
    finally:
        h.close()
