"""Fault-injection / adversarial-behavior tests for autokart_safety.

These exercise the actual ROS node (real rclpy, real autokart_msgs types,
real imported autokart_vcu_safety core) end to end through its public
topics, not just the pure-logic unit tests in the reused core's own test
suite. Every case here is something a NOOP/benign node could get wrong by
silently skipping a check.
"""
import json
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
from std_msgs.msg import String  # noqa: E402
from autokart_msgs.msg import ActuatorFeedback, MissionDebug, VcuState  # noqa: E402

from autokart_safety.node import AutokartSafetyNode  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[3]
REAL_VCU_SAFETY_SRC = REPO_ROOT / "kart" / "ros2_ws" / "src" / "autokart_vcu_safety"
pytestmark = pytest.mark.skipif(
    not REAL_VCU_SAFETY_SRC.is_dir(), reason="real kart/ros2_ws/src checkout not present")


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


def _vcu(state: str, estop_active: bool = False, estop_input_fitted: bool = True):
    msg = VcuState()
    msg.state = state
    msg.motor_enable = state == "ARMED"
    msg.contactor_open = state != "ARMED"
    msg.emergency_stop_active = estop_active
    msg.estop_input_fitted = estop_input_fitted
    msg.active_speed_limit_mps = 2.5
    msg.limiting_reason = ""
    msg.active_faults = []
    msg.reset_epoch = 0
    return msg


def _feedback(speed_mps: float = 0.0, valid: bool = True):
    msg = ActuatorFeedback()
    msg.motor_speed_radps = speed_mps
    msg.valid = valid
    msg.temperatures_c = []
    return msg


def _spin(node):
    received = []
    sub_node = rclpy.create_node(f"test_sub_{id(node)}")
    sub_node.create_subscription(MissionDebug, "/autokart/safety/status", lambda m: received.append(m), 10)
    request_pub = sub_node.create_publisher(String, "/autokart/safety/request", 10)
    requested = []
    sub_node.create_subscription(String, "/autokart/vcu/request", lambda m: requested.append(m.data), 10)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return received, requested, request_pub, executor, sub_node, thread


def _make_node():
    from rclpy.parameter import Parameter
    return AutokartSafetyNode(parameter_overrides=[
        Parameter('vcu_safety_package_path', Parameter.Type.STRING, str(REAL_VCU_SAFETY_SRC)),
    ])


class _LiveTraffic:
    """Keeps re-touching every freshness key on a background thread, the way
    real subscription callbacks would at high rate. vcu_timeout_s/odom_timeout_s
    default to 0.3s in the real core's Config -- a single one-shot touch goes
    stale well within the time an end-to-end rclpy test needs to publish a
    request and observe the result, so tests that must stay "ready" across
    more than ~0.3s of wall-clock time use this instead of a single touch."""

    def __init__(self, node, keys=("vcu", "odom", "perception", "mapping", "planner", "controller"),
                 period_s: float = 0.02):
        self.node = node
        self.keys = keys
        self.period_s = period_s
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        while not self._stop.is_set():
            for key in self.keys:
                self.node._touch(key)
            self._stop.wait(self.period_s)

    def start(self):
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)


def test_ready_but_unverified_arm_attempt_is_rejected(ros_context):
    """Every freshness input is fully fresh/valid, but calibration_verified,
    vehicle_config_verified and braking_verified all default false. An arm
    request must be refused, not silently granted."""
    node = _make_node()
    assert node.logic is not None, node.load_error
    received, requested, request_pub, executor, sub_node, thread = _spin(node)
    try:
        node._on_vcu(_vcu("SAFE", estop_active=False))
        node._on_feedback(_feedback(0.0))
        for key in ("perception", "mapping", "planner", "controller", "odom"):
            node._touch(key)
        node.planner_decision = "CRUISE"
        request_pub.publish(String(data="confirm_manual_power_cut"))
        request_pub.publish(String(data="arm"))
        time.sleep(0.5)
        assert _wait_for(lambda: len(received) >= 2, timeout_s=3.0)
        for message in received:
            payload = json.loads(message.payload_json)
            assert payload["state"] == "DISARMED", payload
            assert payload["arm_allowed"] is False
        assert "ARM" not in requested
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()


def test_arm_succeeds_once_fully_ready_and_verified(ros_context):
    """Positive control for the above: with the three *_verified parameters
    explicitly overridden true (simulating a person who completed the bench
    checklist) and every input fresh, arming must actually proceed and forward
    ARM to the (not-present-in-this-test) vehicle bridge. vehicle_authorized
    stays false in the published payload regardless -- this node never claims
    hardware authorization, only its own software state."""
    from rclpy.parameter import Parameter
    node = AutokartSafetyNode(parameter_overrides=[
        Parameter('vcu_safety_package_path', Parameter.Type.STRING, str(REAL_VCU_SAFETY_SRC)),
        Parameter('calibration_verified', Parameter.Type.BOOL, True),
        Parameter('vehicle_config_verified', Parameter.Type.BOOL, True),
        Parameter('braking_verified', Parameter.Type.BOOL, True),
    ])
    assert node.logic is not None, node.load_error
    received, requested, request_pub, executor, sub_node, thread = _spin(node)
    traffic = _LiveTraffic(node).start()
    try:
        node._on_vcu(_vcu("SAFE", estop_active=False))
        node._on_feedback(_feedback(0.0))
        node.planner_decision = "CRUISE"
        request_pub.publish(String(data="confirm_manual_power_cut"))
        time.sleep(0.3)
        request_pub.publish(String(data="arm"))
        assert _wait_for(lambda: "ARM" in requested, timeout_s=3.0), requested
        assert _wait_for(lambda: any(
            json.loads(m.payload_json)["state"] == "ARMING" for m in received), timeout_s=3.0)
        last_payload = json.loads(received[-1].payload_json)
        assert last_payload["vehicle_authorized"] is False
    finally:
        traffic.stop()
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()


def test_estop_active_forces_disarm_while_armed(ros_context):
    """An E-stop that reports active while the VCU is ARMED must force a
    runtime FAULT/disarm -- this node must never keep reporting ARMED once a
    real fault input appears, even though it has no path to the physical
    circuit itself."""
    from rclpy.parameter import Parameter
    node = AutokartSafetyNode(parameter_overrides=[
        Parameter('vcu_safety_package_path', Parameter.Type.STRING, str(REAL_VCU_SAFETY_SRC)),
        Parameter('calibration_verified', Parameter.Type.BOOL, True),
        Parameter('vehicle_config_verified', Parameter.Type.BOOL, True),
        Parameter('braking_verified', Parameter.Type.BOOL, True),
    ])
    received, requested, request_pub, executor, sub_node, thread = _spin(node)
    traffic = _LiveTraffic(node).start()
    try:
        node._on_vcu(_vcu("SAFE", estop_active=False))
        node._on_feedback(_feedback(0.0))
        node.planner_decision = "CRUISE"
        request_pub.publish(String(data="confirm_manual_power_cut"))
        time.sleep(0.3)
        request_pub.publish(String(data="arm"))
        assert _wait_for(lambda: "ARM" in requested, timeout_s=3.0)
        node._on_vcu(_vcu("ARMED", estop_active=False))
        assert _wait_for(lambda: any(
            json.loads(m.payload_json)["state"] == "ARMED" for m in received), timeout_s=3.0)
        # Now trip the E-stop input while armed.
        node._on_vcu(_vcu("ARMED", estop_active=True, estop_input_fitted=True))
        assert _wait_for(lambda: any(
            json.loads(m.payload_json)["state"] == "FAULT" for m in received[-5:]), timeout_s=3.0)
        assert "DIS" in requested
    finally:
        traffic.stop()
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()


def test_stale_vcu_heartbeat_blocks_arm(ros_context):
    """No /vcu/state has ever arrived (age=None, the fail-safe default) -- an
    arm attempt must be refused even if every other input looks fine."""
    node = _make_node()
    received, requested, request_pub, executor, sub_node, thread = _spin(node)
    try:
        node._on_feedback(_feedback(0.0))
        for key in ("perception", "mapping", "planner", "controller", "odom"):
            node._touch(key)
        node.planner_decision = "CRUISE"
        request_pub.publish(String(data="arm"))
        time.sleep(0.5)
        assert _wait_for(lambda: len(received) >= 1, timeout_s=3.0)
        for message in received:
            payload = json.loads(message.payload_json)
            assert payload["state"] == "DISARMED"
            assert payload["arm_allowed"] is False
        assert "ARM" not in requested
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()
