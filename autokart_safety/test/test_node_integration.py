"""Integration test: real rclpy node, real pub/sub, real autokart_vcu_safety
core loaded from the checked-out repository path.

Requires a sourced ROS 2 Humble environment (rclpy, std_msgs, autokart_msgs)
and the real kart/ros2_ws/src/autokart_vcu_safety checkout to be present.
"""
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


def _make_vcu_state(state: str, estop_active: bool, estop_input_fitted: bool = True):
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


def _make_feedback(speed_mps: float, valid: bool = True):
    msg = ActuatorFeedback()
    msg.motor_speed_radps = speed_mps
    msg.valid = valid
    msg.temperatures_c = []
    return msg


def _make_status(decision: str = "CRUISE"):
    import json
    msg = MissionDebug()
    msg.payload_json = json.dumps({"source": "LIVE", "valid": True, "decision": decision})
    return msg


def test_safety_node_fails_closed_without_core_dependency(ros_context):
    """backend-equivalent: no vcu_safety_package_path set -> permanent UNAVAILABLE,
    never ARMED, status kept arriving (not one-shot)."""
    from rclpy.parameter import Parameter

    node = AutokartSafetyNode(parameter_overrides=[
        Parameter('vcu_safety_package_path', Parameter.Type.STRING, ''),
    ])
    assert node.logic is None and node.load_error is not None

    received = []
    sub_node = rclpy.create_node("test_subscriber_unavailable")
    sub_node.create_subscription(MissionDebug, "/autokart/safety/status", lambda m: received.append(m), 10)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert _wait_for(lambda: len(received) >= 2, timeout_s=3.0)
        import json
        for message in received:
            payload = json.loads(message.payload_json)
            assert payload["state"] == "UNAVAILABLE"
            assert payload["arm_allowed"] is False
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()


def test_safety_node_never_auto_arms_on_startup_with_real_core(ros_context):
    """With the real core loaded and every upstream input already fresh/valid
    (but no operator request sent at all), the node must stay DISARMED."""
    from rclpy.parameter import Parameter

    node = AutokartSafetyNode(parameter_overrides=[
        Parameter('vcu_safety_package_path', Parameter.Type.STRING, str(REAL_VCU_SAFETY_SRC)),
    ])
    assert node.logic is not None, node.load_error

    received = []
    sub_node = rclpy.create_node("test_subscriber_no_auto_arm")
    sub_node.create_subscription(MissionDebug, "/autokart/safety/status", lambda m: received.append(m), 10)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        node._on_vcu(_make_vcu_state("SAFE", estop_active=False))
        node._on_feedback(_make_feedback(0.0))
        for key, topic in (("perception", None), ("mapping", None), ("planner", None), ("controller", None), ("odom", None)):
            pass
        node.planner_decision = "CRUISE"
        for key in ("perception", "mapping", "planner", "controller", "odom"):
            node._touch(key)
        assert _wait_for(lambda: len(received) >= 3, timeout_s=3.0)
        import json
        for message in received[-3:]:
            payload = json.loads(message.payload_json)
            assert payload["state"] == "DISARMED"
            # calibration/vehicle_config/braking_verified default false -> never ready to arm
            assert payload["arm_allowed"] is False
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()
