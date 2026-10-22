"""Integration test: real rclpy node, real pub/sub, synthetic backend, with
the real core.py control law reused from kart/ros2_ws's autokart_vehicle_bridge
package (skipped automatically if that checkout is not present).

Requires a sourced ROS 2 Humble environment (rclpy, std_msgs, autokart_msgs).
Run via: colcon test --packages-select autokart_vcu_bridge
"""
import os
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
from std_msgs.msg import Bool, String  # noqa: E402
from builtin_interfaces.msg import Time as RosTime  # noqa: E402
from autokart_msgs.msg import ActuatorFeedback, MotionCommand, SensorDeliveryMetadata, VcuState  # noqa: E402

from autokart_vcu_bridge.node import VcuBridgeNode  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[3]
# AUTOKART_VEHICLE_BRIDGE_PATH overrides this (e.g. an export bundle's third_party/ copy).
REAL_VEHICLE_BRIDGE_PATH = Path(os.environ.get(
    "AUTOKART_VEHICLE_BRIDGE_PATH",
    str(REPO_ROOT / "kart" / "ros2_ws" / "src" / "autokart_vehicle_bridge")))


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _wait_for(predicate, timeout_s=5.0, step_s=0.02):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(step_s)
    return False


def _zero_stamp():
    return RosTime(sec=0, nanosec=0)


def _republish_candidate_loop(stop_event, candidate_pub, target_speed_mps, period_s=0.05):
    """A single one-shot MotionCommand goes stale within cmd_timeout_s (0.25s
    by default): a real upstream controller republishes continuously, so the
    test does too, rather than racing the gate's own freshness window."""
    while not stop_event.is_set():
        cmd = MotionCommand()
        cmd.timestamp = _zero_stamp()
        cmd.expiry = _zero_stamp()  # 0 = no expiry, per this node's own expiry-check convention
        cmd.target_speed_mps = target_speed_mps
        cmd.valid = True
        candidate_pub.publish(cmd)
        time.sleep(period_s)


@pytest.mark.skipif(not REAL_VEHICLE_BRIDGE_PATH.is_dir(),
                     reason="real autokart_vehicle_bridge checkout not present in this environment")
def test_vcu_bridge_relays_authorized_command_to_nonzero_throttle(ros_context):
    import rclpy as _rclpy
    from rclpy.parameter import Parameter

    node = VcuBridgeNode(parameter_overrides=[
        Parameter('algorithm_package_path', Parameter.Type.STRING, str(REAL_VEHICLE_BRIDGE_PATH)),
        Parameter('tick_hz', Parameter.Type.DOUBLE, 50.0),
    ])
    assert node._core is not None, "expected the real core.py to import successfully"

    received = {"feedback": None, "state": None, "status": None}
    sub_node = _rclpy.create_node("test_subscriber_vcu")
    sub_node.create_subscription(
        ActuatorFeedback, "/vehicle/actuator_feedback", lambda m: received.__setitem__("feedback", m), 10)
    sub_node.create_subscription(VcuState, "/vcu/state", lambda m: received.__setitem__("state", m), 10)
    sub_node.create_subscription(
        SensorDeliveryMetadata, "/vcu_bridge/status", lambda m: received.__setitem__("status", m), 10)
    armed_pub = sub_node.create_publisher(Bool, "/vcu_bridge/safety_armed", 10)
    request_pub = sub_node.create_publisher(String, "/vcu_bridge/request", 10)
    candidate_pub = sub_node.create_publisher(MotionCommand, "/autokart/controller/motion_command_candidate", 10)

    executor = _rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    stop_event = threading.Event()
    publish_thread = threading.Thread(
        target=_republish_candidate_loop, args=(stop_event, candidate_pub, 1.0), daemon=True)

    def _keep_safety_armed_fresh():
        while not stop_event.is_set():
            armed_pub.publish(Bool(data=True))
            time.sleep(0.05)

    safety_thread = threading.Thread(target=_keep_safety_armed_fresh, daemon=True)
    try:
        assert _wait_for(lambda: received["status"] is not None)
        assert received["status"].quality == "OK"  # core imported, no backend fault

        safety_thread.start()
        request_pub.publish(String(data="ARM"))
        assert _wait_for(lambda: received["state"] is not None and received["state"].state == "ARMED",
                          timeout_s=3.0), "expected the synthetic VCU to report ARMED after ARM+safety_armed"

        publish_thread.start()

        def _throttle_actually_flowed():
            fb = received["feedback"]
            return fb is not None and fb.valid and fb.motor_speed_radps > 0.0

        assert _wait_for(_throttle_actually_flowed, timeout_s=3.0), \
            "expected measured speed feedback to go nonzero once ARMED + safety_armed + a fresh valid candidate"
        assert received["state"].active_faults == []
    finally:
        stop_event.set()
        publish_thread.join(timeout=1.0)
        safety_thread.join(timeout=1.0)
        executor.shutdown()
        thread.join(timeout=2.0)  # let any in-flight callback finish before destroy
        node.destroy_node()
        sub_node.destroy_node()


@pytest.mark.skipif(not REAL_VEHICLE_BRIDGE_PATH.is_dir(),
                     reason="real autokart_vehicle_bridge checkout not present in this environment")
def test_vcu_bridge_without_safety_armed_never_sends_nonzero_throttle(ros_context):
    """Even with a valid, continuously-fresh candidate and a successful ARM at
    the firmware level, no safety_armed input (the realistic state of this
    library today -- node 10 does not exist) must mean the reused CommandGate
    never passes nonzero throttle through to the backend, so measured speed
    feedback stays at zero despite the VCU reporting ARMED."""
    import rclpy as _rclpy
    from rclpy.parameter import Parameter

    node = VcuBridgeNode(parameter_overrides=[
        Parameter('algorithm_package_path', Parameter.Type.STRING, str(REAL_VEHICLE_BRIDGE_PATH)),
        Parameter('tick_hz', Parameter.Type.DOUBLE, 50.0),
    ])
    received = {"state": None, "feedback": None}
    sub_node = _rclpy.create_node("test_subscriber_vcu_no_safety")
    sub_node.create_subscription(VcuState, "/vcu/state", lambda m: received.__setitem__("state", m), 10)
    sub_node.create_subscription(
        ActuatorFeedback, "/vehicle/actuator_feedback", lambda m: received.__setitem__("feedback", m), 10)
    request_pub = sub_node.create_publisher(String, "/vcu_bridge/request", 10)
    candidate_pub = sub_node.create_publisher(MotionCommand, "/autokart/controller/motion_command_candidate", 10)

    executor = _rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    stop_event = threading.Event()
    publish_thread = threading.Thread(
        target=_republish_candidate_loop, args=(stop_event, candidate_pub, 5.0), daemon=True)
    try:
        request_pub.publish(String(data="ARM"))
        publish_thread.start()
        assert _wait_for(lambda: received["state"] is not None and received["state"].state == "ARMED",
                          timeout_s=3.0), "ARM is relayed directly (no upstream safety gate on ARM itself)"
        # Give the (intentionally never-armed-by-safety) gate a full second of
        # continuous, fresh, valid 5 m/s candidates to prove it stays shut.
        time.sleep(1.0)
        assert received["feedback"] is not None
        assert not (received["feedback"].valid and received["feedback"].motor_speed_radps > 0.0), \
            "throttle must never flow without a safety_armed=True input, even though the VCU itself is ARMED"
    finally:
        stop_event.set()
        publish_thread.join(timeout=1.0)
        executor.shutdown()
        thread.join(timeout=2.0)  # let any in-flight callback finish before destroy
        node.destroy_node()
        sub_node.destroy_node()
