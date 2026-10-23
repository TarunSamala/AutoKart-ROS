"""Integration tests: real rclpy node, real pub/sub and real TF, with the
reused autokart_localization WheelOdometry (skipped if that checkout is absent).
"""
import math
import os
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.parameter import Parameter  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from autokart_msgs.msg import ActuatorFeedback, SensorDeliveryMetadata  # noqa: E402
from builtin_interfaces.msg import Time as RosTime  # noqa: E402

from autokart_odometry.node import OdometryNode  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[3]
REAL_LOCALIZATION_PATH = Path(os.environ.get(
    "AUTOKART_LOCALIZATION_PATH", str(REPO_ROOT / "kart" / "ros2_ws" / "src" / "autokart_localization")))
pytestmark = pytest.mark.skipif(not REAL_LOCALIZATION_PATH.is_dir(),
                                reason="real autokart_localization checkout not present")


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _wait_for(pred, timeout_s=5.0, step_s=0.02):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(step_s)
    return False


def _params(**extra):
    base = {"algorithm_package_path": str(REAL_LOCALIZATION_PATH), "wheelbase_m": 1.0}
    base.update(extra)
    out = []
    for k, v in base.items():
        t = {str: Parameter.Type.STRING, float: Parameter.Type.DOUBLE, int: Parameter.Type.INTEGER,
             bool: Parameter.Type.BOOL}[type(v)]
        out.append(Parameter(k, t, v))
    return out


def _spin(nodes):
    ex = rclpy.executors.MultiThreadedExecutor()
    for n in nodes:
        ex.add_node(n)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    ex._test_spin_thread = th
    return ex


def test_synthetic_counts_mode_publishes_odom_status_and_tf(ros_context):
    import tf2_ros
    node = OdometryNode(parameter_overrides=_params(counts_per_m=100.0, synthetic_steering_rad=0.1))
    sub = rclpy.create_node("test_odom_sub")
    got = {"odom": [], "status": None}
    sub.create_subscription(Odometry, "/odom", lambda m: got["odom"].append(m), 50)
    sub.create_subscription(SensorDeliveryMetadata, "/odom/status", lambda m: got.__setitem__("status", m), 10)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, sub)
    ex = _spin([node, sub])
    try:
        assert _wait_for(lambda: len(got["odom"]) >= 20)
        o = got["odom"][-1]
        assert o.header.frame_id == "odom" and o.child_frame_id == "base_link"
        assert o.pose.pose.position.x > 0.0
        assert o.pose.pose.orientation.z > 0.0          # positive steering -> positive yaw (bicycle model)
        assert o.pose.covariance[0] > 0 and o.pose.covariance[35] > 0
        assert all(math.isfinite(c) for c in o.pose.covariance) and all(math.isfinite(c) for c in o.twist.covariance)
        assert _wait_for(lambda: got["status"] is not None and got["status"].quality == "OK", timeout_s=2.0)
        assert got["status"].fault_state == "NONE"
        assert got["status"].calibration_version == "UNVERIFIED"

        def _tf_ok():
            try:
                t = buf.lookup_transform("odom", "base_link", rclpy.time.Time())
                return t.transform.translation.x > 0.0
            except Exception:
                return False
        assert _wait_for(_tf_ok, timeout_s=3.0), "odom->base_link TF not observed"
    finally:
        ex.shutdown()
        ex._test_spin_thread.join(timeout=2.0)
        node.destroy_node()
        sub.destroy_node()


def test_actuator_feedback_source_integrates_and_refuses_invalid_feedback(ros_context):
    node = OdometryNode(parameter_overrides=_params(source="actuator_feedback"))
    sub = rclpy.create_node("test_odom_fb")
    got = {"odom": [], "status": []}
    sub.create_subscription(Odometry, "/odom", lambda m: got["odom"].append(m), 50)
    sub.create_subscription(SensorDeliveryMetadata, "/odom/status", lambda m: got["status"].append(m), 50)
    pub = sub.create_publisher(ActuatorFeedback, "/vehicle/actuator_feedback", 10)
    ex = _spin([node, sub])
    state = {"valid": True, "stop": False}

    def feeder():
        t0 = time.monotonic()
        while not state["stop"]:
            m = ActuatorFeedback()
            ns = int((time.monotonic() - t0) * 1e9)
            m.measurement_time = RosTime(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)
            m.motor_speed_radps = 1.0   # m/s (documented field reuse by the VCU bridge)
            m.road_wheel_angle_rad = 0.0
            m.valid = state["valid"]
            pub.publish(m)
            time.sleep(0.02)

    th = threading.Thread(target=feeder, daemon=True)
    th.start()
    try:
        assert _wait_for(lambda: len(got["odom"]) >= 10)
        assert got["odom"][-1].pose.pose.position.x > 0.0
        state["valid"] = False
        assert _wait_for(lambda: got["status"] and got["status"][-1].fault_state == "INPUT_INVALID")
        n = len(got["odom"])
        time.sleep(0.3)
        assert len(got["odom"]) <= n + 1, "no odometry may be published while the feedback is invalid"
    finally:
        state["stop"] = True
        th.join(timeout=1.0)
        ex.shutdown()
        ex._test_spin_thread.join(timeout=2.0)
        node.destroy_node()
        sub.destroy_node()
