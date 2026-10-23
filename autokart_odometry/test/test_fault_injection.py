"""Fault injection: bad config, missing algorithm, bad/missing inputs. The
node must report the fault, never crash, never publish odometry/TF from bad
data, and never go silent.
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
from autokart_msgs.msg import SensorDeliveryMetadata  # noqa: E402

from autokart_odometry.node import OdometryNode  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[3]
REAL_LOCALIZATION_PATH = Path(os.environ.get(
    "AUTOKART_LOCALIZATION_PATH", str(REPO_ROOT / "kart" / "ros2_ws" / "src" / "autokart_localization")))
needs_real = pytest.mark.skipif(not REAL_LOCALIZATION_PATH.is_dir(),
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


def _p(**kv):
    types = {str: Parameter.Type.STRING, float: Parameter.Type.DOUBLE, int: Parameter.Type.INTEGER,
             bool: Parameter.Type.BOOL}
    return [Parameter(k, types[type(v)], v) for k, v in kv.items()]


def _run(params, check):
    node = OdometryNode(parameter_overrides=params)
    sub = rclpy.create_node("test_odom_fault_sub")
    got = {"odom": [], "status": []}
    sub.create_subscription(Odometry, "/odom", lambda m: got["odom"].append(m), 100)
    sub.create_subscription(SensorDeliveryMetadata, "/odom/status", lambda m: got["status"].append(m), 100)
    ex = rclpy.executors.MultiThreadedExecutor()
    ex.add_node(node)
    ex.add_node(sub)
    spin_thread = threading.Thread(target=ex.spin, daemon=True)
    spin_thread.start()
    try:
        check(node, got)
    finally:
        ex.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        sub.destroy_node()


def _yaw(o):
    return 2.0 * math.atan2(o.pose.pose.orientation.z, o.pose.pose.orientation.w)


def test_missing_algorithm_fails_closed_and_keeps_reporting(ros_context):
    def check(node, got):
        assert node.odom is None and "ALGORITHM_PATH_NOT_CONFIGURED" in node.startup_error
        assert _wait_for(lambda: len(got["status"]) >= 2), "fault must repeat, not report once then go silent"
        assert all(s.quality == "INVALID" for s in got["status"])
        time.sleep(0.3)
        assert got["odom"] == [], "no odometry may ever be published without the reused algorithm"
    _run(_p(wheelbase_m=1.0), check)


@needs_real
def test_unconfigured_wheelbase_fails_closed_via_reused_validation(ros_context):
    def check(node, got):
        assert node.odom is None and node.startup_error.startswith("INVALID_CONFIG:")
        assert _wait_for(lambda: len(got["status"]) >= 2)
        assert all("INVALID_CONFIG" in s.fault_state for s in got["status"])
        assert got["odom"] == []
    # wheelbase_m left at its 0.0 default: no guessed physical constant is ever substituted.
    _run(_p(algorithm_package_path=str(REAL_LOCALIZATION_PATH)), check)


@needs_real
@pytest.mark.parametrize("bad", [0.0, -1.0, float("nan")])
def test_nonpositive_or_nonfinite_covariance_is_refused(ros_context, bad):
    """Senior-review regression: a zero/negative/NaN variance must fail closed,
    never be published as a covariance claiming certainty or nonsense."""
    def check(node, got):
        assert node.odom is None and "yaw_variance_rad2" in node.startup_error
        assert _wait_for(lambda: len(got["status"]) >= 1)
        assert got["odom"] == []
    _run(_p(algorithm_package_path=str(REAL_LOCALIZATION_PATH), wheelbase_m=1.0, yaw_variance_rad2=bad), check)


@needs_real
def test_counts_mode_with_actuator_feedback_is_refused_not_silently_downgraded(ros_context):
    def check(node, got):
        assert node.odom is None and "COUNTS_MODE_UNSUPPORTED" in node.startup_error
        assert _wait_for(lambda: len(got["status"]) >= 1)
    _run(_p(algorithm_package_path=str(REAL_LOCALIZATION_PATH), wheelbase_m=1.0, counts_per_m=100.0,
            source="actuator_feedback"), check)


@needs_real
def test_absent_feedback_reports_stale_repeatedly(ros_context):
    def check(node, got):
        assert node.odom is not None
        assert _wait_for(lambda: sum(s.fault_state == "INPUT_STALE_OR_ABSENT" for s in got["status"]) >= 2)
        assert got["odom"] == []
    _run(_p(algorithm_package_path=str(REAL_LOCALIZATION_PATH), wheelbase_m=1.0, source="actuator_feedback"),
         check)


@needs_real
def test_steering_loss_degrades_and_freezes_yaw_but_keeps_publishing(ros_context):
    def check(node, got):
        assert _wait_for(lambda: any(s.quality == "DEGRADED" for s in got["status"]), timeout_s=5.0)
        assert all(s.fault_state == "STEERING_UNAVAILABLE_YAW_FROZEN"
                   for s in got["status"] if s.quality == "DEGRADED")
        n = len(got["odom"])
        assert _wait_for(lambda: len(got["odom"]) >= n + 10)
        tail = got["odom"][-8:]
        assert max(_yaw(o) for o in tail) - min(_yaw(o) for o in tail) < 1e-9, "yaw must be frozen, not guessed"
        assert tail[-1].pose.pose.position.x > tail[0].pose.pose.position.x or \
            tail[-1].pose.pose.position.y != tail[0].pose.pose.position.y, "translation must continue"
    _run(_p(algorithm_package_path=str(REAL_LOCALIZATION_PATH), wheelbase_m=1.0, synthetic_steering_rad=0.2,
            synthetic_steering_invalid_after=10), check)


@needs_real
def test_speed_loss_stops_odometry_entirely(ros_context):
    def check(node, got):
        assert _wait_for(lambda: any(s.fault_state == "INPUT_INVALID" for s in got["status"]))
        n = len(got["odom"])
        time.sleep(0.4)
        assert len(got["odom"]) <= n + 1, "no frozen/garbage pose may be republished after speed loss"
        assert got["status"][-1].quality == "INVALID"
    _run(_p(algorithm_package_path=str(REAL_LOCALIZATION_PATH), wheelbase_m=1.0,
            synthetic_speed_invalid_after=10), check)


@needs_real
def test_timing_gap_is_rejected_then_recovers(ros_context):
    def check(node, got):
        assert _wait_for(lambda: any(s.fault_state == "TIMING_OR_SPEED_INVALID" for s in got["status"]))
        i = next(k for k, s in enumerate(got["status"]) if s.fault_state == "TIMING_OR_SPEED_INVALID")
        assert _wait_for(lambda: any(s.quality == "OK" for s in got["status"][i + 1:]))
    _run(_p(algorithm_package_path=str(REAL_LOCALIZATION_PATH), wheelbase_m=1.0, synthetic_gap_after=10), check)


@needs_real
def test_nonmonotonic_timestamp_is_rejected_then_recovers(ros_context):
    def check(node, got):
        assert _wait_for(lambda: any(s.fault_state == "NON_MONOTONIC_TIMESTAMP" for s in got["status"]))
        i = next(k for k, s in enumerate(got["status"]) if s.fault_state == "NON_MONOTONIC_TIMESTAMP")
        assert _wait_for(lambda: any(s.quality == "OK" for s in got["status"][i + 1:]))
    _run(_p(algorithm_package_path=str(REAL_LOCALIZATION_PATH), wheelbase_m=1.0,
            synthetic_nonmonotonic_after=10), check)


def test_sigint_shutdown_exits_cleanly_without_rclerror():
    """Real entry point as its own process, stopped with SIGINT: exit 0, no
    traceback (the defect found in autokart_vcu_bridge/autokart_imu main())."""
    import signal
    import subprocess
    import sys
    pkg_root = Path(__file__).resolve().parent.parent
    env = dict(os.environ)
    env["PYTHONPATH"] = str(pkg_root) + os.pathsep + env.get("PYTHONPATH", "")
    proc = subprocess.Popen([sys.executable, "-c", "from autokart_odometry.node import main; main()"],
                            cwd=str(pkg_root), env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True)
    try:
        time.sleep(2.5)
        assert proc.poll() is None, proc.stderr.read()
        proc.send_signal(signal.SIGINT)
        _, err = proc.communicate(timeout=10)
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
    assert "Traceback" not in err, err
    assert proc.returncode == 0, (proc.returncode, err)
