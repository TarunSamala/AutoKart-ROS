"""Unit tests: synthetic source + reused-algorithm loader. No ROS runtime needed."""
import math
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from autokart_odometry.backends import (  # noqa: E402
    OdometryBackendError, SyntheticSampleSource, load_wheel_odometry,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
# AUTOKART_LOCALIZATION_PATH overrides this (e.g. an export bundle's third_party/ copy).
REAL_LOCALIZATION_PATH = Path(os.environ.get(
    "AUTOKART_LOCALIZATION_PATH", str(REPO_ROOT / "kart" / "ros2_ws" / "src" / "autokart_localization")))
needs_real = pytest.mark.skipif(not REAL_LOCALIZATION_PATH.is_dir(),
                                reason="real autokart_localization checkout not present")


@pytest.fixture(autouse=True)
def _isolate_lazy_import_state():
    """Same reason as autokart_vcu_bridge's fixture: tests point the loader at
    a real and at fake `autokart_localization` directories in one process, and
    sys.modules would otherwise cache whichever was imported first."""
    saved = list(sys.path)
    yield
    sys.path[:] = saved
    for name in list(sys.modules):
        if name == "autokart_localization" or name.startswith("autokart_localization."):
            del sys.modules[name]


def test_synthetic_source_is_deterministic_and_monotonic():
    a = SyntheticSampleSource(start_ns=0, rate_hz=50.0, speed_mps=1.0, counts_per_m=100.0)
    b = SyntheticSampleSource(start_ns=0, rate_hz=50.0, speed_mps=1.0, counts_per_m=100.0)
    sa = [a.next_sample() for _ in range(10)]
    sb = [b.next_sample() for _ in range(10)]
    assert sa == sb
    assert all(y.stamp_ns > x.stamp_ns for x, y in zip(sa, sa[1:]))
    assert sa[-1].axle_counts == 20  # 10 samples * 0.02 s * 1 m/s * 100 counts/m


def test_synthetic_source_hooks():
    s = SyntheticSampleSource(start_ns=0, rate_hz=50.0, steering_invalid_after=2, speed_invalid_after=4,
                              gap_after=3, nonmonotonic_after=5)
    x = [s.next_sample() for _ in range(5)]
    assert x[0].steering_rad is not None and x[1].steering_rad is None
    assert x[2].stamp_ns - x[1].stamp_ns > 400_000_000      # gap
    assert x[3].valid is False and math.isnan(x[3].speed_mps)
    assert x[4].stamp_ns < x[3].stamp_ns                       # goes backwards


def test_synthetic_source_rejects_bad_rate():
    with pytest.raises(OdometryBackendError):
        SyntheticSampleSource(start_ns=0, rate_hz=0.0)


def test_loader_requires_configured_path():
    with pytest.raises(OdometryBackendError) as exc:
        load_wheel_odometry("")
    assert "ALGORITHM_PATH_NOT_CONFIGURED" in str(exc.value)


def test_loader_missing_path(tmp_path):
    with pytest.raises(OdometryBackendError) as exc:
        load_wheel_odometry(str(tmp_path / "nope"))
    assert "DEPENDENCY_PATH_NOT_FOUND" in str(exc.value)


def test_loader_package_without_odometry_module(tmp_path):
    pkg = tmp_path / "autokart_localization"
    pkg.mkdir()
    (pkg / "__init__.py").write_text("")
    with pytest.raises(OdometryBackendError) as exc:
        load_wheel_odometry(str(tmp_path))
    assert "ALGORITHM_IMPORT_FAILED" in str(exc.value)


def test_loader_detects_api_mismatch(tmp_path):
    pkg = tmp_path / "autokart_localization"
    pkg.mkdir()
    (pkg / "__init__.py").write_text("")
    (pkg / "odometry.py").write_text("class WheelOdometry: pass\n")  # missing the rest of the API
    with pytest.raises(OdometryBackendError) as exc:
        load_wheel_odometry(str(tmp_path))
    assert "ALGORITHM_API_MISMATCH" in str(exc.value)


@needs_real
def test_real_algorithm_straight_line_via_counts_matches_synthetic_distance():
    """Real reuse under real test: synthetic samples fed through the genuine
    WheelOdometry.update_counts() -- 1 m/s for 1 s straight -> x ~= 1 m."""
    algo = load_wheel_odometry(str(REAL_LOCALIZATION_PATH))
    odo = algo.WheelOdometry(algo.OdometryConfig(wheelbase_m=1.0, counts_per_m=100.0))
    src = SyntheticSampleSource(start_ns=0, rate_hz=50.0, speed_mps=1.0, steering_rad=0.0, counts_per_m=100.0)
    prev = src.next_sample()
    for _ in range(50):
        cur = src.next_sample()
        assert odo.update_counts((cur.stamp_ns - prev.stamp_ns) / 1e9, cur.axle_counts - prev.axle_counts,
                                 cur.steering_rad)
        prev = cur
    assert abs(odo.x_m - 1.0) < 0.02 and abs(odo.y_m) < 1e-9
    assert odo.status == algo.STATUS_ENCODER_STEER_MODEL


@needs_real
def test_real_algorithm_signed_reverse_motion_reduces_x():
    """Signed displacement: reversing moves x backwards. (There is no separate
    unsigned path-length field in WheelOdometry to check -- see
    DEVELOPER_REPORT.md.)"""
    algo = load_wheel_odometry(str(REAL_LOCALIZATION_PATH))
    odo = algo.WheelOdometry(algo.OdometryConfig(wheelbase_m=1.0))
    for _ in range(10):
        assert odo.update(0.02, 1.0, 0.0)
    for _ in range(10):
        assert odo.update(0.02, -1.0, 0.0)
    assert abs(odo.x_m) < 1e-9
    assert not hasattr(odo, "distance_travelled_total")
