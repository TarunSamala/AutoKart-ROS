"""Unit tests for ControllerCore. No rclpy/ROS runtime needed."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from autokart_controller.pursuit_backends import BuiltinPursuitBackend  # noqa: E402
from autokart_controller.controller_core import ControllerCore  # noqa: E402


def straight_path(n=11, step=1.0):
    return [(float(i) * step, 0.0) for i in range(n)]


def make_core(**overrides):
    defaults = dict(backend=BuiltinPursuitBackend(wheelbase_m=1.0), maximum_speed_mps=5.0)
    defaults.update(overrides)
    return ControllerCore(**defaults)


def base_kwargs(**overrides):
    defaults = dict(
        path_points=straight_path(), path_stamp_ns=1_000_000_000, now_ns=1_000_000_000,
        envelope_valid=True, envelope_max_speed_mps=1.0, speed_profile=[],
        pose_x=0.0, pose_y=0.0, pose_yaw_rad=0.0, pose_speed_mps=0.5, pose_fresh=True, dt_s=0.1,
    )
    defaults.update(overrides)
    return defaults


def test_valid_inputs_produce_a_signed_candidate_on_the_path_centerline():
    core = make_core()
    result = core.evaluate(**base_kwargs())
    assert result["valid"] is True
    assert result["reason_codes"] == []
    assert abs(result["steering_rad"]) < 1e-6
    assert result["speed_mps"] > 0.0


def test_invalid_envelope_rejects_with_reason():
    core = make_core()
    result = core.evaluate(**base_kwargs(envelope_valid=False))
    assert result["valid"] is False
    assert result["reason_codes"] == ["PLANNER_INVALID"]
    assert result["speed_mps"] == 0.0
    assert result["steering_rad"] == 0.0


def test_stale_pose_rejects():
    core = make_core()
    result = core.evaluate(**base_kwargs(pose_fresh=False))
    assert result["reason_codes"] == ["POSE_MISSING_OR_STALE"]


def test_path_stamped_in_the_future_rejects():
    core = make_core()
    result = core.evaluate(**base_kwargs(path_stamp_ns=2_000_000_000, now_ns=1_000_000_000))
    assert result["reason_codes"] == ["PATH_TIMESTAMP_INVALID"]


def test_expired_path_rejects():
    core = make_core(max_path_age_ns=100_000_000)
    result = core.evaluate(**base_kwargs(now_ns=1_000_000_000 + 200_000_000))
    assert result["reason_codes"] == ["PATH_EXPIRED"]


def test_short_path_rejects():
    core = make_core()
    result = core.evaluate(**base_kwargs(path_points=[(0.0, 0.0)]))
    assert result["reason_codes"] == ["PATH_TOO_SHORT"]


def test_nonfinite_path_rejects():
    core = make_core()
    path = straight_path()
    path[2] = (float("nan"), 0.0)
    result = core.evaluate(**base_kwargs(path_points=path))
    assert result["reason_codes"] == ["PATH_NONFINITE"]


def test_zero_dt_rejects():
    core = make_core()
    result = core.evaluate(**base_kwargs(dt_s=0.0))
    assert result["reason_codes"] == ["CONTROLLER_TIMING_INVALID"]


def test_negative_pose_speed_rejects():
    core = make_core()
    result = core.evaluate(**base_kwargs(pose_speed_mps=-1.0))
    assert result["reason_codes"] == ["POSE_SPEED_INVALID"]


def test_speed_is_capped_by_configured_maximum_speed():
    core = make_core(maximum_speed_mps=0.3)
    result = core.evaluate(**base_kwargs(envelope_max_speed_mps=10.0))
    assert result["valid"] is True
    assert result["speed_mps"] <= 0.3 + 1e-9


def test_curved_path_produces_nonzero_steering_toward_the_target():
    core = make_core()
    path = [(float(x), float(x) * 0.3) for x in range(0, 11)]
    result = core.evaluate(**base_kwargs(path_points=path))
    assert result["valid"] is True
    assert result["steering_rad"] > 0.0


def test_per_point_speed_profile_overrides_the_flat_envelope_cap_when_aligned():
    core = make_core(maximum_speed_mps=5.0)
    path = straight_path(n=5)
    profile = [0.1, 0.2, 0.3, 0.4, 5.0]  # last point's speed should be the one the lookahead picks
    result = core.evaluate(**base_kwargs(path_points=path, speed_profile=profile,
                                         envelope_max_speed_mps=5.0, pose_speed_mps=0.0))
    assert result["valid"] is True


def test_misaligned_speed_profile_falls_back_to_the_flat_envelope_cap():
    core = make_core()
    path = straight_path(n=5)
    misaligned_profile = [0.1, 0.2]  # wrong length: must not be used as-is
    result = core.evaluate(**base_kwargs(path_points=path, speed_profile=misaligned_profile,
                                         envelope_max_speed_mps=0.7))
    assert result["valid"] is False
    assert result["reason_codes"] == ["SPEED_PROFILE_INVALID"]


def test_brake_stuck_zeroes_drive_but_still_reports_valid_candidate():
    core = make_core()
    result = core.evaluate(**base_kwargs(brake_stuck=True))
    assert result["valid"] is False
    assert result["reason_codes"] == ["ACTUATOR_FEEDBACK_INVALID"]
    assert result["drive"] == 0.0


def test_sequence_increments_across_calls():
    core = make_core()
    r1 = core.evaluate(**base_kwargs(path_stamp_ns=1_000_000_000, now_ns=1_000_000_000))
    r2 = core.evaluate(**base_kwargs(path_stamp_ns=1_020_000_000, now_ns=1_020_000_000))
    assert r2["sequence"] == r1["sequence"] + 1


def test_nan_speed_profile_fails_closed():
    result = make_core().evaluate(**base_kwargs(speed_profile=[float("nan")]*11))
    assert not result["valid"]
    assert result["reason_codes"] == ["SPEED_PROFILE_INVALID"]
