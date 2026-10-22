"""Unit tests for PlannerCore. No rclpy/ROS runtime needed."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from autokart_planner.algorithms import BuiltinAlgorithmBackend  # noqa: E402
from autokart_planner.planner_core import PlannerCore  # noqa: E402


def straight_path(n=11, step=1.0):
    return [(float(i) * step, 0.0) for i in range(n)]


def make_core(**overrides):
    defaults = dict(
        algorithm=BuiltinAlgorithmBackend(),
        a_lat_max_mps2=2.0,
        a_brake_min_mps2=2.0,
        t_total_latency_s=0.2,
        d_margin_m=1.0,
        footprint_width_m=1.0,
        speed_limit_mps=1.0,
    )
    defaults.update(overrides)
    return PlannerCore(**defaults)


def test_clear_straight_path_is_valid_with_full_speed_profile():
    core = make_core()
    result = core.evaluate(path_points=straight_path(), path_stamp_ns=1_000_000_000,
                           now_ns=1_000_000_000, pose_x=0.0, pose_y=0.0, pose_speed_mps=0.5,
                           pose_fresh=True, obstacles=[])
    assert result["valid"] is True
    assert result["decision"] == "PATH_CANDIDATE"
    assert result["reason_codes"] == []
    assert len(result["speed_profile"]) == 11
    assert all(v == pytest.approx(1.0) for v in result["speed_profile"])  # capped by speed_limit_mps
    assert result["path_odom"] == straight_path()


def test_stale_odom_is_rejected_and_never_publishes_a_path():
    core = make_core()
    result = core.evaluate(path_points=straight_path(), path_stamp_ns=1_000_000_000,
                           now_ns=1_000_000_000, pose_x=0.0, pose_y=0.0, pose_speed_mps=0.5,
                           pose_fresh=False, obstacles=[])
    assert result["valid"] is False
    assert result["reason_codes"] == ["ODOM_MISSING_OR_STALE"]
    assert result["path_odom"] == []
    assert result["speed_profile"] == []


def test_expired_path_is_rejected():
    core = make_core(max_path_age_ns=100_000_000)
    result = core.evaluate(path_points=straight_path(), path_stamp_ns=1_000_000_000,
                           now_ns=1_000_000_000 + 200_000_000, pose_x=0.0, pose_y=0.0,
                           pose_speed_mps=0.5, pose_fresh=True, obstacles=[])
    assert result["valid"] is False
    assert result["reason_codes"] == ["PATH_EXPIRED"]


def test_path_stamped_in_the_future_is_rejected():
    core = make_core()
    result = core.evaluate(path_points=straight_path(), path_stamp_ns=2_000_000_000,
                           now_ns=1_000_000_000, pose_x=0.0, pose_y=0.0, pose_speed_mps=0.5,
                           pose_fresh=True, obstacles=[])
    assert result["reason_codes"] == ["PATH_TIMESTAMP_INVALID"]


def test_too_short_path_is_rejected():
    core = make_core()
    result = core.evaluate(path_points=[(0.0, 0.0)], path_stamp_ns=1_000_000_000,
                           now_ns=1_000_000_000, pose_x=0.0, pose_y=0.0, pose_speed_mps=0.0,
                           pose_fresh=True, obstacles=[])
    assert result["reason_codes"] == ["PATH_TOO_SHORT"]


def test_nonfinite_path_point_is_rejected():
    core = make_core()
    path = straight_path()
    path[3] = (float("nan"), 0.0)
    result = core.evaluate(path_points=path, path_stamp_ns=1_000_000_000, now_ns=1_000_000_000,
                           pose_x=0.0, pose_y=0.0, pose_speed_mps=0.0, pose_fresh=True, obstacles=[])
    assert result["reason_codes"] == ["PATH_NONFINITE"]


def test_unqualified_brake_capability_fails_closed_by_default():
    # a_brake_min_mps2=0.0 is the node's own documented default (see config/planner_builtin.yaml):
    # unconfigured, not a guessed nonzero number, and must fail closed, not silently cruise.
    core = make_core(a_brake_min_mps2=0.0)
    result = core.evaluate(path_points=straight_path(), path_stamp_ns=1_000_000_000,
                           now_ns=1_000_000_000, pose_x=0.0, pose_y=0.0, pose_speed_mps=0.5,
                           pose_fresh=True, obstacles=[])
    assert result["valid"] is False
    assert "BRAKE_CAPABILITY_UNQUALIFIED" in result["reason_codes"][0]


def test_unqualified_lateral_limit_fails_closed():
    core = make_core(a_lat_max_mps2=0.0)
    result = core.evaluate(path_points=straight_path(), path_stamp_ns=1_000_000_000,
                           now_ns=1_000_000_000, pose_x=0.0, pose_y=0.0, pose_speed_mps=0.5,
                           pose_fresh=True, obstacles=[])
    assert result["reason_codes"] == ["LATERAL_LIMIT_UNQUALIFIED"]


def test_curved_path_caps_speed_below_speed_limit():
    # A sharp-enough arc must cap commanded speed under the flat speed_limit_mps.
    import math
    radius = 1.0
    points = [(radius * math.sin(a), radius - radius * math.cos(a))
              for a in (i * 0.1 for i in range(10))]
    core = make_core(a_lat_max_mps2=1.0, speed_limit_mps=5.0, d_margin_m=0.0)
    result = core.evaluate(path_points=points, path_stamp_ns=1_000_000_000, now_ns=1_000_000_000,
                           pose_x=0.0, pose_y=0.0, pose_speed_mps=0.0, pose_fresh=True, obstacles=[])
    assert result["valid"] is True
    assert any(v < 5.0 for v in result["speed_profile"]), "sharp curvature should cap below the flat limit"
    for v, k in zip(result["speed_profile"], result["curvature_profile"]):
        if abs(k) > 1e-3:
            assert v == pytest.approx(math.sqrt(1.0 / abs(k)), rel=1e-6)


def test_obstacle_within_stopping_distance_blocks_and_zeroes_speed_profile():
    core = make_core(a_brake_min_mps2=1.0, t_total_latency_s=0.0, d_margin_m=0.5)
    # stopping_distance(2.0) = 0 + 4/(2*1) + 0.5 = 2.5 m; obstacle 2.0 m ahead is inside that.
    result = core.evaluate(path_points=straight_path(n=11), path_stamp_ns=1_000_000_000,
                           now_ns=1_000_000_000, pose_x=0.0, pose_y=0.0, pose_speed_mps=2.0,
                           pose_fresh=True, obstacles=[{"x_m": 2.0, "y_m": 0.0, "radius_m": 0.2}])
    assert result["valid"] is False
    assert result["decision"] == "STOP_CANDIDATE"
    assert result["reason_codes"] == ["CORRIDOR_BLOCKED"]
    assert result["speed_profile"] == [0.0] * 11
    assert result["path_odom"] == []


def test_obstacle_beyond_stopping_distance_still_allows_cruising():
    core = make_core(a_brake_min_mps2=5.0, t_total_latency_s=0.0, d_margin_m=0.2)
    # stopping_distance(1.0) = 1/(2*5) + 0.2 = 0.3 m; obstacle far beyond that at x=9.
    result = core.evaluate(path_points=straight_path(n=11), path_stamp_ns=1_000_000_000,
                           now_ns=1_000_000_000, pose_x=0.0, pose_y=0.0, pose_speed_mps=1.0,
                           pose_fresh=True, obstacles=[{"x_m": 9.0, "y_m": 0.0, "radius_m": 0.2}])
    assert result["valid"] is True
    assert result["decision"] == "PATH_CANDIDATE"


def test_obstacle_behind_the_vehicle_is_ignored():
    core = make_core(a_brake_min_mps2=1.0, t_total_latency_s=0.0, d_margin_m=0.1)
    result = core.evaluate(path_points=straight_path(n=11), path_stamp_ns=1_000_000_000,
                           now_ns=1_000_000_000, pose_x=5.0, pose_y=0.0, pose_speed_mps=0.0,
                           pose_fresh=True, obstacles=[{"x_m": 1.0, "y_m": 0.0, "radius_m": 0.2}])
    assert result["valid"] is True


def test_malformed_obstacle_entry_rejects_rather_than_crashing():
    core = make_core()
    result = core.evaluate(path_points=straight_path(), path_stamp_ns=1_000_000_000,
                           now_ns=1_000_000_000, pose_x=0.0, pose_y=0.0, pose_speed_mps=0.0,
                           pose_fresh=True, obstacles=[{"x_m": "not-a-number", "y_m": 0.0, "radius_m": 0.1}])
    assert result["valid"] is False
    assert result["reason_codes"] == ["INVALID_OBSTACLE"]


def test_zero_curvature_epsilon_rejects_instead_of_dividing_by_zero():
    # Regression test: curvature_eps_m_inv=0.0 combined with a perfectly straight path (kappa=0
    # everywhere) used to raise an uncaught ZeroDivisionError inside the speed-profile
    # comprehension (a_lat_max_mps2 / max(abs(k), 0.0) == a_lat_max_mps2 / 0.0). Confirmed crashing
    # on the pre-fix code before this guard was added (see planner_core.py).
    core = make_core(curvature_eps_m_inv=0.0)
    result = core.evaluate(path_points=straight_path(), path_stamp_ns=1_000_000_000,
                           now_ns=1_000_000_000, pose_x=0.0, pose_y=0.0, pose_speed_mps=0.0,
                           pose_fresh=True, obstacles=[])
    assert result["valid"] is False
    assert result["reason_codes"] == ["CURVATURE_EPSILON_UNQUALIFIED"]


def test_negative_curvature_epsilon_also_rejects():
    core = make_core(curvature_eps_m_inv=-1.0)
    result = core.evaluate(path_points=straight_path(), path_stamp_ns=1_000_000_000,
                           now_ns=1_000_000_000, pose_x=0.0, pose_y=0.0, pose_speed_mps=0.0,
                           pose_fresh=True, obstacles=[])
    assert result["reason_codes"] == ["CURVATURE_EPSILON_UNQUALIFIED"]


def test_sequence_increments_across_calls():
    core = make_core()
    r1 = core.evaluate(path_points=straight_path(), path_stamp_ns=1_000_000_000, now_ns=1_000_000_000,
                       pose_x=0.0, pose_y=0.0, pose_speed_mps=0.0, pose_fresh=True, obstacles=[])
    r2 = core.evaluate(path_points=straight_path(), path_stamp_ns=1_020_000_000, now_ns=1_020_000_000,
                       pose_x=0.0, pose_y=0.0, pose_speed_mps=0.0, pose_fresh=True, obstacles=[])
    assert r2["sequence"] == r1["sequence"] + 1
