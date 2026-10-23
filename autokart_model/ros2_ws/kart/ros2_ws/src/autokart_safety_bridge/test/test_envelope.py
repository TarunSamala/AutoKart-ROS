import math

from autokart_safety_bridge_ros.envelope import (
    STATE_BRAKING,
    STATE_CLEAR,
    STATE_CRITICAL,
    STATE_UNKNOWN,
    EnvelopeParameters,
    braking_trigger_distance_m,
    classify,
    nearest_obstacle_clearance_m,
)

PARAMS = EnvelopeParameters()


def test_trigger_distance_matches_hand_calculation_at_2mps():
    # 1.0 + 2*0.25 + 2**2/(2*2.0) + 0.3 = 1.0 + 0.5 + 1.0 + 0.3 = 2.8
    assert math.isclose(braking_trigger_distance_m(2.0, PARAMS), 2.8, rel_tol=1e-9)


def test_trigger_distance_grows_with_speed():
    assert braking_trigger_distance_m(10.0, PARAMS) > braking_trigger_distance_m(2.0, PARAMS)


def test_trigger_distance_clamps_negative_speed_to_zero():
    assert braking_trigger_distance_m(-5.0, PARAMS) == braking_trigger_distance_m(0.0, PARAMS)


def test_nearest_obstacle_ignores_points_behind_bumper():
    positions = [(0.5, 0.0)]  # behind the bumper (bumper_setback_m=1.0668)
    assert nearest_obstacle_clearance_m(positions, PARAMS) is None


def test_nearest_obstacle_ignores_points_outside_corridor_width():
    positions = [(2.0, 1.0)]  # 1.0 m to the side, wider than corridor_half_width_m
    assert nearest_obstacle_clearance_m(positions, PARAMS) is None


def test_nearest_obstacle_picks_closest_in_corridor():
    positions = [(3.0, 0.0), (2.0668, 0.0), (5.0, -0.4)]
    assert math.isclose(nearest_obstacle_clearance_m(positions, PARAMS), 1.0, rel_tol=1e-9)


def test_classify_unknown_when_odometry_stale():
    result = classify(
        speed_mps=0.0, odometry_fresh=False, obstacle_feed_fresh=True,
        obstacle_positions_xy=[], params=PARAMS)
    assert result.state == STATE_UNKNOWN
    assert result.valid is False


def test_classify_unknown_when_obstacle_feed_stale_even_with_no_obstacles():
    result = classify(
        speed_mps=0.0, odometry_fresh=True, obstacle_feed_fresh=False,
        obstacle_positions_xy=[], params=PARAMS)
    assert result.state == STATE_UNKNOWN
    assert result.valid is False


def test_classify_clear_when_fresh_and_no_obstacles():
    result = classify(
        speed_mps=3.0, odometry_fresh=True, obstacle_feed_fresh=True,
        obstacle_positions_xy=[], params=PARAMS)
    assert result.state == STATE_CLEAR
    assert result.valid is True
    assert result.clearance_m is None


def test_classify_critical_within_minimum_clearance():
    # obstacle 0.5 m ahead of the bumper
    positions = [(PARAMS.bumper_setback_m + 0.5, 0.0)]
    result = classify(
        speed_mps=0.0, odometry_fresh=True, obstacle_feed_fresh=True,
        obstacle_positions_xy=positions, params=PARAMS)
    assert result.state == STATE_CRITICAL


def test_classify_braking_inside_speed_dependent_envelope_but_outside_minimum():
    # at 2 m/s the trigger distance is 2.8 m; put the obstacle at 2.0 m clearance
    positions = [(PARAMS.bumper_setback_m + 2.0, 0.0)]
    result = classify(
        speed_mps=2.0, odometry_fresh=True, obstacle_feed_fresh=True,
        obstacle_positions_xy=positions, params=PARAMS)
    assert result.state == STATE_BRAKING


def test_classify_clear_when_obstacle_beyond_trigger_distance():
    positions = [(PARAMS.bumper_setback_m + 10.0, 0.0)]
    result = classify(
        speed_mps=2.0, odometry_fresh=True, obstacle_feed_fresh=True,
        obstacle_positions_xy=positions, params=PARAMS)
    assert result.state == STATE_CLEAR


def test_classify_never_reports_clear_when_speed_missing():
    result = classify(
        speed_mps=None, odometry_fresh=True, obstacle_feed_fresh=True,
        obstacle_positions_xy=[], params=PARAMS)
    assert result.state == STATE_UNKNOWN
