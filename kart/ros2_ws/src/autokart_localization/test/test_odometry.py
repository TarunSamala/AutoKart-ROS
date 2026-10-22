import math

from autokart_localization.odometry import (
    STATUS_ENCODER_ONLY_NO_YAW, STATUS_ENCODER_STEER_MODEL, STATUS_INVALID, OdometryConfig, WheelOdometry,
)


def test_straight_line_no_steering_input():
    odo = WheelOdometry(OdometryConfig(wheelbase_m=1.0))
    for _ in range(100):
        assert odo.update(0.01, 1.0, 0.0)
    assert abs(odo.x_m - 1.0) < 1e-6 and abs(odo.y_m) < 1e-9
    assert odo.status == STATUS_ENCODER_STEER_MODEL


def test_turning_changes_yaw_with_steering_model():
    odo = WheelOdometry(OdometryConfig(wheelbase_m=1.0))
    for _ in range(1000):
        odo.update(0.01, 1.0, math.radians(10.0))
    assert odo.yaw_rad > 1.0
    assert odo.status == STATUS_ENCODER_STEER_MODEL


def test_missing_steering_freezes_yaw_but_keeps_speed():
    odo = WheelOdometry(OdometryConfig(wheelbase_m=1.0))
    assert odo.update(0.01, 1.0, None)
    assert odo.status == STATUS_ENCODER_ONLY_NO_YAW
    assert odo.yaw_rate_radps == 0.0
    assert odo.x_m > 0.0  # still integrates forward motion


def test_invalid_speed_marks_invalid_and_does_not_move():
    odo = WheelOdometry(OdometryConfig(wheelbase_m=1.0))
    odo.update(0.01, 1.0, 0.0)
    x_before, y_before = odo.x_m, odo.y_m
    assert not odo.update(0.01, float("nan"), 0.0)
    assert odo.status == STATUS_INVALID
    assert odo.x_m == x_before and odo.y_m == y_before  # frozen, not corrupted

    odo2 = WheelOdometry(OdometryConfig(wheelbase_m=1.0))
    assert not odo2.update(0.01, None, 0.0)
    assert odo2.status == STATUS_INVALID


def test_bad_dt_rejected():
    odo = WheelOdometry(OdometryConfig(wheelbase_m=1.0))
    assert not odo.update(0.0, 1.0, 0.0)
    assert not odo.update(-0.01, 1.0, 0.0)
    assert not odo.update(1.0, 1.0, 0.0)  # far too large a gap to trust


def test_reset_clears_state():
    odo = WheelOdometry(OdometryConfig(wheelbase_m=1.0))
    odo.update(0.01, 1.0, math.radians(5.0))
    odo.reset()
    assert odo.x_m == odo.y_m == odo.yaw_rad == odo.speed_mps == 0.0
    assert odo.status == STATUS_INVALID


def test_calibrated_count_delta_controls_distance_and_rejects_missing_calibration():
    odo = WheelOdometry(OdometryConfig(wheelbase_m=1.0, counts_per_m=100.0))
    assert odo.update_counts(0.1, 25, 0.0)
    assert odo.x_m == 0.25
    assert odo.speed_mps == 2.5
    fallback = WheelOdometry(OdometryConfig())
    assert not fallback.update_counts(0.1, 25, 0.0)
    reversed_encoder = WheelOdometry(OdometryConfig(counts_per_m=100.0, encoder_forward_sign=-1))
    assert reversed_encoder.update_counts(0.1, -25, 0.0)
    assert reversed_encoder.x_m == 0.25
