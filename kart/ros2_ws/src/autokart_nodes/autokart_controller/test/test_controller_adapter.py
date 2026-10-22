from kart_sim.types import PlanarKartState

from autokart_controller.engine import ControllerEngine


def straight_path(length_m=10.0, step_m=1.0):
    count = int(length_m / step_m) + 1
    return [(step_m * index, 0.0) for index in range(count)]


def valid_status(stamp_ns, target_speed_mps=2.0, expires_ns=None):
    return {
        "valid": True,
        "sample_time_ns": stamp_ns,
        "expires_ns": expires_ns if expires_ns is not None else stamp_ns + 100_000_000,
        "target_speed_mps": target_speed_mps,
        "mission_state": "RIGHT_LAP_1",
        "decision": "LOOP_PATH_CANDIDATE",
    }


def pose_at_origin(time_ns):
    return PlanarKartState(time_s=time_ns / 1e9, x_m=0.0, y_m=0.0, yaw_rad=0.0, u_mps=1.0)


def test_valid_inputs_produce_a_signed_candidate_on_the_path_centerline():
    engine = ControllerEngine(source="SIMULATED", wheelbase_m=1.0)
    stamp = 5_000_000_000
    result = engine.evaluate(
        path_points=straight_path(),
        path_stamp_ns=stamp,
        planner_status=valid_status(stamp),
        pose=pose_at_origin(stamp),
        pose_valid=True,
        pose_time_ns=stamp,
        now_ns=stamp,
        dt_s=0.1,
    )
    assert result["valid"] is True
    assert result["reason_codes"] == []
    assert result["target_speed_mps"] > 0.0
    assert abs(result["steering_angle_rad"]) < 1e-6
    assert result["integrity"] != 0
    assert result["mission_state"] == "RIGHT_LAP_1"


def test_missing_planner_status_rejects_with_reason():
    engine = ControllerEngine()
    result = engine.evaluate(
        path_points=straight_path(),
        path_stamp_ns=1_000,
        planner_status=None,
        pose=pose_at_origin(1_000),
        pose_valid=True,
        pose_time_ns=1_000,
        now_ns=1_000,
        dt_s=0.1,
    )
    assert result["valid"] is False
    assert result["reason_codes"] == ["PLANNER_INVALID"]
    assert result["target_speed_mps"] == 0.0


def test_invalid_planner_status_rejects():
    engine = ControllerEngine()
    stamp = 2_000_000_000
    status = valid_status(stamp)
    status["valid"] = False
    result = engine.evaluate(
        path_points=straight_path(),
        path_stamp_ns=stamp,
        planner_status=status,
        pose=pose_at_origin(stamp),
        pose_valid=True,
        pose_time_ns=stamp,
        now_ns=stamp,
        dt_s=0.1,
    )
    assert result["reason_codes"] == ["PLANNER_INVALID"]


def test_mismatched_path_and_status_stamp_rejects():
    engine = ControllerEngine()
    stamp = 3_000_000_000
    result = engine.evaluate(
        path_points=straight_path(),
        path_stamp_ns=stamp,
        planner_status=valid_status(stamp + 20_000_000),
        pose=pose_at_origin(stamp),
        pose_valid=True,
        pose_time_ns=stamp,
        now_ns=stamp,
        dt_s=0.1,
    )
    assert result["reason_codes"] == ["PATH_STATUS_MISMATCH"]


def test_expired_planner_status_rejects():
    engine = ControllerEngine()
    stamp = 4_000_000_000
    status = valid_status(stamp, expires_ns=stamp + 10_000_000)
    result = engine.evaluate(
        path_points=straight_path(),
        path_stamp_ns=stamp,
        planner_status=status,
        pose=pose_at_origin(stamp),
        pose_valid=True,
        pose_time_ns=stamp,
        now_ns=stamp + 50_000_000,
        dt_s=0.1,
    )
    assert result["reason_codes"] == ["PLANNER_STATUS_EXPIRED"]


def test_short_path_rejects():
    engine = ControllerEngine()
    stamp = 6_000_000_000
    result = engine.evaluate(
        path_points=[(0.0, 0.0)],
        path_stamp_ns=stamp,
        planner_status=valid_status(stamp),
        pose=pose_at_origin(stamp),
        pose_valid=True,
        pose_time_ns=stamp,
        now_ns=stamp,
        dt_s=0.1,
    )
    assert result["reason_codes"] == ["PATH_TOO_SHORT"]


def test_invalid_pose_rejects():
    engine = ControllerEngine()
    stamp = 7_000_000_000
    result = engine.evaluate(
        path_points=straight_path(),
        path_stamp_ns=stamp,
        planner_status=valid_status(stamp),
        pose=pose_at_origin(stamp),
        pose_valid=False,
        pose_time_ns=stamp,
        now_ns=stamp,
        dt_s=0.1,
    )
    assert result["reason_codes"] == ["POSE_INVALID"]


def test_stale_pose_rejects():
    engine = ControllerEngine()
    stamp = 8_000_000_000
    result = engine.evaluate(
        path_points=straight_path(),
        path_stamp_ns=stamp,
        planner_status=valid_status(stamp),
        pose=pose_at_origin(stamp),
        pose_valid=True,
        pose_time_ns=stamp - 500_000_000,
        now_ns=stamp,
        dt_s=0.1,
    )
    assert result["reason_codes"] == ["POSE_STALE"]


def test_negative_target_speed_rejects():
    engine = ControllerEngine()
    stamp = 9_000_000_000
    status = valid_status(stamp, target_speed_mps=-1.0)
    result = engine.evaluate(
        path_points=straight_path(),
        path_stamp_ns=stamp,
        planner_status=status,
        pose=pose_at_origin(stamp),
        pose_valid=True,
        pose_time_ns=stamp,
        now_ns=stamp,
        dt_s=0.1,
    )
    assert result["reason_codes"] == ["TARGET_SPEED_INVALID"]


def test_zero_or_negative_dt_rejects():
    engine = ControllerEngine()
    stamp = 10_000_000_000
    result = engine.evaluate(
        path_points=straight_path(),
        path_stamp_ns=stamp,
        planner_status=valid_status(stamp),
        pose=pose_at_origin(stamp),
        pose_valid=True,
        pose_time_ns=stamp,
        now_ns=stamp,
        dt_s=0.0,
    )
    assert result["reason_codes"] == ["CONTROLLER_TIMING_INVALID"]


def test_curved_path_produces_nonzero_steering_toward_the_target():
    engine = ControllerEngine(source="SIMULATED", wheelbase_m=1.0, lookahead_m=1.5)
    stamp = 11_000_000_000
    path = [(x, x * 0.3) for x in range(0, 11)]
    result = engine.evaluate(
        path_points=path,
        path_stamp_ns=stamp,
        planner_status=valid_status(stamp),
        pose=pose_at_origin(stamp),
        pose_valid=True,
        pose_time_ns=stamp,
        now_ns=stamp,
        dt_s=0.1,
    )
    assert result["valid"] is True
    assert result["steering_angle_rad"] > 0.0


def test_speed_is_capped_by_configured_maximum():
    engine = ControllerEngine(maximum_speed_mps=1.0)
    stamp = 12_000_000_000
    status = valid_status(stamp, target_speed_mps=10.0)
    result = engine.evaluate(
        path_points=straight_path(),
        path_stamp_ns=stamp,
        planner_status=status,
        pose=pose_at_origin(stamp),
        pose_valid=True,
        pose_time_ns=stamp,
        now_ns=stamp,
        dt_s=0.1,
    )
    assert result["valid"] is True
    assert result["target_speed_mps"] <= 1.0
