import math

from autokart_vehicle_bridge.core import (
    CommandGate, ThrottleConfig, ThrottleController, arm_request_approved, brake_pct_for_candidate,
    raw_to_rad, wrap_raw_delta,
)
from autokart_vehicle_bridge.protocol import (
    THROTTLE_MAX_MV, build_cmd, build_frame, build_observation, build_simple, crc16_ccitt, parse_frame, parse_message,
)


def test_crc_check_value():
    assert crc16_ccitt(b"123456789") == 0x29B1


def test_arm_request_requires_fresh_supervisor_arming_and_fresh_request():
    assert arm_request_approved(10.0, True, 9.9, 9.95)
    assert not arm_request_approved(10.0, False, 9.9, 9.95)
    assert not arm_request_approved(10.0, True, None, 9.95)
    assert not arm_request_approved(10.0, True, 9.5, 9.95)
    assert not arm_request_approved(10.0, True, 9.9, 9.5)


def test_frame_roundtrip_and_corruption():
    f = build_cmd(7, 500, -120, 0)
    assert parse_frame(f) == "CMD,7,500,-120,0"
    bad = bytearray(f)
    bad[4] ^= 1
    assert parse_frame(bytes(bad)) is None
    assert parse_frame("garbage") is None
    assert parse_frame("$X*ZZZZ") is None
    assert build_frame("CMD,1,0,0,0").endswith(b"\n")


def test_parse_tel_and_nak_and_hello():
    payload = "TEL,5,12345,1,0,49,1,0,1500,-300,820,2050,-1,42,15"
    kind, t = parse_message(payload)
    assert kind == "TEL" and t.state_name == "ARMED" and t.estop_input_fitted and t.estop_ok and t.dac_ok and not t.steer_pwm_ok
    _, absent = parse_message("TEL,5,12345,0,0,16,0,1,0,-1,-1,-1,-1,42,15")
    assert not absent.estop_input_fitted and not absent.estop_ok and absent.dac_ok
    assert t.speed_mmps == 820 and t.steer_raw == 2050 and t.brake_raw == -1
    assert parse_message("TEL,1,2") is None
    assert parse_message("HELLO,0.2.2-no-estop-input-locked,2") == ("HELLO", ("0.2.2-no-estop-input-locked", 2))
    assert parse_message("NAK,9,3") == ("NAK", (9, 3))
    assert build_simple("ARM", 1).startswith(b"$ARM,1*")


def test_observation_wire_roundtrip_and_invalid_values():
    assert parse_frame(build_observation(7, 123, 9, 1217, 150, 2)) == "OBS,7,123,9,1217,150,2"
    assert parse_message(parse_frame(build_frame("OBSACK,7,2"))) == ("OBSACK", (7, 2))
    assert parse_message("OBSACK,7,3") is None
    import pytest
    with pytest.raises(ValueError):
        build_observation(1, 2, 3, 4, 5, 3)


def test_throttle_controller_never_exceeds_absolute_cap():
    cfg = ThrottleConfig(volts_per_mps=10.0, throttle_v_max=9.0, throttle_run_v=9.0)
    ctl = ThrottleController(cfg)
    for k in range(200):
        out = ctl.step(k * 0.05, 5.0, 0.0)
        assert 0.0 <= out <= THROTTLE_MAX_MV / 1000.0
    assert ctl.step(20.0, 0.0, 0.0) == 0.0  # target <= 0 drops straight to 0 V


def test_throttle_controller_run_floor_never_sits_between_zero_and_run():
    # Default config: run == max == 2.0 V, so any positive target is exactly 2.0 V (on/off).
    ctl = ThrottleController(ThrottleConfig())
    for k in range(50):
        out = ctl.step(k * 0.05, 1.0, 0.3 * math.sin(k * 0.1))
        assert out == 2.0

    # A wider band: output is 0, or clamped into [run, max], never strictly between.
    cfg = ThrottleConfig(volts_per_mps=0.2, kp_v_per_mps=0.05, ki_v_per_mps_s=0.0, pi_trim_max_v=0.05,
                         throttle_run_v=1.0, throttle_v_max=2.0)
    ctl2 = ThrottleController(cfg)
    for k in range(50):
        out = ctl2.step(k * 0.05, 1.0, 0.0)
        assert out == 0.0 or cfg.throttle_run_v <= out <= cfg.throttle_v_max


def test_throttle_controller_integral_does_not_wind_up_while_saturated():
    ctl = ThrottleController(ThrottleConfig(volts_per_mps=1.0, ki_v_per_mps_s=0.5))
    for k in range(2000):
        ctl.step(k * 0.02, 1.0, 0.0)  # measured never catches up -> err stays positive forever
    assert abs(ctl.integral) < 10.0  # would be ~20 after 2000 steps of dt=0.02 if it wound up unchecked


def test_brake_for_candidate():
    assert brake_pct_for_candidate(True, 1.0) == 0.0
    assert brake_pct_for_candidate(True, 0.0) == 100.0
    assert brake_pct_for_candidate(False, 1.0) == 100.0
    assert brake_pct_for_candidate(True, float("nan")) == 100.0


def test_gate_caps_throttle_at_absolute_ceiling_and_requires_both_armed_and_fresh():
    gate = CommandGate(throttle_v_max=5.0)  # even a bad config cannot exceed 2 V
    out = gate.compute(1.0, True, True, 9.0, 0.0, 0.99)
    assert out.throttle_mv == THROTTLE_MAX_MV and out.reason == "pass"

    assert gate.compute(1.0, False, True, 1.0, 0.0, 0.99).throttle_mv == 0
    assert gate.compute(1.0, True, False, 1.0, 0.0, 0.99).throttle_mv == 0
    stale = gate.compute(2.0, True, True, 1.0, 0.0, 0.99)
    assert stale.throttle_mv == 0 and stale.brake_pct == 100
    assert gate.compute(1.0, True, True, float("nan"), 0.0, 0.99).throttle_mv == 0
    braking = gate.compute(1.0, True, True, 1.5, 100.0, 0.99)
    assert braking.throttle_mv == 0 and braking.brake_pct == 100


def test_angle_wrap_and_conversion():
    assert wrap_raw_delta(10, 4090) == 16
    assert wrap_raw_delta(4090, 10) == -16
    assert raw_to_rad(-1, 0, 0.1) is None
    assert abs(raw_to_rad(2148, 2048, 0.0879) - math.radians(8.79)) < 1e-9
