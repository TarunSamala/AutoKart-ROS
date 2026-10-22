import math, subprocess, os
from autokart_vehicle_bridge.protocol import (build_frame, parse_frame, crc16_ccitt, build_cmd, build_simple,
                                              parse_message, THROTTLE_MAX_MV)
from autokart_vehicle_bridge.core import CommandGate, Odometry, raw_to_deg, wrap_raw_delta

PI_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FW_ROOT = os.path.join(os.path.dirname(PI_ROOT), "esp32_firmware")


def test_crc_check_value():
    assert crc16_ccitt(b"123456789") == 0x29B1


def test_frame_roundtrip_and_corruption():
    f = build_cmd(7, 500, -120, 0)
    assert parse_frame(f) == "CMD,7,500,-120,0"
    bad = bytearray(f); bad[4] ^= 1
    assert parse_frame(bytes(bad)) is None
    assert parse_frame("garbage") is None
    assert parse_frame("$X*ZZZZ") is None


def test_parse_tel():
    payload = "TEL,5,12345,1,0,17,1,0,1500,-300,820,2050,-1,42,15"
    kind, t = parse_message(payload)
    assert kind == "TEL" and t.state_name == "ARMED" and t.estop_ok and t.dac_ok and not t.steer_pwm_ok
    assert t.speed_mmps == 820 and t.steer_raw == 2050 and t.brake_raw == -1
    assert parse_message("TEL,1,2") is None
    assert parse_message("HELLO,0.1.0,1") == ("HELLO", ("0.1.0", 1))


def test_crc_matches_firmware_cpp():
    exe = "/tmp/vcu_test"
    if not os.path.exists(exe):
        subprocess.check_call(["g++", "-std=c++11", "-I", f"{FW_ROOT}/include", "-o", exe,
                               f"{FW_ROOT}/test/test_vcu_logic.cpp", f"{FW_ROOT}/src/protocol.cpp",
                               f"{FW_ROOT}/src/vcu_logic.cpp"])
    msgs = ["CMD,1,0,0,0", "TEL,1,2,3,4,5,6,7,8,9,10,11,12,13,14", "ARM,99", "HELLO,0.1.0,1"]
    out = subprocess.check_output([exe, "crc"] + msgs, text=True).split()
    assert out == [f"{crc16_ccitt(m.encode()):04X}" for m in msgs]


def test_gate_caps_throttle_at_2v():
    g = CommandGate(throttle_v_max=5.0)           # even a bad config cannot exceed 2 V
    o = g.compute(1.0, True, True, (9.0, 0.0, 0.0), 0.99)
    assert o.throttle_mv == THROTTLE_MAX_MV and o.reason == "pass"
    assert g.compute(1.0, True, True, (-1.0, 0.0, 0.0), 0.99).throttle_mv == 0


def test_gate_blocks():
    g = CommandGate()
    cmd = (1.0, 5.0, 0.0)
    assert g.compute(1.0, False, True, cmd, 0.99).throttle_mv == 0
    assert g.compute(1.0, True, False, cmd, 0.99).throttle_mv == 0
    stale = g.compute(2.0, True, True, cmd, 0.99)
    assert stale.throttle_mv == 0 and stale.brake_pct == 100
    assert g.compute(1.0, True, True, (float("nan"), 0, 0), 0.99).throttle_mv == 0
    b = g.compute(1.0, True, True, (1.5, 0.0, 100.0), 0.99)
    assert b.throttle_mv == 0 and b.brake_pct == 100


def test_angles():
    assert wrap_raw_delta(10, 4090) == 16
    assert wrap_raw_delta(4090, 10) == -16
    assert raw_to_deg(-1, 0, 0.1) is None
    assert abs(raw_to_deg(2148, 2048, 0.0879) - 8.79) < 1e-6


def test_odometry_straight_and_turn():
    o = Odometry(1.0)
    for _ in range(100):
        o.update(0.01, 1.0, 0.0)
    assert abs(o.x - 1.0) < 1e-6 and abs(o.y) < 1e-9
    o2 = Odometry(1.0)
    for _ in range(1000):
        o2.update(0.01, 1.0, 10.0)
    assert o2.yaw > 1.0 and o2.status == "ENCODER_STEER_MODEL"
    o3 = Odometry(1.0)
    o3.update(0.01, 1.0, None)
    assert o3.status == "ENCODER_ONLY_NO_YAW"
