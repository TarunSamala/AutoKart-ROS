import pytest

from autokart_vehicle_bridge.demo_drive import DemoDriveController, THROTTLE_MAX_MV


def test_boot_state_is_not_driving_and_outputs_safe():
    c = DemoDriveController()
    out = c.compute(0.0)
    assert out.throttle_mv == 0 and out.brake_pct == 100


def test_drive_refused_unless_active_ie_already_armed():
    c = DemoDriveController()
    ok, reason = c.request_drive(0.0, "forward", 2.0)
    assert not ok and reason == "DEMO_DRIVE_NOT_ACTIVE"


def test_normal_drive_cycle_outputs_capped_throttle_then_stops_at_duration():
    c = DemoDriveController()
    c.set_active(True)
    c.heartbeat(0.0)
    ok, reason = c.request_drive(0.0, "forward", 2.0)
    assert ok and reason == "OK"
    c.heartbeat(0.5)
    out = c.compute(0.5)
    assert out.throttle_mv == THROTTLE_MAX_MV and out.brake_pct == 0
    c.heartbeat(2.1)
    out = c.compute(2.1)
    assert out.throttle_mv == 0 and out.brake_pct == 100 and out.reason == "timed move complete"


def test_fault_injection_heartbeat_drop_forces_safe_state():
    c = DemoDriveController()
    c.set_active(True)
    c.heartbeat(0.0)
    c.request_drive(0.0, "forward", 10.0)
    assert c.compute(0.3).throttle_mv == THROTTLE_MAX_MV
    out = c.compute(0.51)  # > 500 ms since last heartbeat
    assert out.throttle_mv == 0 and out.brake_pct == 100 and out.reason == "browser heartbeat lost"


def test_fault_injection_losing_active_mid_drive_forces_safe_state_immediately():
    """e-stop / disarm / fault mid-ramp: autokart_vcu_safety clears demo_drive_active, which must
    stop output on the very next compute(), not wait for a timeout."""
    c = DemoDriveController()
    c.set_active(True)
    c.heartbeat(0.0)
    c.request_drive(0.0, "forward", 10.0)
    assert c.compute(0.1).throttle_mv == THROTTLE_MAX_MV
    c.set_active(False)
    out = c.compute(0.1)
    assert out.throttle_mv == 0 and out.brake_pct == 100


def test_stop_always_works_from_any_state():
    c = DemoDriveController()
    c.request_stop()  # never armed/active/driving; must not raise
    assert c.compute(0.0).throttle_mv == 0
    c.set_active(True)
    c.heartbeat(0.0)
    c.request_drive(0.0, "forward", 10.0)
    c.request_stop()
    assert c.compute(0.1).throttle_mv == 0


def test_over_cap_request_is_clamped_not_merely_ui_side():
    c = DemoDriveController(throttle_mv=5000)
    c.set_active(True)
    c.heartbeat(0.0)
    c.request_drive(0.0, "forward", 1.0)
    assert c.compute(0.1).throttle_mv == THROTTLE_MAX_MV


def test_reverse_is_refused_not_silently_a_noop():
    c = DemoDriveController()
    c.set_active(True)
    c.heartbeat(0.0)
    ok, reason = c.request_drive(0.0, "reverse", 1.0)
    assert not ok and reason == "REVERSE_NOT_SUPPORTED_BY_FIRMWARE"


def test_direction_change_while_moving_is_refused():
    c = DemoDriveController()
    c.set_active(True)
    c.heartbeat(0.0)
    c.request_drive(0.0, "forward", 10.0)
    c.compute(0.1)  # moving_direction becomes "forward"
    ok, reason = c.request_drive(0.1, "reverse", 1.0)
    assert not ok and reason == "REVERSE_NOT_SUPPORTED_BY_FIRMWARE"
    # Same direction while moving is allowed (re-arms the timer).
    ok, reason = c.request_drive(0.1, "forward", 1.0)
    assert ok and reason == "OK"


def test_duration_out_of_range_refused():
    c = DemoDriveController()
    c.set_active(True)
    c.heartbeat(0.0)
    ok, _ = c.request_drive(0.0, "forward", 0.0)
    assert not ok
    ok, _ = c.request_drive(0.0, "forward", 999.0)
    assert not ok


def test_set_active_false_mid_drive_clears_moving_direction_too():
    c = DemoDriveController()
    c.set_active(True)
    c.heartbeat(0.0)
    c.request_drive(0.0, "forward", 10.0)
    c.compute(0.1)
    c.set_active(False)
    assert c.moving_direction is None and not c.driving
