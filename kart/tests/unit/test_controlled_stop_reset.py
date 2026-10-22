from dataclasses import replace

from kart_sim.vcu import (
    ControlledStopManager,
    ControlledStopPhase,
    ResetInputs,
    ResetRequest,
    VCU,
    VcuInputs,
    VcuState,
)


def stop_input(t: float, *, speed: float = 0.0, wheels=(0.0, 0.0, 0.0, 0.0), torque=0.0,
               current=0.0, pressure=200_000.0, wheel_valid=True, feedback_time=None) -> VcuInputs:
    return VcuInputs(
        t, None, speed, 0.0, pressure, torque, 0.01, True, wheels, wheel_valid,
        current, True, True, True, False, t if feedback_time is None else feedback_time,
    )


def valid_reset_inputs() -> ResetInputs:
    return ResetInputs(
        timestamp_s=1.0,
        vehicle_speed_mps=0.0,
        wheel_speeds_mps=(0.0, 0.0, 0.0, 0.0),
        wheel_feedback_valid=True,
        motor_torque_nm=0.0,
        motor_current_a=0.0,
        motor_enable_removed=True,
        contactor_feedback_closed=False,
        contactor_open_commanded=True,
        physical_estop_active=False,
        remote_estop_active=False,
        independent_safety_healthy=True,
        watchdog_healthy=True,
        steering_primary_valid=True,
        steering_secondary_valid=True,
        steering_disagreement_rad=0.0,
        brake_pressure_pa=200_000.0,
        brake_pressure_valid=True,
        brake_actuator_valid=True,
        power_valid=True,
        thermal_valid=True,
        cooldown_complete=True,
        active_critical_faults=("MOTOR-003",),
    )


def test_stop_completion_requires_redundant_fresh_motion_torque_current_and_hold():
    manager = ControlledStopManager(confirmation_s=.05)
    assert manager.update(stop_input(0, speed=1, wheels=(0, 0, 0, 0))) != ControlledStopPhase.COMPLETE
    assert manager.update(stop_input(.1, speed=0, wheels=(.2, 0, 0, 0))) != ControlledStopPhase.COMPLETE
    assert manager.update(stop_input(.2, torque=.2)) != ControlledStopPhase.COMPLETE
    assert manager.update(stop_input(.3, current=2)) != ControlledStopPhase.COMPLETE
    assert manager.update(stop_input(.4, pressure=0)) != ControlledStopPhase.COMPLETE
    assert manager.update(stop_input(.5, feedback_time=.3)) != ControlledStopPhase.COMPLETE
    assert manager.update(stop_input(.6)) in (ControlledStopPhase.FINAL_STOP, ControlledStopPhase.HOLD)
    assert manager.update(stop_input(.66)) == ControlledStopPhase.COMPLETE


def test_rolling_restart_revokes_hold_and_stop_timeout_fails():
    manager = ControlledStopManager(confirmation_s=.1)
    manager.update(stop_input(0))
    assert manager.update(stop_input(.02)) == ControlledStopPhase.HOLD
    assert manager.update(stop_input(.04, speed=.2, wheels=(.2, .2, .2, .2))) == ControlledStopPhase.DECELERATE
    assert manager.rolling_restart_count == 1
    timeout = ControlledStopManager(timeout_s=.1)
    timeout.update(stop_input(0, speed=2, wheels=(2, 2, 2, 2)))
    assert timeout.update(stop_input(.11, speed=1, wheels=(1, 1, 1, 1))) == ControlledStopPhase.FAILED


def test_reset_guard_matrix_reports_each_unsafe_reason():
    vcu = VCU(); vcu.state = VcuState.FAULT_LOCKOUT
    request = ResetRequest("operator", 1.0, 1.1, True, True)
    base = valid_reset_inputs()
    cases = (
        (replace(base, vehicle_speed_mps=1), "vehicle_moving"),
        (replace(base, wheel_speeds_mps=(.2, 0, 0, 0)), "wheel_motion_or_invalid"),
        (replace(base, motor_torque_nm=.2), "motor_torque_present"),
        (replace(base, motor_current_a=2), "motor_current_present"),
        (replace(base, motor_enable_removed=False), "motor_enable_not_removed"),
        (replace(base, contactor_feedback_closed=True), "contactor_feedback_inconsistent"),
        (replace(base, physical_estop_active=True), "physical_estop_active"),
        (replace(base, remote_estop_active=True), "remote_estop_active"),
        (replace(base, independent_safety_healthy=False), "independent_safety_unhealthy"),
        (replace(base, watchdog_healthy=False), "watchdog_unhealthy"),
        (replace(base, steering_primary_valid=False), "steering_feedback_invalid"),
        (replace(base, steering_disagreement_rad=.1), "steering_disagreement"),
        (replace(base, brake_pressure_valid=False), "brake_feedback_invalid"),
        (replace(base, brake_pressure_pa=0), "brake_hold_missing"),
        (replace(base, power_valid=False), "power_invalid"),
        (replace(base, thermal_valid=False), "thermal_invalid"),
        (replace(base, cooldown_complete=False), "cooldown_incomplete"),
    )
    for inputs, reason in cases:
        decision = vcu.evaluate_reset_request(request, inputs)
        assert not decision.accepted and reason in decision.reasons
    assert "unauthorized" in vcu.evaluate_reset_request(replace(request, source="jetson"), base).reasons
    assert "reset_request_stale_or_future" in vcu.evaluate_reset_request(replace(request, valid_until_s=.9), base).reasons
    assert "repair_not_acknowledged" in vcu.evaluate_reset_request(replace(request, repair_acknowledged=False), base).reasons


def test_valid_reset_requires_self_test_safe_ready_and_never_reuses_old_command():
    vcu = VCU(); vcu.state = VcuState.EMERGENCY_STOP; vcu.last_sequence = 99; vcu._fault("MOTOR-003", .5)
    decision = vcu.reset_with_inputs(ResetRequest("operator", 1, 1.1, True), valid_reset_inputs())
    assert decision.accepted and vcu.state == VcuState.SELF_TEST and vcu.last_sequence is None
    assert "MOTOR-003" in vcu.fault_history and "MOTOR-003" in vcu.fault_manager.history
    assert vcu.transition(VcuState.SAFE, healthy=True)
    assert vcu.transition(VcuState.READY)
    assert vcu.state == VcuState.READY
    assert not vcu.transition(VcuState.AUTONOMOUS, healthy=False)
    assert vcu.state == VcuState.READY
    failed = VCU(); failed.state = VcuState.FAULT_LOCKOUT
    assert failed.reset_with_inputs(ResetRequest("operator", 1, 1.1, True), valid_reset_inputs()).accepted
    assert not failed.transition(VcuState.SAFE, healthy=False)
    assert failed.state == VcuState.FAULT_LOCKOUT


def test_independent_safety_latch_has_separate_guarded_reset():
    vcu = VCU()
    vcu.independent.pulse(0.0)
    assert not vcu.independent.permit(.06)
    assert vcu.independent.stop_latched
    assert not vcu.independent.reset(authorized=True, source_resolved=True, vehicle_stopped=False)
    assert not vcu.independent.reset(authorized=False, source_resolved=True, vehicle_stopped=True)
    assert vcu.independent.reset(authorized=True, source_resolved=True, vehicle_stopped=True)
    assert not vcu.independent.stop_latched
