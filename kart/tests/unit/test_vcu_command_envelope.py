from dataclasses import replace
from math import nan

from kart_sim.vcu import MotionCommand, SafetyEnvelope, VCU, VcuInputs, VcuState


def ready_vcu():
    vcu = VCU(); vcu.state = VcuState.AUTONOMOUS; vcu.independent.pulse(0)
    return vcu


def inputs(t, command):
    return VcuInputs(t, command, 1, 0, 0, 0, 0, True, (1, 1, 1, 1), True, 0)


def test_crc_authorization_future_numeric_duplicate_and_stale_reject_without_drive():
    cases = [
        (MotionCommand("jetson", 1, 0, 1, 2, 0, checksum=123), "COMM-CRC"),
        (MotionCommand("attacker", 1, 0, 1, 2, 0).signed(), "COMM-AUTH"),
        (MotionCommand("jetson", 1, .1, 1, 2, 0).signed(), "COMM-FUTURE"),
        (MotionCommand("jetson", 1, 0, 1, nan, 0), "COMM-NUMERIC"),
    ]
    for command, code in cases:
        vcu = ready_vcu()
        output = vcu.cycle(inputs(0, command))
        assert output.command.drive == 0 and output.state == VcuState.CONTROLLED_STOP
        assert code in output.active_faults
    vcu = ready_vcu(); command = MotionCommand("jetson", 1, 0, 1, 2, 0).signed()
    assert vcu.cycle(inputs(0, command)).command.drive > 0
    duplicate = vcu.cycle(inputs(.01, command))
    assert duplicate.command.drive == 0 and "COMM-SEQUENCE" in duplicate.active_faults
    stale_vcu = ready_vcu()
    stale_vcu.independent.pulse(.2)
    stale = stale_vcu.cycle(inputs(.2, MotionCommand("jetson", 1, 0, .1, 2, 0).signed()))
    assert stale.command.drive == 0 and "COMM-STALE" in stale.active_faults


def test_sequence_wrap_is_accepted_but_reverse_and_large_gap_are_rejected():
    wrapped = ready_vcu()
    wrapped.last_sequence = 255
    wrap_command = MotionCommand("jetson", 0, 0, 1, 2, 0).signed()
    assert wrapped.cycle(inputs(0, wrap_command)).command.drive > 0
    reverse = ready_vcu()
    reverse.last_sequence = 10
    output = reverse.cycle(inputs(0, MotionCommand("jetson", 9, 0, 1, 2, 0).signed()))
    assert output.command.drive == 0 and "COMM-SEQUENCE" in output.active_faults
    gap = ready_vcu()
    gap.last_sequence = 10
    output = gap.cycle(inputs(0, MotionCommand("jetson", 15, 0, 1, 2, 0).signed()))
    assert output.command.drive == 0 and "COMM-SEQUENCE" in output.active_faults


def test_safety_limit_drops_immediately_recovers_slowly_and_steering_rate_is_limited():
    vcu = ready_vcu()
    command = MotionCommand("jetson", 1, 0, 1, 8, .5).signed()
    first = vcu.cycle(inputs(0, command), SafetyEnvelope(clear_distance_m=100))
    low_command = replace(command, sequence=2, timestamp_s=.01, valid_until_s=1).signed()
    low = vcu.cycle(inputs(.01, low_command), SafetyEnvelope(clear_distance_m=1))
    assert low.active_speed_limit_mps < first.active_speed_limit_mps
    recovery_command = replace(command, sequence=3, timestamp_s=.02, valid_until_s=1).signed()
    recovery = vcu.cycle(inputs(.02, recovery_command), SafetyEnvelope(clear_distance_m=100))
    assert recovery.active_speed_limit_mps <= low.active_speed_limit_mps + .0051
    assert abs(first.command.steering_rad) <= .6
    assert abs(recovery.command.steering_rad - low.command.steering_rad) <= .0201


def test_safety_envelope_responds_to_curvature_latency_uncertainty_and_brake_capability():
    nominal = SafetyEnvelope(clear_distance_m=10, delay_s=.1, guaranteed_decel_mps2=3, uncertainty_m=.5)
    assert nominal.speed_limit(curvature=.2) < nominal.speed_limit(curvature=0)
    assert SafetyEnvelope(clear_distance_m=10, delay_s=.5).speed_limit() < nominal.speed_limit()
    assert SafetyEnvelope(clear_distance_m=10, uncertainty_m=5).speed_limit() < nominal.speed_limit()
    assert SafetyEnvelope(clear_distance_m=10, guaranteed_decel_mps2=1).speed_limit() < nominal.speed_limit()
    assert SafetyEnvelope(clear_distance_m=1, uncertainty_m=.5, margin_m=.75).speed_limit() == 0
