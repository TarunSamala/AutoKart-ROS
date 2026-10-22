from dataclasses import replace

from kart_sim.communication import crc16_ccitt
from kart_sim.safety_diagnostics import DTC_REGISTRY
from kart_sim.safety_health import (
    HealthQuality,
    LocalizationHealth,
    PerceptionHealth,
    SensorFunctionHealthManager,
    SignalHealth,
)
from kart_sim.vcu import MotionCommand, VCU, VcuInputs, VcuState


def test_crc16_ccitt_vector_and_motion_command_field_protection():
    assert crc16_ccitt(b"123456789") == 0x29B1
    command = MotionCommand("jetson", 7, 1.0, 1.1, 3.0, 0.2, 0.1).signed()
    assert command.checksum == command.crc()
    assert replace(command, speed_mps=3.1).crc() != command.checksum


def test_function_health_uses_age_quality_and_redundancy():
    manager = SensorFunctionHealthManager()
    wheel = SignalHealth("wheel", 1.0)
    motor = SignalHealth("motor", 1.0)
    assert manager.evaluate("speed", (wheel, motor), now_s=1.05, maximum_age_s=.1).quality == HealthQuality.VALID
    degraded = manager.evaluate(
        "speed",
        (wheel, replace(motor, valid=False)),
        now_s=1.05,
        maximum_age_s=.1,
        required_sources=1,
    )
    assert degraded.quality == HealthQuality.DEGRADED
    assert not manager.evaluate("speed", (wheel,), now_s=1.2, maximum_age_s=.1).valid
    assert LocalizationHealth(1.0, .1, .01, True).valid(1.05)
    assert not LocalizationHealth(1.0, 2.0, .01, True).valid(1.05)
    assert PerceptionHealth(1.0, 10.0, 5.0, .8).valid(1.05)


def test_critical_fault_captures_observable_snapshot_and_dtc_metadata():
    assert DTC_REGISTRY["MOTOR-003"].response == "EMERGENCY_STOP"
    vcu = VCU()
    for state in (VcuState.BOOT, VcuState.SELF_TEST, VcuState.SAFE, VcuState.READY):
        assert vcu.transition(state)
    from kart_sim.vcu import AutonomousGuards
    assert vcu.request_autonomous(AutonomousGuards())
    command = MotionCommand("scenario", 1, 0.0, 1.0, 0.0, 0.0).signed()
    vcu.independent.pulse(0.0)
    for timestamp in (0.0, .06):
        vcu.independent.pulse(timestamp)
        vcu.cycle(VcuInputs(timestamp, command, 2.0, 0.0, 0.0, 5.0, motor_current_a=10.0))
    snapshots = vcu.fault_snapshots.records()
    assert snapshots and "MOTOR-003" in snapshots[-1].fault_codes
    assert snapshots[-1].motor_torque_nm == 5.0
