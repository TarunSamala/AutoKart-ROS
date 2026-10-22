from dataclasses import replace

from kart_sim.vcu import AutonomousGuards, MotionCommand, VCU, VcuInputs, VcuState


def ready_vcu():
    vcu = VCU()
    for state in (VcuState.BOOT, VcuState.SELF_TEST, VcuState.SAFE, VcuState.READY):
        assert vcu.transition(state)
    return vcu


def test_autonomous_entry_requires_every_explicit_guard_and_direct_transition_is_rejected():
    fields = tuple(AutonomousGuards().__dict__)
    for field in fields:
        vcu = ready_vcu()
        assert not vcu.request_autonomous(replace(AutonomousGuards(), **{field: False}))
        assert vcu.state == VcuState.READY
        assert any(field.upper() in fault for fault in vcu.faults)
    vcu = ready_vcu()
    assert not vcu.transition(VcuState.AUTONOMOUS)
    assert vcu.request_autonomous(AutonomousGuards())


def test_service_and_degraded_states_do_not_create_propulsion_authority():
    service = VCU(); service.state = VcuState.SAFE
    assert service.transition(VcuState.SERVICE_MODE)
    assert not service.request_autonomous(AutonomousGuards())
    service.independent.pulse(0)
    service.cycle(VcuInputs(0, None, 0, 0, 2e5, 0))
    service.independent.pulse(.6)
    service.cycle(VcuInputs(.6, None, 0, 0, 2e5, 0))
    assert service.state == VcuState.SAFE and "SERVICE-TIMEOUT" in service.faults
    degraded = VCU(); degraded.state = VcuState.AUTONOMOUS
    assert degraded.transition(VcuState.DEGRADED)
    degraded.independent.pulse(0)
    output = degraded.cycle(VcuInputs(0, MotionCommand("jetson", 1, 0, .1, 8, 0).signed(), 0, 0, 0, 0))
    assert degraded.state == VcuState.DEGRADED and 0 < output.command.drive <= .25
