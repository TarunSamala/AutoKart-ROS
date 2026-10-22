from autokart_vcu_safety.core import Config, Inputs, SafetyLogic, VcuView, evaluate_startup


def good(**overrides) -> Inputs:
    base = dict(
        ages={"vcu": 0.0, "odom": 0.0, "perception": 0.0, "mapping": 0.0, "planner": 0.0, "controller": 0.0},
        vcu=VcuView(state="SAFE", estop_active=False, estop_input_fitted=True),
        speed_mps=0.0, speed_valid=True, planner_decision="CRUISE",
        calibration_verified=True, vehicle_config_verified=True, braking_verified=True,
        manual_power_cut_confirmed=True,
    )
    base.update(overrides)
    return Inputs(**base)


def arm_ready(logic: SafetyLogic) -> None:
    out = logic.step(0.0, good(), ["confirm_manual_power_cut", "arm"])
    assert out.state == "ARMING", out.events
    out = logic.step(0.1, good(vcu=VcuView(state="ARMED", estop_active=False)), [])
    assert out.state == "ARMED", out.events


def test_startup_ready_only_when_everything_fresh_and_verified():
    stages, ready, reasons = evaluate_startup(good(), Config())
    assert ready and not reasons
    assert all(s["state"] == "READY" for s in stages[:-1])


def test_each_unverified_flag_blocks_readiness():
    expect_substring = {
        "calibration_verified": "calibration_verified",
        "vehicle_config_verified": "vehicle_config_verified",
        "braking_verified": "braking_verified",
        "manual_power_cut_confirmed": "operator has not confirmed",
    }
    for flag, substring in expect_substring.items():
        _, ready, reasons = evaluate_startup(good(**{flag: False}), Config())
        assert not ready
        assert any(substring in r for r in reasons), (flag, reasons)


def test_stale_input_blocks_readiness():
    for key in ("vcu", "odom", "perception", "mapping", "planner", "controller"):
        inp = good()
        inp.ages[key] = 5.0
        _, ready, reasons = evaluate_startup(inp, Config())
        assert not ready, key


def test_arm_refused_when_not_ready():
    logic = SafetyLogic()
    out = logic.step(0.0, good(braking_verified=False), ["arm"])
    assert out.state == "DISARMED" and any("refused" in e for e in out.events)


def test_arm_refused_unless_vcu_reports_safe():
    logic = SafetyLogic()
    out = logic.step(0.0, good(vcu=VcuView(state="UNKNOWN", estop_active=True)), ["arm"])
    assert out.state == "DISARMED" and any("refused" in e for e in out.events)


def test_full_arm_sequence_and_vcu_request_forwarded():
    logic = SafetyLogic()
    out = logic.step(0.0, good(), ["confirm_manual_power_cut", "arm"])
    assert out.state == "ARMING" and out.vcu_request == "ARM"
    out = logic.step(0.1, good(vcu=VcuView(state="ARMED", estop_active=False)), [])
    assert out.state == "ARMED" and out.arm_allowed is False  # already armed, not "allowed to arm again"


def test_arming_timeout_disarms():
    logic = SafetyLogic()
    logic.step(0.0, good(), ["confirm_manual_power_cut", "arm"])
    out = logic.step(5.0, good(), [])  # VCU never actually reports ARMED
    assert out.state == "DISARMED" and out.vcu_request == "DIS"


def test_faults_while_armed_disarm_the_vcu():
    cases = {
        "vcu_stale": lambda i: i.ages.update(vcu=1.0),
        "odom_stale": lambda i: i.ages.update(odom=1.0),
        "perception_stale": lambda i: i.ages.update(perception=1.0),
        "estop": lambda i: setattr(i, "vcu", VcuView(state="ARMED", estop_active=True, estop_input_fitted=True)),
        "vcu_fault": lambda i: setattr(i, "vcu", VcuView(state="FAULT", estop_active=False,
                                                         active_faults=("LINK_TIMEOUT",))),
        "overspeed": lambda i: setattr(i, "speed_mps", 9.0),
    }
    for name, mutate in cases.items():
        logic = SafetyLogic()
        arm_ready(logic)
        inp = good(vcu=VcuView(state="ARMED", estop_active=False))
        mutate(inp)
        out = logic.step(1.0, inp, [])
        assert out.state == "FAULT" and out.vcu_request == "DIS", name


def test_stop_not_effective_faults_after_timeout():
    logic = SafetyLogic(Config(stop_ineffective_s=1.0, stop_speed_mps=0.15))
    arm_ready(logic)
    inp = good(vcu=VcuView(state="ARMED", estop_active=False), planner_decision="STOP", speed_mps=1.0)
    first = logic.step(1.0, inp, [])
    assert first.state == "DISARMED" and first.stop_latched and first.vcu_request == "DIS"
    out = logic.step(1.5, inp, [])
    assert out.state == "DISARMED", "stop must remain latched before the ineffective timeout"
    out = logic.step(2.5, inp, [])
    assert out.state == "FAULT" and "stop not effective" in out.reasons[0] or any(
        "stop not effective" in r for r in out.reasons)


def test_stop_request_latches_until_explicit_reset():
    logic = SafetyLogic()
    arm_ready(logic)
    out = logic.step(1.0, good(vcu=VcuView(state="ARMED", estop_active=False)), ["stop"])
    assert out.state == "DISARMED" and out.stop_latched and out.vcu_request == "DIS"
    out = logic.step(1.1, good(vcu=VcuView(state="SAFE", estop_active=False)), ["arm"])
    assert out.state == "DISARMED" and any("reset first" in e for e in out.events)
    out = logic.step(1.2, good(vcu=VcuView(state="SAFE", estop_active=False)), ["reset_fault"])
    assert out.state == "DISARMED" and not out.stop_latched


def test_reset_refused_while_estop_active():
    logic = SafetyLogic()
    arm_ready(logic)
    logic.step(1.0, good(vcu=VcuView(state="ARMED", estop_active=True, estop_input_fitted=True)), [])  # faults from estop
    out = logic.step(1.1, good(vcu=VcuView(state="FAULT", estop_active=True, estop_input_fitted=True)), ["reset_fault"])
    assert out.state == "FAULT" and any("refused" in e for e in out.events)


def test_not_fitted_amber_and_manual_confirmation_lifecycle():
    logic = SafetyLogic()
    absent = VcuView(state="SAFE", estop_active=True, estop_input_fitted=False)
    inp = good(vcu=absent, manual_power_cut_confirmed=False)
    stages, ready, _ = evaluate_startup(inp, Config())
    assert stages[7]["state"] == "NOT_FITTED" and "MANUAL POWER CUT ONLY" in stages[7]["detail"]
    assert not ready
    out = logic.step(0.0, inp, ["arm"])
    assert out.state == "DISARMED" and not out.arm_allowed
    out = logic.step(0.1, inp, ["confirm_manual_power_cut", "arm"])
    assert out.state == "ARMING" and out.vcu_request == "ARM"
    assert out.stages[7]["state"] == "NOT_FITTED"
    armed = VcuView(state="ARMED", estop_active=True, estop_input_fitted=False)
    out = logic.step(0.2, good(vcu=armed), [])
    assert out.state == "ARMED"  # an unfitted pin never trips a fault
    out = logic.step(0.3, good(vcu=armed), ["stop"])
    assert out.state == "DISARMED" and not out.manual_power_cut_confirmed and out.stop_latched
    logic.step(0.4, good(vcu=absent), ["reset_fault"])
    out = logic.step(0.5, good(vcu=absent), ["arm"])
    assert out.state == "DISARMED" and not out.manual_power_cut_confirmed


def test_confirmation_clears_on_link_loss_and_fault():
    logic = SafetyLogic()
    logic.step(0.0, good(), ["confirm_manual_power_cut"])
    lost = good()
    lost.ages["vcu"] = 1.0
    assert not logic.step(0.1, lost, []).manual_power_cut_confirmed
    logic.step(0.2, good(), ["confirm_manual_power_cut"])
    faulted = good(vcu=VcuView(state="FAULT", estop_input_fitted=True))
    assert not logic.step(0.3, faulted, []).manual_power_cut_confirmed
    logic.step(0.4, good(), ["confirm_manual_power_cut"])
    stale_map = good()
    stale_map.ages["mapping"] = 1.0
    assert not logic.step(0.5, stale_map, []).manual_power_cut_confirmed


def test_demo_drive_refused_unless_already_armed():
    logic = SafetyLogic()
    out = logic.step(0.0, good(), ["demo_drive_start"])
    assert out.state == "DISARMED" and not out.demo_drive_active
    assert any("must be ARMED" in e for e in out.events)


def test_demo_drive_granted_while_armed_and_cleared_by_stop():
    logic = SafetyLogic()
    arm_ready(logic)
    out = logic.step(0.2, good(vcu=VcuView(state="ARMED", estop_active=False)), ["demo_drive_start"])
    assert out.demo_drive_active
    out = logic.step(0.3, good(vcu=VcuView(state="ARMED", estop_active=False)), ["stop"])
    assert not out.demo_drive_active and out.state == "DISARMED"


def test_demo_drive_cleared_by_runtime_fault_not_just_explicit_stop():
    logic = SafetyLogic()
    arm_ready(logic)
    out = logic.step(0.2, good(vcu=VcuView(state="ARMED", estop_active=False)), ["demo_drive_start"])
    assert out.demo_drive_active
    stale = good(vcu=VcuView(state="ARMED", estop_active=False))
    stale.ages["vcu"] = 5.0
    out = logic.step(0.3, stale, [])
    assert out.state == "FAULT" and not out.demo_drive_active


def test_demo_drive_explicit_stop_works_even_if_never_active():
    logic = SafetyLogic()
    out = logic.step(0.0, good(), ["demo_drive_stop"])
    assert not out.demo_drive_active
