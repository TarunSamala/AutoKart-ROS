from autokart_safety.core import (Config, Inputs, SafetyLogic, TelView, evaluate_startup, READY, DEGRADED, UNAVAILABLE)


def good(**kw):
    base = dict(ages={k: 0.05 for k in ("telemetry", "image", "detections", "map", "odom", "planner", "controller")},
                tel=TelView(0, 0, [], True, True, True, True, True, "0.1.0", 0.0),
                detections_depth_valid=True, detections_inference_ms=20.0, calibration_verified=True,
                vehicle_config_verified=True, braking_verified=True, estop_operator_confirmed=True,
                controller_state="IDLE", controller_stopped=False)
    base.update(kw)
    return Inputs(**base)


def test_nothing_connected_is_unavailable_never_ready():
    stages, ready, reasons = evaluate_startup(Inputs(ages={}), Config())
    assert not ready and stages[1]["state"] == UNAVAILABLE and stages[6]["state"] == UNAVAILABLE
    assert stages[13]["state"] != READY


def test_all_good_is_ready():
    stages, ready, reasons = evaluate_startup(good(), Config())
    assert ready, reasons
    assert stages[13]["state"] == READY and stages[5]["state"] == UNAVAILABLE  # IMU optional


def test_blockers():
    for kw, text in [(dict(calibration_verified=False), "stage 10"), (dict(braking_verified=False), "braking"),
                     (dict(vehicle_config_verified=False), "vehicle config"),
                     (dict(estop_operator_confirmed=False), "stage 13")]:
        _, ready, reasons = evaluate_startup(good(**kw), Config())
        assert not ready and any(text in r for r in reasons), (kw, reasons)
    inp = good(); inp.tel.axle_motion_seen = False
    _, ready, reasons = evaluate_startup(inp, Config())
    assert not ready and any("stage 04" in r for r in reasons)
    inp = good(); inp.ages["image"] = 2.0
    _, ready, _ = evaluate_startup(inp, Config())
    assert not ready


def arm_ready(L, now=0.0):
    out = L.step(now, good(), ["arm"])
    assert out.state == "ARMING" and out.vcu_request == "ARM", out.events
    inp = good(); inp.tel.state = 1
    out = L.step(now + 0.1, inp, [])
    assert out.state == "ARMED"
    return inp


def test_arm_flow():
    L = SafetyLogic()
    assert L.step(0.0, good(), []).arm_allowed
    arm_ready(L)


def test_arm_refused_when_not_ready():
    L = SafetyLogic()
    out = L.step(0.0, good(braking_verified=False), ["arm"])
    assert out.state == "DISARMED" and any("refused" in e for e in out.events)


def test_arming_timeout():
    L = SafetyLogic()
    L.step(0.0, good(), ["arm"])
    out = L.step(5.0, good(), [])
    assert out.state == "DISARMED" and out.vcu_request == "DISARM"


def test_faults_while_armed():
    cases = {
        "telemetry": lambda i: i.ages.update(telemetry=1.0),
        "image": lambda i: i.ages.update(image=1.0),
        "detections": lambda i: i.ages.update(detections=1.0),
        "odom": lambda i: i.ages.update(odom=1.0),
        "controller": lambda i: i.ages.update(controller=1.0),
        "vcu_fault": lambda i: (setattr(i.tel, "state", 2), setattr(i.tel, "fault_names", ["LINK_TIMEOUT"])),
        "estop": lambda i: setattr(i.tel, "estop_ok", False),
        "overspeed": lambda i: setattr(i.tel, "speed_mps", 9.0),
    }
    for name, mut in cases.items():
        L = SafetyLogic(); arm_ready(L)
        inp = good(); inp.tel.state = 1
        mut(inp)
        out = L.step(1.0, inp, [])
        assert out.state == "FAULT" and out.vcu_request == "DISARM", name


def test_blind_camera_ok_after_stopped():
    L = SafetyLogic(); arm_ready(L)
    inp = good(controller_state="STOPPED", controller_stopped=True); inp.tel.state = 1
    inp.ages.update(image=5.0, detections=5.0, map=5.0, planner=5.0)
    assert L.step(1.0, inp, []).state == "ARMED"


def test_stop_ineffective():
    L = SafetyLogic(); arm_ready(L)
    inp = good(controller_state="STOPPING"); inp.tel.state = 1; inp.tel.speed_mps = 1.0
    assert L.step(1.0, inp, []).state == "ARMED"
    assert L.step(6.0, inp, []).state == "FAULT"


def test_stop_request_latches_until_reset():
    L = SafetyLogic(); arm_ready(L)
    out = L.step(1.0, good(), ["stop"])
    assert out.state == "DISARMED" and out.stop_latched and out.vcu_request == "DISARM" and not out.arm_allowed
    out = L.step(1.1, good(), ["arm"])
    assert out.state == "DISARMED" and any("refused" in e for e in out.events)
    out = L.step(1.2, good(), ["reset_fault"])
    assert not out.stop_latched and out.arm_allowed


def test_reset_refused_when_cause_present():
    L = SafetyLogic(); arm_ready(L)
    inp = good(); inp.tel.state = 1; inp.tel.estop_ok = False
    assert L.step(1.0, inp, []).state == "FAULT"
    out = L.step(1.1, inp, ["reset_fault"])
    assert out.state == "FAULT"
    ok = good(); ok.tel.state = 2
    out = L.step(1.2, ok, ["reset_fault"])
    assert out.state == "DISARMED" and out.vcu_request == "RESET"
