"""VCU arming safety supervisor -- pure logic, no ROS dependencies.

Distinct from autokart_safety_bridge's front_safety_envelope_monitor, which is a DIAGNOSTIC_ONLY obstacle-
distance classifier with vehicle_authority permanently OFF (see its node.py). This module is the actual arming
authority for autokart_vehicle_bridge: it is the only thing that can request ARM, and it requires every
upstream stage to be measurably fresh/valid, all three *_verified flags to be true, and an operator to have
confirmed the manual power cut is within reach, before it will ever ask the VCU to arm.

Startup stages are computed ONLY from real signal freshness/validity -- never "a process exists" or "page
loaded". Fail-safe: any missing input defaults to NOT ready / NOT armed.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

READY, INIT, DEGRADED, FAULT, UNAVAILABLE, NOT_FITTED = "READY", "INITIALIZING", "DEGRADED", "FAULT", "UNAVAILABLE", "NOT_FITTED"

STAGE_NAMES = [
    "Supervisor running", "VCU telemetry (vehicle_bridge -> /vcu/state)", "Axle/steering odometry",
    "Perception", "Mapping", "Planner", "Controller candidate", "E-stop input status",
    "Manual power cut confirmation", "Disarmed, mission-ready",
]


@dataclass
class VcuView:
    state: str = "UNKNOWN"          # "SAFE" | "ARMED" | "FAULT" | "UNKNOWN"
    estop_active: bool = True       # fail-safe default: assume tripped until proven otherwise
    estop_input_fitted: bool = False
    active_faults: Tuple[str, ...] = ()


@dataclass
class Inputs:
    ages: Dict[str, Optional[float]]          # seconds since last message per topic; None = never received
    vcu: Optional[VcuView] = None
    speed_mps: Optional[float] = None         # from /vehicle/actuator_feedback, axle-derived
    speed_valid: bool = False
    planner_decision: Optional[str] = None    # latest /autokart/planner/status payload "decision"
    calibration_verified: bool = False
    vehicle_config_verified: bool = False
    braking_verified: bool = False
    manual_power_cut_confirmed: bool = False


@dataclass
class Config:
    required_stages: Tuple[int, ...] = (1, 2, 3, 4, 5, 6, 7, 8)
    vcu_timeout_s: float = 0.3
    odom_timeout_s: float = 0.3
    perception_timeout_s: float = 0.5
    mapping_timeout_s: float = 0.5
    planner_timeout_s: float = 0.5
    controller_timeout_s: float = 0.5
    overspeed_mps: float = 2.5           # tune after measuring real top speed at the 2 V throttle cap
    stop_ineffective_s: float = 4.0
    stop_speed_mps: float = 0.15
    arming_timeout_s: float = 3.0


STOP_DECISIONS = {"STOP", "STOP_CANDIDATE", "BRAKE"}


def _age(inp: Inputs, key: str) -> Optional[float]:
    return inp.ages.get(key)


def _fresh(inp: Inputs, key: str, limit: float) -> bool:
    a = _age(inp, key)
    return a is not None and a <= limit


def evaluate_startup(inp: Inputs, cfg: Config) -> Tuple[List[dict], bool, List[str]]:
    """Return (stages, ready, reasons). Stage dicts: id, name, state, detail."""
    S: Dict[int, Tuple[str, str]] = {1: (READY, "supervisor node running")}
    vcu = inp.vcu
    vcu_fresh = _fresh(inp, "vcu", cfg.vcu_timeout_s)

    if not vcu_fresh or vcu is None:
        S[2] = (UNAVAILABLE, "no /vcu/state from the vehicle bridge")
    elif vcu.state == "UNKNOWN":
        S[2] = (FAULT, "bridge is running but reports no VCU telemetry")
    else:
        S[2] = (READY, f"vcu state={vcu.state}")

    odom_fresh = _fresh(inp, "odom", cfg.odom_timeout_s)
    S[3] = (READY, "odometry fresh") if odom_fresh else (
        UNAVAILABLE if _age(inp, "odom") is None else FAULT, "odometry stale or absent")

    perception_fresh = _fresh(inp, "perception", cfg.perception_timeout_s)
    S[4] = (READY, "perception status fresh") if perception_fresh else (
        UNAVAILABLE if _age(inp, "perception") is None else FAULT, "perception status stale or absent")

    mapping_fresh = _fresh(inp, "mapping", cfg.mapping_timeout_s)
    S[5] = (READY, "mapping status fresh") if mapping_fresh else (
        UNAVAILABLE if _age(inp, "mapping") is None else FAULT, "mapping status stale or absent")

    planner_fresh = _fresh(inp, "planner", cfg.planner_timeout_s)
    S[6] = (READY, "planner status fresh") if planner_fresh else (
        UNAVAILABLE if _age(inp, "planner") is None else FAULT, "planner status stale or absent")

    controller_fresh = _fresh(inp, "controller", cfg.controller_timeout_s)
    S[7] = (READY, "controller status fresh") if controller_fresh else (
        UNAVAILABLE if _age(inp, "controller") is None else FAULT, "controller status stale or absent")

    if not vcu_fresh or vcu is None:
        S[8] = (UNAVAILABLE, "no VCU telemetry")
    elif not vcu.estop_input_fitted:
        S[8] = (NOT_FITTED, "NOT FITTED - MANUAL POWER CUT ONLY")
    elif vcu.estop_active:
        S[8] = (FAULT, "E-stop circuit reports active/not-OK")
    else:
        S[8] = (READY, "E-stop circuit OK")

    S[9] = (READY, "physical power cut confirmed within reach for this ARM") if inp.manual_power_cut_confirmed else (
        DEGRADED, "operator has not confirmed a physical power cut within reach")

    reasons: List[str] = []
    for sid in cfg.required_stages:
        state, detail = S[sid]
        if state != READY and not (sid == 8 and state == NOT_FITTED):
            reasons.append(f"stage {sid:02d} {STAGE_NAMES[sid - 1]}: {state} ({detail})")
    if not inp.calibration_verified:
        reasons.append("calibration_verified is false")
    if not inp.vehicle_config_verified:
        reasons.append("vehicle_config_verified is false")
    if not inp.braking_verified:
        reasons.append("braking_verified is false")
    if not inp.manual_power_cut_confirmed:
        reasons.append("operator has not confirmed a physical power cut within reach")
    if vcu is not None and vcu.state == "FAULT":
        reasons.append("VCU latched FAULT: " + ",".join(vcu.active_faults or ["?"]))

    ready = not reasons
    S[10] = (READY, "awaiting authorization") if ready else (DEGRADED, f"{len(reasons)} blocking item(s)")
    stages = [{"id": i, "name": STAGE_NAMES[i - 1], "state": S[i][0], "detail": S[i][1]} for i in range(1, 11)]
    return stages, ready, reasons


@dataclass
class SafetyOutput:
    state: str
    arm_allowed: bool
    vcu_heartbeat_ok: bool
    startup_ready: bool
    stop_latched: bool
    manual_power_cut_confirmed: bool
    reasons: List[str]
    stages: List[dict]
    vcu_request: Optional[str] = None      # "ARM" | "DIS" | "RST"
    events: List[str] = field(default_factory=list)
    demo_drive_active: bool = False        # DEMO DRIVE, not autonomy; only ever true while ARMED


class SafetyLogic:
    def __init__(self, cfg: Optional[Config] = None):
        self.cfg = cfg or Config()
        self.state = "DISARMED"
        self.stop_latched = False
        self.arming_since: Optional[float] = None
        self.stopping_since: Optional[float] = None
        self.fault_reasons: List[str] = []
        self.manual_power_cut_confirmed = False
        self.demo_drive_active = False

    def _runtime_faults(self, inp: Inputs) -> List[str]:
        c, f = self.cfg, []
        vcu = inp.vcu
        if not _fresh(inp, "vcu", c.vcu_timeout_s):
            f.append("VCU heartbeat lost")
        elif vcu is not None:
            if vcu.state == "FAULT":
                f.append("VCU fault: " + ",".join(vcu.active_faults or ["?"]))
            elif vcu.state in ("SAFE", "UNKNOWN"):
                f.append("VCU disarmed/unknown unexpectedly")
            if vcu.estop_input_fitted and vcu.estop_active:
                f.append("E-stop circuit not OK")
        if inp.speed_valid and inp.speed_mps is not None and inp.speed_mps > c.overspeed_mps:
            f.append(f"overspeed {inp.speed_mps:.2f} m/s > {c.overspeed_mps:.2f}")
        if not _fresh(inp, "odom", c.odom_timeout_s):
            f.append("odometry stale")
        if not _fresh(inp, "perception", c.perception_timeout_s):
            f.append("perception status stale")
        if not _fresh(inp, "mapping", c.mapping_timeout_s):
            f.append("mapping status stale")
        if not _fresh(inp, "planner", c.planner_timeout_s):
            f.append("planner status stale")
        if not _fresh(inp, "controller", c.controller_timeout_s):
            f.append("controller status stale")
        return f

    def step(self, now: float, inp: Inputs, requests: List[str]) -> SafetyOutput:
        events: List[str] = []
        vcu_req: Optional[str] = None
        vcu = inp.vcu
        hb_ok = _fresh(inp, "vcu", self.cfg.vcu_timeout_s)
        inp.manual_power_cut_confirmed = self.manual_power_cut_confirmed
        initial_stages, _, _ = evaluate_startup(inp, self.cfg)
        stage_fault = any(stage["state"] == FAULT for stage in initial_stages[:8])
        if (not hb_ok or vcu is None or vcu.state in ("FAULT", "UNKNOWN") or
                (vcu.estop_input_fitted and vcu.estop_active) or stage_fault or self.state == "FAULT"):
            self.manual_power_cut_confirmed = False
        if ("confirm_manual_power_cut" in requests and hb_ok and vcu is not None and vcu.state == "SAFE" and
                self.state == "DISARMED" and not self.stop_latched and not stage_fault and
                (not vcu.estop_input_fitted or not vcu.estop_active)):
            self.manual_power_cut_confirmed = True
            events.append("operator confirmed physical power cut within reach")
        inp.manual_power_cut_confirmed = self.manual_power_cut_confirmed
        stages, ready, reasons = evaluate_startup(inp, self.cfg)

        for r in requests:
            if r == "confirm_manual_power_cut":
                continue
            elif r == "stop":
                if self.state in ("ARMED", "ARMING"):
                    events.append("STOP requested: disarming")
                    self.stopping_since = now
                self.state = "DISARMED"
                self.stop_latched = True
                self.arming_since = None
                self.manual_power_cut_confirmed = False
                vcu_req = "DIS"
            elif r == "reset_fault":
                if self.state == "FAULT" or self.stop_latched:
                    cause_gone = (hb_ok and vcu is not None and
                                  (not vcu.estop_input_fitted or not vcu.estop_active) and
                                  inp.speed_valid and inp.speed_mps is not None and
                                  inp.speed_mps <= self.cfg.stop_speed_mps)
                    if cause_gone:
                        self.state, self.stop_latched, self.fault_reasons = "DISARMED", False, []
                        self.manual_power_cut_confirmed = False
                        vcu_req = "RST" if (vcu is not None and vcu.state == "FAULT") else vcu_req
                        events.append("fault/stop latch cleared")
                    else:
                        events.append("reset refused: cause still present (VCU link or E-stop)")
            elif r == "arm":
                if self.state != "DISARMED" or self.stop_latched:
                    events.append("arm refused: " + (
                        "stop/fault latched, reset first" if self.stop_latched or self.state == "FAULT"
                        else f"state {self.state}"))
                elif not ready:
                    events.append("arm refused: " + "; ".join(reasons[:3]))
                elif not hb_ok or vcu is None or vcu.state != "SAFE":
                    events.append("arm refused: VCU not in SAFE state")
                else:
                    self.state = "ARMING"
                    self.arming_since = now
                    vcu_req = "ARM"
                    events.append("arming requested")
            elif r == "demo_drive_start":
                # DEMO DRIVE, not autonomy: only ever granted while already ARMED, same as every
                # other request here. Stop/fault/disarm below always clears it, never silently.
                if self.state != "ARMED":
                    events.append(f"demo drive refused: state {self.state}, must be ARMED")
                else:
                    self.demo_drive_active = True
                    events.append("demo drive granted")
            elif r == "demo_drive_stop":
                self.demo_drive_active = False
                events.append("demo drive ended")

        if self.state == "ARMING":
            if hb_ok and vcu is not None and vcu.state == "ARMED":
                self.state = "ARMED"
                events.append("ARMED")
            elif self.arming_since is not None and now - self.arming_since > self.cfg.arming_timeout_s:
                self.state = "DISARMED"
                self.manual_power_cut_confirmed = False
                events.append("arming timed out (VCU did not arm)")
                vcu_req = "DIS"

        if self.state == "ARMED":
            faults = self._runtime_faults(inp)
            if inp.planner_decision in STOP_DECISIONS:
                self.state = "DISARMED"
                self.manual_power_cut_confirmed = False
                self.stop_latched = True
                self.stopping_since = now
                vcu_req = "DIS"
                events.append("planner stop latched: disarming")
            else:
                self.stopping_since = None
            if faults and self.state == "ARMED":
                self.state = "FAULT"
                self.manual_power_cut_confirmed = False
                self.fault_reasons = faults
                vcu_req = "DIS"
                events.append("FAULT: " + "; ".join(faults))
        elif self.stop_latched and self.stopping_since is not None:
            if (now - self.stopping_since > self.cfg.stop_ineffective_s and inp.speed_valid and
                    inp.speed_mps is not None and inp.speed_mps > self.cfg.stop_speed_mps):
                self.state = "FAULT"
                self.manual_power_cut_confirmed = False
                self.fault_reasons = ["stop not effective: still moving after braking"]
                vcu_req = "DIS"
                events.append("FAULT: stop not effective")
        else:
            self.stopping_since = None

        # Single point of truth: demo_drive_active can never be true except in plain ARMED state --
        # this covers every exit path above (stop, fault, arming timeout, planner stop, runtime
        # fault, stop-not-effective) without needing to clear it at each one individually.
        if self.state != "ARMED":
            self.demo_drive_active = False

        inp.manual_power_cut_confirmed = self.manual_power_cut_confirmed
        stages, ready, reasons = evaluate_startup(inp, self.cfg)
        out_reasons = list(self.fault_reasons) if self.state == "FAULT" else list(reasons)
        arm_allowed = (self.state == "DISARMED" and ready and not self.stop_latched and hb_ok and
                      vcu is not None and vcu.state == "SAFE")
        return SafetyOutput(self.state, arm_allowed, hb_ok, ready, self.stop_latched,
                            self.manual_power_cut_confirmed, out_reasons, stages,
                            vcu_req, events, demo_drive_active=self.demo_drive_active)
