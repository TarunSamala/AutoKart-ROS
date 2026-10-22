"""Safety supervisor logic (no ROS). Startup stages are computed ONLY from real signals.

READY is never inferred from "a process exists": every stage reads data freshness or a reported flag.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

READY, INIT, DEGRADED, FAULT, UNAVAILABLE = "READY", "INITIALIZING", "DEGRADED", "FAULT", "UNAVAILABLE"

STAGE_NAMES = [
    "Start ROS 2 runtime", "ESP32-S3 USB communication", "Firmware / protocol compatibility",
    "AS5047P axle encoder", "Steering + brake encoders", "MPU-6500 IMU", "HP60C SDK + camera",
    "RGB/depth synchronization", "YOLO26s ONNX model", "Camera calibration", "Localization + mapping",
    "Planner + controller", "Independent E-stop + interlocks", "Disarmed, mission-ready",
]


@dataclass
class TelView:
    state: int = 0
    fault: int = 0
    fault_names: List[str] = field(default_factory=list)
    estop_ok: bool = False
    steer_pwm_ok: bool = False
    brake_pwm_ok: bool = False
    axle_motion_seen: bool = False
    protocol_ok: bool = False
    fw_version: str = ""
    speed_mps: float = 0.0


@dataclass
class Inputs:
    ages: Dict[str, Optional[float]]          # seconds since last message per topic; None = never received
    tel: Optional[TelView] = None
    detections_depth_valid: bool = False
    detections_inference_ms: float = 0.0
    calibration_verified: bool = False
    vehicle_config_verified: bool = False
    braking_verified: bool = False
    estop_operator_confirmed: bool = False
    controller_state: str = "IDLE"
    controller_stopped: bool = False


@dataclass
class Config:
    required_stages: Tuple[int, ...] = (1, 2, 3, 4, 7, 8, 9, 10, 11, 12, 13, 14)
    tel_timeout_s: float = 0.3
    image_timeout_s: float = 0.5
    detections_timeout_s: float = 0.5
    odom_timeout_s: float = 0.3
    map_timeout_s: float = 1.0
    planner_timeout_s: float = 0.5
    controller_timeout_s: float = 0.5
    overspeed_mps: float = 2.5          # tune after measuring top speed at the 2 V throttle cap
    stop_ineffective_s: float = 4.0
    stop_speed_mps: float = 0.15
    arming_timeout_s: float = 3.0


def _age(inp: Inputs, key: str) -> Optional[float]:
    return inp.ages.get(key)


def _fresh(inp: Inputs, key: str, limit: float) -> bool:
    a = _age(inp, key)
    return a is not None and a <= limit


def evaluate_startup(inp: Inputs, cfg: Config) -> Tuple[List[dict], bool, List[str]]:
    """Return (stages, ready, reasons). Stage dicts: id, name, state, detail."""
    S: Dict[int, Tuple[str, str]] = {}
    tel = inp.tel
    tel_a = _age(inp, "telemetry")

    S[1] = (READY, "safety node running")

    if tel_a is None:
        S[2] = (UNAVAILABLE, "no telemetry from VCU")
    elif tel_a > cfg.tel_timeout_s:
        S[2] = (FAULT, f"telemetry stale ({tel_a:.1f} s)")
    else:
        S[2] = (READY, "telemetry fresh")

    if tel is None or tel_a is None:
        S[3] = (UNAVAILABLE, "no VCU")
    elif not tel.fw_version:
        S[3] = (INIT, "waiting for HELLO")
    elif not tel.protocol_ok:
        S[3] = (FAULT, f"protocol mismatch (fw {tel.fw_version})")
    else:
        S[3] = (READY, f"fw {tel.fw_version}")

    if tel is None:
        S[4] = (UNAVAILABLE, "no VCU")
    elif tel.axle_motion_seen:
        S[4] = (READY, "counts changed since boot")
    else:
        S[4] = (DEGRADED, "unverified: rotate the wheel by hand once")

    if tel is None:
        S[5] = (UNAVAILABLE, "no VCU")
    elif tel.steer_pwm_ok and tel.brake_pwm_ok:
        S[5] = (READY, "both AS5600 PWM signals valid")
    elif tel.steer_pwm_ok or tel.brake_pwm_ok:
        S[5] = (DEGRADED, "steering ok" if tel.steer_pwm_ok else "brake ok")
    else:
        S[5] = (UNAVAILABLE, "no AS5600 PWM signal")

    S[6] = (UNAVAILABLE, "not wired in this build (wheel+steering odometry only)")

    ia = _age(inp, "image")
    if ia is None:
        S[7] = (UNAVAILABLE, "no camera frames")
    elif ia > cfg.image_timeout_s:
        S[7] = (FAULT, f"frames stale ({ia:.1f} s)")
    else:
        S[7] = (READY, "frames arriving")

    da = _age(inp, "detections")
    if da is None:
        S[8] = (UNAVAILABLE, "no synchronized frames")
        S[9] = (UNAVAILABLE, "no inference output")
    elif da > cfg.detections_timeout_s:
        S[8] = (FAULT, f"detections stale ({da:.1f} s)")
        S[9] = (FAULT, f"detections stale ({da:.1f} s)")
    else:
        S[8] = (READY, "RGB+depth paired") if inp.detections_depth_valid else (FAULT, "depth invalid")
        S[9] = (READY, f"inference {inp.detections_inference_ms:.0f} ms") if inp.detections_inference_ms > 0 \
            else (INIT, "model not producing output")

    S[10] = (READY, "intrinsics + mount marked verified") if inp.calibration_verified \
        else (DEGRADED, "calibration not marked verified (config/camera/camera.yaml)")

    odom_ok = _fresh(inp, "odom", cfg.odom_timeout_s)
    map_ok = _fresh(inp, "map", cfg.map_timeout_s)
    if odom_ok and map_ok:
        S[11] = (READY, "odometry + map publishing")
    elif _age(inp, "odom") is None and _age(inp, "map") is None:
        S[11] = (UNAVAILABLE, "no odometry or map")
    else:
        S[11] = (FAULT if (_age(inp, "odom") is not None or _age(inp, "map") is not None) else UNAVAILABLE,
                 f"odom {'ok' if odom_ok else 'stale'}, map {'ok' if map_ok else 'stale'}")

    pl_ok = _fresh(inp, "planner", cfg.planner_timeout_s)
    ct_ok = _fresh(inp, "controller", cfg.controller_timeout_s)
    if pl_ok and ct_ok:
        S[12] = (READY, "planner + controller publishing")
    elif _age(inp, "planner") is None and _age(inp, "controller") is None:
        S[12] = (UNAVAILABLE, "no planner or controller")
    else:
        S[12] = (FAULT, f"planner {'ok' if pl_ok else 'stale'}, controller {'ok' if ct_ok else 'stale'}")

    if tel is None or tel_a is None:
        S[13] = (UNAVAILABLE, "no VCU")
    elif not tel.estop_ok:
        S[13] = (FAULT, "E-stop circuit reports not OK")
    elif not inp.estop_operator_confirmed:
        S[13] = (DEGRADED, "status line OK; operator must confirm the physical E-stop test")
    else:
        S[13] = (READY, "status OK + operator confirmed")

    reasons: List[str] = []
    for sid in cfg.required_stages:
        if sid == 14:
            continue
        st, detail = S[sid]
        if st != READY:
            reasons.append(f"stage {sid:02d} {STAGE_NAMES[sid-1]}: {st} ({detail})")
    if not inp.vehicle_config_verified:
        reasons.append("vehicle config (wheelbase, tyre radius, steering scale) not marked verified")
    if not inp.braking_verified:
        reasons.append("braking deceleration not verified by a physical test")
    if tel is not None and tel.state == 2:
        reasons.append("VCU latched FAULT: " + ",".join(tel.fault_names or ["?"]))

    ready = not reasons
    S[14] = (READY, "awaiting authorization") if ready else (DEGRADED, f"{len(reasons)} blocking item(s)")
    stages = [{"id": i, "name": STAGE_NAMES[i - 1], "state": S[i][0], "detail": S[i][1]} for i in range(1, 15)]
    return stages, ready, reasons


@dataclass
class SafetyOutput:
    state: str
    arm_allowed: bool
    vcu_heartbeat_ok: bool
    startup_ready: bool
    stop_latched: bool
    reasons: List[str]
    stages: List[dict]
    vcu_request: Optional[str] = None      # "ARM" | "DISARM" | "RESET"
    events: List[str] = field(default_factory=list)


class SafetyLogic:
    def __init__(self, cfg: Optional[Config] = None):
        self.cfg = cfg or Config()
        self.state = "DISARMED"
        self.stop_latched = False
        self.arming_since: Optional[float] = None
        self.stopping_since: Optional[float] = None
        self.fault_reasons: List[str] = []

    def _runtime_faults(self, inp: Inputs) -> List[str]:
        c, f = self.cfg, []
        tel = inp.tel
        if not _fresh(inp, "telemetry", c.tel_timeout_s):
            f.append("VCU heartbeat lost")
        elif tel is not None:
            if tel.state == 2:
                f.append("VCU fault: " + ",".join(tel.fault_names or ["?"]))
            elif tel.state == 0:
                f.append("VCU disarmed unexpectedly")
            if not tel.estop_ok:
                f.append("E-stop circuit not OK")
            if tel.speed_mps > c.overspeed_mps:
                f.append(f"overspeed {tel.speed_mps:.2f} m/s > {c.overspeed_mps:.2f}")
        finished = inp.controller_state == "STOPPED" and inp.controller_stopped
        if not finished:  # once safely stopped, a blind camera is no longer a hazard
            if not _fresh(inp, "image", c.image_timeout_s):
                f.append("camera frames stale")
            if not _fresh(inp, "detections", c.detections_timeout_s):
                f.append("detections stale")
            if not _fresh(inp, "map", c.map_timeout_s):
                f.append("map stale")
            if not _fresh(inp, "planner", c.planner_timeout_s):
                f.append("planner stale")
        if not _fresh(inp, "odom", c.odom_timeout_s):
            f.append("odometry stale")
        if not _fresh(inp, "controller", c.controller_timeout_s):
            f.append("controller stale")
        return f

    def step(self, now: float, inp: Inputs, requests: List[str]) -> SafetyOutput:
        events: List[str] = []
        vcu_req: Optional[str] = None
        stages, ready, reasons = evaluate_startup(inp, self.cfg)
        tel = inp.tel
        hb_ok = _fresh(inp, "telemetry", self.cfg.tel_timeout_s)

        for r in requests:
            if r == "confirm_estop_tested":
                events.append("operator confirmed physical E-stop test")  # applied by the node as an input
            elif r == "stop":
                if self.state in ("ARMED", "ARMING"):
                    events.append("STOP requested: disarming")
                self.state = "DISARMED"
                self.stop_latched = True
                self.arming_since = None
                vcu_req = "DISARM"
            elif r == "reset_fault":
                if self.state == "FAULT" or self.stop_latched:
                    cause_gone = hb_ok and tel is not None and tel.estop_ok
                    if cause_gone:
                        self.state, self.stop_latched, self.fault_reasons = "DISARMED", False, []
                        vcu_req = "RESET" if tel.state == 2 else vcu_req
                        events.append("fault/stop latch cleared")
                    else:
                        events.append("reset refused: cause still present (VCU link or E-stop)")
            elif r == "arm":
                if self.state != "DISARMED" or self.stop_latched:
                    events.append("arm refused: " + ("stop/fault latched, reset first" if self.stop_latched or self.state == "FAULT" else f"state {self.state}"))
                elif not ready:
                    events.append("arm refused: " + "; ".join(reasons[:3]))
                elif not hb_ok or tel is None or tel.state != 0:
                    events.append("arm refused: VCU not in SAFE state")
                else:
                    self.state = "ARMING"
                    self.arming_since = now
                    vcu_req = "ARM"
                    events.append("arming VCU")

        if self.state == "ARMING":
            if hb_ok and tel is not None and tel.state == 1:
                self.state = "ARMED"
                events.append("ARMED")
            elif self.arming_since is not None and now - self.arming_since > self.cfg.arming_timeout_s:
                self.state = "DISARMED"
                events.append("arming timed out (VCU did not arm)")
                vcu_req = "DISARM"

        if self.state == "ARMED":
            faults = self._runtime_faults(inp)
            # Stop that does not slow the kart down is itself a fault.
            if inp.controller_state == "STOPPING":
                self.stopping_since = self.stopping_since if self.stopping_since is not None else now
                if now - self.stopping_since > self.cfg.stop_ineffective_s and tel is not None and \
                        tel.speed_mps > self.cfg.stop_speed_mps:
                    faults.append("stop not effective: still moving after braking")
            else:
                self.stopping_since = None
            if faults:
                self.state = "FAULT"
                self.fault_reasons = faults
                vcu_req = "DISARM"
                events.append("FAULT: " + "; ".join(faults))
        else:
            self.stopping_since = None

        out_reasons = list(self.fault_reasons) if self.state == "FAULT" else list(reasons)
        arm_allowed = (self.state == "DISARMED" and ready and not self.stop_latched and hb_ok
                       and tel is not None and tel.state == 0)
        return SafetyOutput(self.state, arm_allowed, hb_ok, ready, self.stop_latched, out_reasons, stages, vcu_req, events)
