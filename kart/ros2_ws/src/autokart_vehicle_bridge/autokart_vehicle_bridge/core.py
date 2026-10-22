"""Pure logic for the real VCU bridge: throttle-voltage mapping and the command gate. No ROS dependencies.

autokart_controller publishes a candidate MotionCommand (target_speed_mps, steering_angle_rad) -- see
docs/NODE_CONTRACTS.md: "A future VCU node subscribes to the candidate topic and applies its own gates." This
module is that VCU-specific layer: it never actuates steering_angle_rad (observed only, per the workspace hard
rules), and it converts target_speed_mps into a throttle voltage that is physically meaningful for THIS motor
(the motor does not move below roughly throttle_run_v), which the abstract MotionCommand schema has no field
for and should not need to know about.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from .protocol import THROTTLE_MAX_MV


def arm_request_approved(now: float, arm_pending: bool, safety_stamp: Optional[float],
                         request_stamp: Optional[float], timeout_s: float = 0.3) -> bool:
    return (arm_pending and safety_stamp is not None and request_stamp is not None and
            0.0 <= now - safety_stamp <= timeout_s and
            0.0 <= now - request_stamp <= timeout_s)


@dataclass
class ThrottleConfig:
    throttle_v_max: float = 2.0       # absolute ceiling; matches vcu_firmware THROTTLE_MAX_MV (2000 mV)
    throttle_run_v: float = 2.0       # MEASURE: motor start voltage; matches vcu_firmware THROTTLE_RUN_MV
    volts_per_mps: float = 1.0        # MEASURE: feed-forward volts needed per m/s
    kp_v_per_mps: float = 0.4
    ki_v_per_mps_s: float = 0.2
    pi_trim_max_v: float = 0.4


class ThrottleController:
    """Feed-forward + clamped PI converting a candidate target speed into a throttle voltage.

    Output is always 0 V, or clamped into [throttle_run_v, throttle_v_max] -- never in between, because the
    motor does not move below roughly throttle_run_v. Conditional-integration anti-windup stops the integral
    growing while the output is saturated against that floor/ceiling, which with throttle_run_v == throttle_v_max
    (the as-shipped default) is true for every nonzero command: speed control is then effectively on/off.
    """

    def __init__(self, cfg: ThrottleConfig):
        self.cfg = cfg
        self.integral = 0.0
        self.last_t: Optional[float] = None

    def reset(self) -> None:
        self.integral = 0.0
        self.last_t = None

    def step(self, now: float, target_mps: float, measured_mps: float) -> float:
        c = self.cfg
        dt = 0.0 if self.last_t is None else max(0.0, min(0.2, now - self.last_t))
        self.last_t = now
        if target_mps <= 0.0:
            self.integral = 0.0
            return 0.0

        hi = min(c.throttle_v_max, THROTTLE_MAX_MV / 1000.0)
        lo = min(max(0.0, c.throttle_run_v), hi)

        err = target_mps - measured_mps
        ff = c.volts_per_mps * target_mps
        candidate_integral = self.integral + err * dt
        i_term = c.ki_v_per_mps_s * candidate_integral
        if abs(i_term) > c.pi_trim_max_v:
            i_term = math.copysign(c.pi_trim_max_v, i_term)
            candidate_integral = i_term / max(c.ki_v_per_mps_s, 1e-9)
        trim = max(-c.pi_trim_max_v, min(c.pi_trim_max_v, c.kp_v_per_mps * err + i_term))
        raw = ff + trim
        out = max(lo, min(hi, raw))

        saturated_high = raw > hi
        saturated_low = raw < lo
        if not ((saturated_high and err > 0.0) or (saturated_low and err < 0.0)):
            self.integral = candidate_integral
        return out


@dataclass
class GateOutput:
    throttle_mv: int
    brake_pct: int
    reason: str


class CommandGate:
    """Last line of defence before a throttle voltage reaches the serial link.

    Forces throttle to 0 (and brake to 100) unless: the upstream VCU safety supervisor reports ARMED, the VCU
    itself reports ARMED (TEL state == 1), and the candidate command is fresh. Output throttle never exceeds the
    absolute 2 V ceiling regardless of misconfiguration.
    """

    def __init__(self, throttle_v_max: float = 2.0, cmd_timeout_s: float = 0.25):
        self.v_max = min(float(throttle_v_max), THROTTLE_MAX_MV / 1000.0)
        self.timeout = cmd_timeout_s

    def compute(self, now: float, safety_armed: bool, vcu_armed: bool,
                throttle_v: float, brake_pct: float, cmd_stamp: Optional[float]) -> GateOutput:
        if not safety_armed:
            return GateOutput(0, 100, "safety not armed")
        if not vcu_armed:
            return GateOutput(0, 100, "vcu not armed")
        if cmd_stamp is None or now - cmd_stamp > self.timeout:
            return GateOutput(0, 100, "controller command stale")
        if not math.isfinite(throttle_v) or not math.isfinite(brake_pct):
            return GateOutput(0, 100, "non-finite command")
        throttle_v = max(0.0, min(throttle_v, self.v_max))
        brake_pct = max(0.0, min(brake_pct, 100.0))
        if brake_pct >= 50.0:
            throttle_v = 0.0
        return GateOutput(int(round(throttle_v * 1000)), int(round(brake_pct)), "pass")


def brake_pct_for_candidate(valid: bool, target_speed_mps: float) -> float:
    """MotionCommand has no brake field (see autokart_msgs/MotionCommand.msg): an invalid or non-positive
    candidate means stop, so this bridge brakes (100%) rather than merely coasting to 0 V throttle."""
    if not valid or not math.isfinite(target_speed_mps) or target_speed_mps <= 0.0:
        return 100.0
    return 0.0


def wrap_raw_delta(raw: int, zero: int) -> int:
    """12-bit AS5600 difference wrapped into [-2048, 2047]."""
    d = (raw - zero) % 4096
    return d - 4096 if d >= 2048 else d


def raw_to_rad(raw: int, zero_raw: int, rad_per_raw: float) -> Optional[float]:
    if raw < 0:
        return None
    return math.radians(wrap_raw_delta(raw, zero_raw) * rad_per_raw)
