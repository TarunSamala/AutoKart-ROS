"""Pure Pursuit + speed control (no ROS).

Steering is computed and reported for observation only. In this release the VCU does not actuate steering.
Throttle is limited to the 2 V hard cap here, in the bridge gate, and in the firmware.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

Pose = Tuple[float, float, float]
THROTTLE_V_MAX = 2.0
THROTTLE_RUN_V = 2.0   # measured motor start voltage (see esp32_firmware/include/config.h THROTTLE_RUN_MV)


@dataclass
class ControllerConfig:
    wheelbase_m: float = 1.0            # MEASURE
    lookahead_min_m: float = 0.8
    lookahead_max_m: float = 2.5
    lookahead_gain_s: float = 1.0       # Ld = min + gain * speed
    max_steer_deg: float = 25.0         # MEASURE: mechanical limit
    volts_per_mps: float = 1.0          # MEASURE: throttle volts needed per m/s (feed-forward)
    kp_v_per_mps: float = 0.4
    ki_v_per_mps_s: float = 0.2
    pi_trim_max_v: float = 0.4
    throttle_v_max: float = THROTTLE_V_MAX
    throttle_run_v: float = THROTTLE_RUN_V   # commanded throttle while RUN is 0 or in [throttle_run_v, throttle_v_max]
    stopped_speed_mps: float = 0.03
    stopped_hold_s: float = 1.0


@dataclass
class SteerResult:
    valid: bool
    delta_deg: float = 0.0
    alpha_rad: float = 0.0
    lookahead_m: float = 0.0
    lookahead_xy: Tuple[float, float] = (0.0, 0.0)


def pure_pursuit(path: List[Tuple[float, float]], pose: Pose, speed: float, cfg: ControllerConfig) -> SteerResult:
    """delta = atan(2 L sin(alpha) / Ld). Lookahead point is the first path point at distance >= Ld ahead of the kart."""
    if len(path) < 2:
        return SteerResult(False)
    px, py, yaw = pose
    ld = max(cfg.lookahead_min_m, min(cfg.lookahead_max_m, cfg.lookahead_min_m + cfg.lookahead_gain_s * max(0.0, speed)))
    c, s = math.cos(yaw), math.sin(yaw)
    target = None
    for x, y in path:
        dx, dy = x - px, y - py
        fx, fy = c * dx + s * dy, -s * dx + c * dy      # vehicle frame
        if fx > 0.0 and math.hypot(fx, fy) >= ld:
            target = (x, y, fx, fy)
            break
    if target is None:                                   # end of path: aim at the last point
        x, y = path[-1]
        dx, dy = x - px, y - py
        fx, fy = c * dx + s * dy, -s * dx + c * dy
        if fx <= 0.05:
            return SteerResult(False)
        target = (x, y, fx, fy)
    x, y, fx, fy = target
    dist = math.hypot(fx, fy)
    alpha = math.atan2(fy, fx)
    delta = math.atan2(2.0 * cfg.wheelbase_m * math.sin(alpha), max(dist, 1e-6))
    deg = max(-cfg.max_steer_deg, min(cfg.max_steer_deg, math.degrees(delta)))
    return SteerResult(True, deg, alpha, dist, (x, y))


class SpeedController:
    """Feed-forward + clamped PI. Output is 0 V, or clamped into [throttle_run_v, throttle_v_max] (<= 2.0).

    The motor does not move below roughly throttle_run_v, so a nonzero command is never left sitting between
    0 and that floor. When throttle_run_v == throttle_v_max this degenerates to on/off speed control: every
    RUN command is exactly that one voltage, and the PI trim can never actually change the output — anti-windup
    below stops the integral accumulating in that case instead of growing unbounded against a wall it can never
    move.
    """

    def __init__(self, cfg: ControllerConfig):
        self.cfg = cfg
        self.integral = 0.0
        self.last_t: Optional[float] = None

    def reset(self):
        self.integral = 0.0
        self.last_t = None

    def step(self, now: float, target: float, measured: float) -> float:
        c = self.cfg
        dt = 0.0 if self.last_t is None else max(0.0, min(0.2, now - self.last_t))
        self.last_t = now
        if target <= 0.0:
            self.integral = 0.0
            return 0.0

        hi = min(c.throttle_v_max, THROTTLE_V_MAX)
        lo = min(max(0.0, c.throttle_run_v), hi)  # never above hi, even with a bad config

        err = target - measured
        ff = c.volts_per_mps * target
        # Candidate integral update; only committed below if it would not drive the output deeper into
        # saturation (conditional integration anti-windup).
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
class ControlOutput:
    state: str                  # IDLE RUN STOPPING STOPPED
    throttle_v: float
    brake_pct: float
    steer_deg: float
    stopped: bool
    steer: SteerResult


class Controller:
    def __init__(self, cfg: ControllerConfig):
        self.cfg = cfg
        self.speed_ctl = SpeedController(cfg)
        self.state = "IDLE"
        self.slow_since: Optional[float] = None

    def step(self, now: float, armed: bool, decision: str, target_speed: float, path: List[Tuple[float, float]],
             pose: Pose, speed: float) -> ControlOutput:
        steer = pure_pursuit(path, pose, speed, self.cfg)
        steer_deg = steer.delta_deg if steer.valid else 0.0
        c = self.cfg

        if not armed:
            self.state = "IDLE"
            self.slow_since = None
            self.speed_ctl.reset()
            return ControlOutput("IDLE", 0.0, 100.0, steer_deg, False, steer)

        if decision in ("BRAKE", "STOP"):
            if self.state not in ("STOPPING", "STOPPED"):
                self.state = "STOPPING"
                self.slow_since = None
            if speed <= c.stopped_speed_mps:
                self.slow_since = self.slow_since if self.slow_since is not None else now
                if now - self.slow_since >= c.stopped_hold_s:
                    self.state = "STOPPED"
            else:
                self.slow_since = None
                if self.state == "STOPPED":
                    self.state = "STOPPING"
            self.speed_ctl.reset()
            return ControlOutput(self.state, 0.0, 100.0, steer_deg, self.state == "STOPPED", steer)

        if self.state in ("STOPPING", "STOPPED"):
            # Once a stop has been commanded the controller never drives again on its own;
            # the operator must disarm and start a new mission.
            return ControlOutput(self.state, 0.0, 100.0, steer_deg, self.state == "STOPPED", steer)

        if decision == "CRUISE" and target_speed > 0.0:
            self.state = "RUN"
            thr = self.speed_ctl.step(now, target_speed, speed)
            return ControlOutput("RUN", thr, 0.0, steer_deg, False, steer)

        self.state = "IDLE"
        self.speed_ctl.reset()
        return ControlOutput("IDLE", 0.0, 100.0 if decision != "CRUISE" else 0.0, steer_deg, False, steer)
