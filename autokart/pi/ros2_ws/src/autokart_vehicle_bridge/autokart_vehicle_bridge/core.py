"""Bridge logic that does not need ROS: angle calibration, odometry, and the command gate."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from .protocol import THROTTLE_MAX_MV


def wrap_raw_delta(raw: int, zero: int) -> int:
    """12-bit AS5600 difference wrapped into [-2048, 2047]."""
    d = (raw - zero) % 4096
    return d - 4096 if d >= 2048 else d


def raw_to_deg(raw: int, zero_raw: int, deg_per_raw: float) -> Optional[float]:
    if raw < 0:
        return None
    return wrap_raw_delta(raw, zero_raw) * deg_per_raw


class Odometry:
    """Wheel-encoder odometry. Yaw comes from a bicycle model using the MEASURED steering angle.

    There is no gyro in this configuration, so yaw is an estimate; status says so.
    """

    def __init__(self, wheelbase_m: float):
        self.L = wheelbase_m
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.status = "ENCODER_ONLY"

    def update(self, dt: float, speed_mps: float, steer_deg: Optional[float]) -> None:
        if dt <= 0.0 or dt > 0.2:
            return
        if steer_deg is None or self.L <= 0:
            self.yaw_rate = 0.0
            self.status = "ENCODER_ONLY_NO_YAW"
        else:
            self.yaw_rate = speed_mps / self.L * math.tan(math.radians(steer_deg))
            self.status = "ENCODER_STEER_MODEL"
        yaw_mid = self.yaw + 0.5 * self.yaw_rate * dt
        self.x += speed_mps * dt * math.cos(yaw_mid)
        self.y += speed_mps * dt * math.sin(yaw_mid)
        self.yaw += self.yaw_rate * dt

    def reset(self) -> None:
        self.x = self.y = self.yaw = self.yaw_rate = 0.0


@dataclass
class GateOutput:
    throttle_mv: int
    steer_cdeg: int
    brake_pct: int
    reason: str


class CommandGate:
    """Last line of defence on the laptop. Controller output only passes when ALL of these hold:
    safety state is ARMED, the VCU reports ARMED, the controller command is fresh.
    Throttle is clamped to [0, throttle_v_max] and never exceeds the 2 V hard cap.
    """

    def __init__(self, throttle_v_max: float = 2.0, cmd_timeout_s: float = 0.25):
        self.v_max = min(float(throttle_v_max), THROTTLE_MAX_MV / 1000.0)
        self.timeout = cmd_timeout_s

    def compute(self, now: float, safety_armed: bool, vcu_armed: bool,
                cmd: Optional[Tuple[float, float, float]], cmd_stamp: Optional[float]) -> GateOutput:
        if not safety_armed:
            return GateOutput(0, 0, 100, "safety not armed")
        if not vcu_armed:
            return GateOutput(0, 0, 100, "vcu not armed")
        if cmd is None or cmd_stamp is None or now - cmd_stamp > self.timeout:
            return GateOutput(0, 0, 100, "controller command stale")
        thr_v, steer_deg, brake_pct = cmd
        if not all(math.isfinite(x) for x in (thr_v, steer_deg, brake_pct)):
            return GateOutput(0, 0, 100, "non-finite command")
        thr_v = max(0.0, min(thr_v, self.v_max))
        brake_pct = max(0.0, min(brake_pct, 100.0))
        if brake_pct >= 50.0:
            thr_v = 0.0
        return GateOutput(int(round(thr_v * 1000)), int(round(steer_deg * 100)), int(round(brake_pct)), "pass")
