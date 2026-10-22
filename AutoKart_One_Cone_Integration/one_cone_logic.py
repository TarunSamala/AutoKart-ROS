"""Fail-closed ONE-CONE stop decision logic. NO hardware access.

Input distances MUST be independently calibrated bumper-to-nearest-cone-surface
clearances, not raw HP60C camera-relative depths. Units are metres and seconds.
This module cannot guarantee stopping; the VCU and independent brake remain required.
"""
from dataclasses import dataclass
from enum import Enum
import math

class Decision(str, Enum):
    STOP = 'STOP_REQUEST'
    APPROACH_CANDIDATE = 'APPROACH_CANDIDATE'  # not a motor command!

@dataclass(frozen=True)
class Sample:
    cone_gap_m: float
    wheel_speed_mps: float
    observation_age_s: float
    source: str
    calibration_status: str
    depth_valid: bool
    wheel_speed_valid: bool
    emergency_stop_clear: bool

@dataclass(frozen=True)
class Limits:
    target_gap_m: float = 0.15
    latency_s: float = 0.20              # provisional; measure before driving
    min_decel_mps2: float = 0.50         # provisional; measure before driving
    distance_margin_m: float = 0.10      # provisional; verify statistically
    max_observation_age_s: float = 0.15
    max_wheel_speed_mps: float = 0.20    # provisional; independent of DAC volts
    max_valid_gap_m: float = 10.0

@dataclass(frozen=True)
class Result:
    decision: Decision
    reason: str
    trigger_gap_m: float | None

class OneConePlanner:
    def __init__(self, limits: Limits=Limits()):
        self.limits=limits
        self.latched_stop = False

    def update(self, s: Sample) -> Result:
        l=self.limits
        if self.latched_stop:
            return Result(Decision.STOP, 'LATCHED_STOP_NEEDS_RESET', None)
        if not s.emergency_stop_clear:
            self.latched_stop=True
            return Result(Decision.STOP,'ESTOP_NOT_CLEAR',None)
        if s.source != 'LIVE' or s.calibration_status != 'MOUNT_VERIFIED':
            self.latched_stop=True
            return Result(Decision.STOP,'UNTRUSTED_SENSOR_OR_CALIBRATION',None)
        if not s.depth_valid or not s.wheel_speed_valid:
            self.latched_stop=True
            return Result(Decision.STOP,'INVALID_DEPTH_OR_SPEED',None)
        values=(s.cone_gap_m, s.wheel_speed_mps, s.observation_age_s)
        if not all(math.isfinite(v) for v in values):
            self.latched_stop=True
            return Result(Decision.STOP,'NONFINITE_MEASUREMENT',None)
        if not (0 <= s.cone_gap_m <= l.max_valid_gap_m) or s.wheel_speed_mps < -0.01:
            self.latched_stop=True
            return Result(Decision.STOP,'OUT_OF_RANGE_MEASUREMENT',None)
        if not (0 <= s.observation_age_s <= l.max_observation_age_s):
            self.latched_stop=True
            return Result(Decision.STOP,'STALE_OBSERVATION',None)
        if s.wheel_speed_mps > l.max_wheel_speed_mps:
            self.latched_stop=True
            return Result(Decision.STOP,'OVERSPEED',None)
        if l.min_decel_mps2 <= 0 or l.latency_s < 0 or l.distance_margin_m < 0:
            self.latched_stop=True
            return Result(Decision.STOP,'INVALID_STOP_MODEL',None)
        v=max(0.0,s.wheel_speed_mps)
        trigger=l.target_gap_m + v*l.latency_s + (v*v)/(2*l.min_decel_mps2) + l.distance_margin_m
        if s.cone_gap_m <= trigger:
            self.latched_stop=True
            return Result(Decision.STOP,'STOP_ENVELOPE_REACHED',trigger)
        return Result(Decision.APPROACH_CANDIDATE,'DISTANCE_OUTSIDE_STOP_ENVELOPE',trigger)

    def reset_for_NEW_BENCH_TEST_only(self):
        """Call only after external VCU/safety layer has independently disarmed."""
        self.latched_stop=False
