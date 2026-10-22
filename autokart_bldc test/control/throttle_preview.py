"""BLDC throttle command PREVIEW.

Computes what DAC millivolt command WOULD be sent for a given target speed,
with a configurable ceiling, ramp rate, min/max and a command-timeout concept
mirroring vcu_firmware's own 1000 mV/s ramp and 300 ms CMD_TIMEOUT_MS
(docs/VCU_PROTOCOL.md). It never actuates anything by itself:

- `enabled` defaults to, and is only ever read from, config.throttle.motor_output_enabled.
  This module has no setter for it; nothing in this file flips it to True.
- Even when enabled, the value this module returns still has to pass through
  vcu/serial_interface.py's own hard clamp, and then through the real VCU,
  which independently refuses ARM while VCU_OUTPUT_WIRING_VERIFIED=0 (NAK 7).
  This module cannot bypass that -- it only decides what to ask for.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ThrottleState:
    dac_mv: int
    requested_v: float
    enabled: bool
    clamped: bool


class ThrottlePreview:
    def __init__(self, cfg: dict):
        self.enabled = bool(cfg["motor_output_enabled"])
        self.ceiling_v = float(cfg["dac_voltage_limit_v"])
        self.min_v = float(cfg["min_command_v"])
        self.max_v = min(float(cfg["max_command_v"]), self.ceiling_v)
        self.ramp_v_per_s = float(cfg["ramp_v_per_s"])
        self._current_v = 0.0

    def update(self, target_speed_mps: float, dt_s: float, *, armed: bool, stop_requested: bool) -> ThrottleState:
        request_drive = self.enabled and armed and not stop_requested and target_speed_mps > 0
        # No speed->voltage curve exists yet: any positive drive request saturates to the ceiling preview.
        target_v = self.max_v if request_drive else 0.0
        clamped = request_drive

        step = self.ramp_v_per_s * max(dt_s, 0.0)
        if self._current_v < target_v:
            self._current_v = min(target_v, self._current_v + step)
        else:
            self._current_v = max(target_v, self._current_v - step)
        if not request_drive and self._current_v < 1e-9:
            self._current_v = 0.0

        self._current_v = max(0.0, min(self.max_v, self._current_v))
        dac_mv = int(round(self._current_v * 1000))
        return ThrottleState(dac_mv=dac_mv, requested_v=self._current_v,
                              enabled=self.enabled, clamped=clamped)

    def reset(self):
        self._current_v = 0.0
