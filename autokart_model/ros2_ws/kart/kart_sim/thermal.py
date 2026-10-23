"""Deterministic first-order thermal masses and continuous derating."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ThermalState:
    temperature_c: float = 25.0
    loss_power_w: float = 0.0
    derating: float = 1.0
    shutdown: bool = False


class ThermalMass:
    def __init__(self, capacitance_j_per_k: float, resistance_k_per_w: float, *, derate_c: float = 90.0, shutdown_c: float = 120.0, restart_c: float = 80.0) -> None:
        if capacitance_j_per_k <= 0 or resistance_k_per_w <= 0 or not restart_c < shutdown_c or not derate_c < shutdown_c:
            raise ValueError("invalid thermal parameters")
        self.capacitance_j_per_k, self.resistance_k_per_w = capacitance_j_per_k, resistance_k_per_w
        self.derate_c, self.shutdown_c, self.restart_c = derate_c, shutdown_c, restart_c
        self.state = ThermalState()

    def update(self, loss_power_w: float, ambient_c: float, dt_s: float) -> ThermalState:
        if dt_s <= 0 or loss_power_w < 0:
            raise ValueError("thermal update requires positive timestep and non-negative loss")
        s = self.state
        cooling_w = (s.temperature_c - ambient_c) / self.resistance_k_per_w
        s.temperature_c += (loss_power_w - cooling_w) * dt_s / self.capacitance_j_per_k
        s.loss_power_w = loss_power_w
        if s.temperature_c >= self.shutdown_c:
            s.shutdown = True
        elif s.shutdown and s.temperature_c <= self.restart_c:
            s.shutdown = False
        s.derating = 0.0 if s.shutdown else max(0.0, min(1.0, (self.shutdown_c - s.temperature_c) / (self.shutdown_c - self.derate_c)))
        return s
