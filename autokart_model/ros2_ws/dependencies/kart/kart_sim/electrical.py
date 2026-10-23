"""Reduced-order battery, pre-charge, DC bus, and power-domain models.

Battery current is positive for discharge. This convention is used throughout.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .config import BatteryConfig, PowerDomainConfig
from .thermal import ThermalMass


class ContactorState(str, Enum):
    OPEN = "OPEN"
    PRECHARGING = "PRECHARGING"
    CLOSED = "CLOSED"
    FAULT = "FAULT"
    WELDED = "WELDED"


@dataclass
class BatteryFeedback:
    soc: float
    open_circuit_voltage_v: float
    terminal_voltage_v: float
    current_a: float
    power_w: float
    bus_voltage_v: float
    contactor_state: ContactorState
    bms_shutdown: bool
    discharged_energy_j: float
    regenerated_energy_j: float


class BatteryPack:
    def __init__(self, config: BatteryConfig) -> None:
        self.config = config
        self.soc = config.initial_soc
        self.bus_voltage_v = 0.0
        self.contactor_state = ContactorState.PRECHARGING
        self.welded = False
        self.bms_shutdown = False
        self.voltage_scale = 1.0
        self.discharged_energy_j = 0.0
        self.regenerated_energy_j = 0.0
        self.thermal = ThermalMass(45000.0, 0.35, derate_c=45.0, shutdown_c=60.0, restart_c=50.0)
        self.feedback = BatteryFeedback(self.soc, 0.0, 0.0, 0.0, 0.0, 0.0, self.contactor_state, False, 0.0, 0.0)

    def open_circuit_voltage(self) -> float:
        # Provisional linear OCV fallback; replace with a measured monotone table.
        return self.voltage_scale * self.config.nominal_voltage_v * (0.90 + 0.10 * self.soc)

    def request_open(self) -> None:
        if self.welded:
            self.contactor_state = ContactorState.WELDED
        else:
            self.contactor_state = ContactorState.OPEN

    def update(self, requested_power_w: float, dt_s: float, *, close_request: bool = True) -> BatteryFeedback:
        voc = self.open_circuit_voltage()
        if self.bms_shutdown:
            self.contactor_state = ContactorState.OPEN
        elif close_request and self.contactor_state == ContactorState.OPEN:
            self.contactor_state = ContactorState.PRECHARGING
        if self.contactor_state == ContactorState.PRECHARGING:
            tau = self.config.precharge_resistance_ohm * self.config.bus_capacitance_f
            self.bus_voltage_v += (voc - self.bus_voltage_v) * dt_s / tau
            if self.bus_voltage_v >= 0.90 * voc:
                self.contactor_state = ContactorState.CLOSED
        elif self.contactor_state in (ContactorState.CLOSED, ContactorState.WELDED):
            self.bus_voltage_v = voc
        else:
            self.bus_voltage_v = 0.0
        available = self.contactor_state in (ContactorState.CLOSED, ContactorState.WELDED) and not self.bms_shutdown
        requested_current = requested_power_w / max(self.bus_voltage_v, 1.0) if available else 0.0
        current = min(self.config.peak_discharge_current_a, max(-self.config.maximum_charge_current_a, requested_current))
        terminal = max(0.0, voc - current * self.config.internal_resistance_ohm) if available else 0.0
        if terminal < 0.75 * self.config.nominal_voltage_v and current > 0:
            self.bms_shutdown = True
            terminal, current = 0.0, 0.0
        self.soc = min(1.0, max(0.0, self.soc - current * dt_s / (3600.0 * self.config.capacity_ah)))
        power = terminal * current
        if power >= 0:
            self.discharged_energy_j += power * dt_s
        else:
            self.regenerated_energy_j += -power * dt_s
        self.thermal.update(current * current * self.config.internal_resistance_ohm, 25.0, dt_s)
        self.feedback = BatteryFeedback(self.soc, voc, terminal, current, power, self.bus_voltage_v, self.contactor_state, self.bms_shutdown, self.discharged_energy_j, self.regenerated_energy_j)
        return self.feedback


@dataclass
class PowerDomainFeedback:
    voltage_v: float
    current_a: float
    enabled: bool
    brownout: bool


class PowerDomain:
    def __init__(self, config: PowerDomainConfig) -> None:
        self.config = config
        self.feedback = PowerDomainFeedback(0.0, 0.0, False, True)

    def update(self, bus_voltage_v: float, load_power_w: float, enabled: bool = True) -> PowerDomainFeedback:
        ratio = min(1.0, bus_voltage_v / max(1.0, 48.0))
        voltage = self.config.nominal_voltage_v * ratio if enabled else 0.0
        current = min(self.config.maximum_current_a, load_power_w / max(voltage, 0.1)) if enabled else 0.0
        brownout = voltage < self.config.brownout_voltage_v
        self.feedback = PowerDomainFeedback(voltage, current, enabled and not brownout, brownout)
        return self.feedback
