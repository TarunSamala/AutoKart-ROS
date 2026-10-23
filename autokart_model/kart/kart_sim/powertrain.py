"""Reduced-order BLDC motor controller and chain transmission."""
from __future__ import annotations

from dataclasses import asdict, dataclass

from .config import MotorConfig
from .thermal import ThermalMass


@dataclass
class MotorFeedback:
    requested_torque_nm: float = 0.0
    available_torque_nm: float = 0.0
    actual_torque_nm: float = 0.0
    motor_speed_radps: float = 0.0
    motor_current_a: float = 0.0
    bus_current_a: float = 0.0
    electrical_power_w: float = 0.0
    mechanical_power_w: float = 0.0
    chain_torque_nm: float = 0.0
    chain_tension_n: float = 0.0
    motor_temperature_c: float = 25.0
    controller_temperature_c: float = 25.0
    fault: str | None = None


class BldcPowertrain:
    GEAR_RATIO = 5.0

    def __init__(self, config: MotorConfig) -> None:
        self.config, self.feedback = config, MotorFeedback()
        self.motor_thermal = ThermalMass(3500.0, 0.45)
        self.controller_thermal = ThermalMass(1800.0, 0.6)
        self.fault: str | None = None

    def update(self, request_fraction: float, axle_speed_radps: float, bus_voltage_v: float, dt_s: float, *, enabled: bool = True) -> MotorFeedback:
        c, f = self.config, self.feedback
        f.requested_torque_nm = max(-1.0, min(1.0, request_fraction)) * c.peak_current_a * c.torque_constant_nm_per_a
        f.motor_speed_radps = axle_speed_radps * self.GEAR_RATIO
        voltage_limit_current = max(0.0, (bus_voltage_v - c.back_emf_constant_v_per_radps * abs(f.motor_speed_radps)) / c.phase_resistance_ohm)
        current_limit = min(c.peak_current_a, voltage_limit_current)
        f.available_torque_nm = current_limit * c.torque_constant_nm_per_a * self.motor_thermal.state.derating * self.controller_thermal.state.derating
        target = max(-f.available_torque_nm, min(f.available_torque_nm, f.requested_torque_nm))
        if not enabled or self.fault in ("torque_loss", "controller_disabled", "chain_break") or self.motor_thermal.state.shutdown or self.controller_thermal.state.shutdown:
            target = 0.0
        if self.fault == "torque_stuck_positive" and enabled: target = abs(f.available_torque_nm)
        if self.fault == "wrong_sign": target = -target
        slew = c.torque_rise_rate_nm_per_s if abs(target) > abs(f.actual_torque_nm) else c.torque_fall_rate_nm_per_s
        f.actual_torque_nm += max(-slew * dt_s, min(slew * dt_s, target - f.actual_torque_nm))
        f.motor_current_a = abs(f.actual_torque_nm) / c.torque_constant_nm_per_a
        f.mechanical_power_w = f.actual_torque_nm * f.motor_speed_radps
        copper_w = f.motor_current_a**2 * c.phase_resistance_ohm
        f.electrical_power_w = max(0.0, f.mechanical_power_w / c.efficiency) + copper_w
        f.bus_current_a = f.electrical_power_w / max(bus_voltage_v, 1.0)
        f.chain_torque_nm = 0.0 if self.fault == "chain_break" else f.actual_torque_nm * self.GEAR_RATIO * c.efficiency
        f.chain_tension_n = abs(f.chain_torque_nm) / 0.04
        f.motor_temperature_c = self.motor_thermal.update(copper_w, 25.0, dt_s).temperature_c
        f.controller_temperature_c = self.controller_thermal.update(0.03 * f.electrical_power_w, 25.0, dt_s).temperature_c
        f.fault = self.fault
        return f

    def telemetry(self) -> dict: return asdict(self.feedback)
