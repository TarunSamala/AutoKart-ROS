"""Dynamic steering and brake command-to-output chains with deterministic faults."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from math import copysign, tanh

from .config import BrakeSystemConfig, SteeringConfig
from .electrical import PowerDomainFeedback
from .thermal import ThermalMass


@dataclass
class SteeringFeedback:
    requested_angle_rad: float = 0.0
    validated_target_rad: float = 0.0
    shaft_angle_rad: float = 0.0
    shaft_velocity_radps: float = 0.0
    linkage_angle_rad: float = 0.0
    left_wheel_angle_rad: float = 0.0
    right_wheel_angle_rad: float = 0.0
    current_a: float = 0.0
    supply_voltage_v: float = 0.0
    following_error_rad: float = 0.0
    temperature_c: float = 25.0
    fault: str | None = None


class SteeringSystem:
    def __init__(self, config: SteeringConfig) -> None:
        self.config, self.feedback = config, SteeringFeedback()
        self.thermal = ThermalMass(900.0, 1.2)
        self.fault: str | None = None
        self._backlash_reference = 0.0

    def update(self, target_rad: float, supply: PowerDomainFeedback, dt_s: float, tire_load_torque_nm: float = 0.0) -> SteeringFeedback:
        c, a, l = self.config, self.config.actuator, self.config.linkage
        f = self.feedback
        f.requested_angle_rad = target_rad
        f.validated_target_rad = max(c.right_limit_rad, min(c.left_limit_rad, target_rad))
        target_shaft = max(-a.maximum_angle_rad, min(a.maximum_angle_rad, f.validated_target_rad * l.ratio))
        voltage_scale = min(1.0, supply.voltage_v / 24.0) if supply.enabled else 0.0
        available_torque = a.stall_torque_nm * voltage_scale * max(0.0, 1.0 - abs(f.shaft_velocity_radps) / a.no_load_speed_radps) * self.thermal.state.derating
        if self.fault in ("power_loss", "jam"):
            available_torque = 0.0 if self.fault == "power_loss" else available_torque
        error = target_shaft - f.shaft_angle_rad
        command_torque = max(-available_torque, min(available_torque, 8.0 * error - 0.8 * f.shaft_velocity_radps))
        friction = a.coulomb_friction_nm * tanh(f.shaft_velocity_radps / 0.02) + a.viscous_damping_nms_per_rad * f.shaft_velocity_radps
        if self.fault == "jam":
            shaft_accel = 0.0
        elif self.fault == "runaway":
            # Physical actuator-controller runaway: shaft motion develops without
            # a matching VCU target. The VCU detects only measured motion.
            shaft_accel = 30.0
        else:
            shaft_accel = (command_torque - tire_load_torque_nm - friction) / a.reflected_inertia_kg_m2
        f.shaft_velocity_radps += shaft_accel * dt_s
        f.shaft_velocity_radps = max(-a.maximum_velocity_radps, min(a.maximum_velocity_radps, f.shaft_velocity_radps))
        f.shaft_angle_rad += f.shaft_velocity_radps * dt_s
        input_road = f.shaft_angle_rad / l.ratio
        difference = input_road - self._backlash_reference
        if abs(difference) > l.backlash_rad:
            self._backlash_reference += difference - copysign(l.backlash_rad, difference)
        if self.fault != "linkage_break":
            f.linkage_angle_rad += (self._backlash_reference - f.linkage_angle_rad) * min(1.0, l.stiffness_nm_per_rad * dt_s / 30.0)
        if self.fault != "road_wheel_jam":
            f.left_wheel_angle_rad = max(c.right_limit_rad, min(c.left_limit_rad, f.linkage_angle_rad))
            f.right_wheel_angle_rad = f.left_wheel_angle_rad
        f.following_error_rad = f.validated_target_rad - f.linkage_angle_rad
        f.current_a = min(a.peak_current_a, abs(command_torque) / a.torque_constant_nm_per_a)
        f.supply_voltage_v, f.fault = supply.voltage_v, self.fault
        f.temperature_c = self.thermal.update(f.current_a * f.current_a * a.winding_resistance_ohm, 25.0, dt_s).temperature_c
        return f

    def telemetry(self) -> dict: return asdict(self.feedback)


@dataclass
class BrakeFeedback:
    requested_effort: float = 0.0
    commanded_position_m: float = 0.0
    actual_position_m: float = 0.0
    cable_tension_n: float = 0.0
    hydraulic_pressure_pa: float = 0.0
    brake_torque_nm: float = 0.0
    missed_steps: float = 0.0
    current_a: float = 0.0
    temperature_c: float = 25.0
    fault: str | None = None


class BrakeSystem:
    def __init__(self, config: BrakeSystemConfig) -> None:
        self.config, self.feedback = config, BrakeFeedback()
        self.thermal = ThermalMass(1200.0, 1.0, derate_c=100.0, shutdown_c=160.0, restart_c=90.0)
        self.disc_thermal = ThermalMass(9000.0, 0.08, derate_c=350.0, shutdown_c=600.0, restart_c=300.0)
        self.fault: str | None = None

    def update(self, effort: float, supply: PowerDomainFeedback, wheel_speed_radps: float, dt_s: float) -> BrakeFeedback:
        c, a, h, f = self.config, self.config.actuator, self.config.hydraulics, self.feedback
        f.requested_effort = max(0.0, min(1.0, effort))
        f.commanded_position_m = f.requested_effort * a.maximum_travel_m
        speed = a.maximum_speed_radps * a.pulley_radius_m * min(1.0, supply.voltage_v / a.supply_voltage_v) if supply.enabled else 0.0
        if self.fault in ("stepper_stall", "power_loss"):
            speed = 0.0
            f.missed_steps += abs(f.commanded_position_m - f.actual_position_m) * 20000.0 * dt_s
        delta = max(-speed * dt_s, min(speed * dt_s, f.commanded_position_m - f.actual_position_m))
        f.actual_position_m += delta
        cable_displacement = max(0.0, f.actual_position_m - c.cable_slack_m)
        transmission_scale = 0.25 if self.fault == "cable_partial_25" else 1.0
        f.cable_tension_n = (
            0.0
            if self.fault == "cable_break"
            else transmission_scale * c.cable_stiffness_n_per_m * cable_displacement
        )
        pressure_target = transmission_scale * min(
            h.maximum_pressure_pa,
            h.pressure_gain_pa_per_m * cable_displacement,
        )
        # A broken cable leaves actuator-side travel but transmits no master-cylinder force.
        if self.fault in ("hydraulic_leak", "cable_break"): pressure_target = 0.0
        if self.fault == "not_releasing" and f.requested_effort < 0.05 and f.hydraulic_pressure_pa > 0:
            pressure_target = f.hydraulic_pressure_pa
        rate = h.pressure_rate_pa_per_s * dt_s
        f.hydraulic_pressure_pa += max(-rate, min(rate, pressure_target - f.hydraulic_pressure_pa))
        f.hydraulic_pressure_pa *= max(0.0, 1.0 - h.leakage_rate_per_s * dt_s)
        friction_scale = 0.25 if self.fault in ("pad_friction_reduced", "fade") else 1.0
        f.brake_torque_nm = f.hydraulic_pressure_pa * h.effective_piston_area_m2 * h.pad_friction_coefficient * h.effective_radius_m * friction_scale
        f.current_a = a.driver_current_a if abs(delta) > 1e-12 else 0.2
        f.temperature_c = self.thermal.update(f.current_a * f.current_a * 1.0, 25.0, dt_s).temperature_c
        self.disc_thermal.update(abs(f.brake_torque_nm * wheel_speed_radps), 25.0, dt_s)
        f.fault = self.fault
        return f

    def telemetry(self) -> dict:
        output = asdict(self.feedback); output["disc_temperature_c"] = self.disc_thermal.state.temperature_c; return output
