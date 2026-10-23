"""Schema-backed loading of explicit, SI-unit simulation configurations."""

from __future__ import annotations

from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, Field, ValidationError, confloat

Positive = confloat(gt=0)
NonNegative = confloat(ge=0)
UnitInterval = confloat(ge=0, le=1)


class CgConfig(BaseModel):
    x_m: float
    y_m: float = 0.0
    z_m: Positive


class InertiaConfig(BaseModel):
    yaw_kg_m2: Positive


class KartConfig(BaseModel):
    name: str = Field(min_length=1)
    mass_kg: Positive
    wheelbase_m: Positive
    front_track_m: Positive
    rear_track_m: Positive
    # Overall footprint, referenced to the rear axle: rear_overhang_m + front_extent_m == length_m.
    length_m: Positive
    width_m: Positive
    height_m: Positive
    ground_clearance_m: Positive
    rear_overhang_m: Positive
    front_extent_m: Positive
    cg: CgConfig
    inertia: InertiaConfig
    # Software steering clamp actually enforced by the dynamics step (see vehicle.py).
    max_road_wheel_angle_rad: Positive
    # Mechanical end-stop, documentation only: must be >= the software limit above.
    mechanical_max_road_wheel_angle_rad: Positive

    def model_post_init(self, __context: object) -> None:
        if abs(self.rear_overhang_m + self.front_extent_m - self.length_m) > 1e-6:
            raise ValueError("rear_overhang_m + front_extent_m must equal length_m")
        if self.mechanical_max_road_wheel_angle_rad < self.max_road_wheel_angle_rad:
            raise ValueError("mechanical_max_road_wheel_angle_rad must be >= max_road_wheel_angle_rad")


class TireConfig(BaseModel):
    model: Literal["linear_friction_circle"]
    radius_m: Positive
    width_m: Positive = 0.10
    friction_peak: Positive
    cornering_stiffness_n_per_rad: Positive
    longitudinal_stiffness_n: Positive


class MotorConfig(BaseModel):
    max_wheel_force_n: Positive
    torque_constant_nm_per_a: Positive = 0.14
    back_emf_constant_v_per_radps: Positive = 0.14
    phase_resistance_ohm: Positive = 0.08
    rotor_inertia_kg_m2: Positive = 0.012
    maximum_speed_radps: Positive = 500.0
    continuous_current_a: Positive = 45.0
    peak_current_a: Positive = 90.0
    torque_rise_rate_nm_per_s: Positive = 500.0
    torque_fall_rate_nm_per_s: Positive = 800.0
    controller_delay_s: NonNegative = 0.02
    efficiency: confloat(gt=0, le=1) = 0.90


class BrakeConfig(BaseModel):
    max_brake_force_n: Positive


class SteeringActuatorConfig(BaseModel):
    reflected_inertia_kg_m2: Positive = 0.002
    stall_torque_nm: Positive = 3.5
    no_load_speed_radps: Positive = 7.0
    continuous_current_a: Positive = 8.0
    peak_current_a: Positive = 18.0
    winding_resistance_ohm: Positive = 0.18
    torque_constant_nm_per_a: Positive = 0.12
    viscous_damping_nms_per_rad: NonNegative = 0.03
    coulomb_friction_nm: NonNegative = 0.4
    maximum_angle_rad: Positive = 1.5
    maximum_velocity_radps: Positive = 6.0


class SteeringLinkageConfig(BaseModel):
    ratio: Positive = 10.0
    backlash_rad: NonNegative = 0.01
    stiffness_nm_per_rad: Positive = 900.0
    damping_nms_per_rad: NonNegative = 20.0


class SteeringConfig(BaseModel):
    actuator: SteeringActuatorConfig = SteeringActuatorConfig()
    linkage: SteeringLinkageConfig = SteeringLinkageConfig()
    left_limit_rad: Positive = 0.60
    right_limit_rad: float = -0.60


class BrakeActuatorConfig(BaseModel):
    holding_torque_nm: Positive = 3.0
    maximum_speed_radps: Positive = 12.0
    rotor_inertia_kg_m2: Positive = 0.0008
    driver_current_a: Positive = 3.5
    supply_voltage_v: Positive = 24.0
    pulley_radius_m: Positive = 0.025
    efficiency: confloat(gt=0, le=1) = 0.80
    maximum_travel_m: Positive = 0.025


class BrakeHydraulicsConfig(BaseModel):
    pressure_gain_pa_per_m: Positive = 2.0e9
    leakage_rate_per_s: NonNegative = 0.001
    maximum_pressure_pa: Positive = 8.0e6
    pressure_rate_pa_per_s: Positive = 5.0e7
    effective_piston_area_m2: Positive = 0.0012
    effective_radius_m: Positive = 0.09
    pad_friction_coefficient: Positive = 0.40


class BrakeSystemConfig(BaseModel):
    actuator: BrakeActuatorConfig = BrakeActuatorConfig()
    cable_slack_m: NonNegative = 0.002
    cable_stiffness_n_per_m: Positive = 200000.0
    cable_damping_ns_per_m: NonNegative = 600.0
    cable_break_force_n: Positive = 4000.0
    hydraulics: BrakeHydraulicsConfig = BrakeHydraulicsConfig()


class RearAxleConfig(BaseModel):
    type: Literal["solid"] = "solid"
    inertia_kg_m2: Positive = 0.12
    torsional_stiffness_nm_per_rad: Positive = 5000.0
    bearing_loss_nm: NonNegative = 0.2


class ChassisConfig(BaseModel):
    torsional_stiffness_nm_per_rad: Positive = 1500.0
    torsional_damping_nms_per_rad: NonNegative = 80.0


class BatteryConfig(BaseModel):
    nominal_voltage_v: Positive = 48.0
    capacity_ah: Positive = 30.0
    initial_soc: UnitInterval = 0.90
    internal_resistance_ohm: NonNegative = 0.035
    continuous_discharge_current_a: Positive = 80.0
    peak_discharge_current_a: Positive = 150.0
    maximum_charge_current_a: Positive = 40.0
    precharge_resistance_ohm: Positive = 10.0
    bus_capacitance_f: Positive = 0.005

    @classmethod
    def _limits_valid(cls, values: dict) -> dict:
        return values


class PowerDomainConfig(BaseModel):
    nominal_voltage_v: Positive
    brownout_voltage_v: Positive
    maximum_current_a: Positive = 20.0

    def model_post_init(self, __context: object) -> None:
        if self.brownout_voltage_v >= self.nominal_voltage_v:
            raise ValueError("brownout voltage must be below nominal voltage")


class PowerDomainsConfig(BaseModel):
    steering: PowerDomainConfig = PowerDomainConfig(nominal_voltage_v=24.0, brownout_voltage_v=19.0)
    brake: PowerDomainConfig = PowerDomainConfig(nominal_voltage_v=24.0, brownout_voltage_v=19.0)
    vcu: PowerDomainConfig = PowerDomainConfig(nominal_voltage_v=5.0, brownout_voltage_v=4.5)
    compute: PowerDomainConfig = PowerDomainConfig(nominal_voltage_v=12.0, brownout_voltage_v=10.5)


class EnvironmentConfig(BaseModel):
    rolling_resistance_coefficient: NonNegative
    air_density_kg_per_m3: Positive
    drag_coefficient_area_m2: NonNegative
    grade_rad: float = 0.0
    surface_friction_scale: Positive = 1.0


class SteeringLagConfig(BaseModel):
    """Simple first-order lag + slew-rate limit on the steering command, applied in
    PlanarBicycleKart.step() ahead of the tyre-force calculation. Defaults are
    effectively instantaneous so existing configs that omit this block are unaffected."""

    rate_limit_radps: Positive = 1000.0
    lag_s: NonNegative = 0.0


class BrakeLagConfig(BaseModel):
    """Dead-time + ramp-rate on the brake command, applied in PlanarBicycleKart.step()
    ahead of the tyre-force calculation. Defaults are effectively instantaneous so
    existing configs that omit this block are unaffected."""

    dead_time_s: NonNegative = 0.0
    ramp_rate_per_s: Positive = 1000.0


class ActuatorLagConfig(BaseModel):
    steering: SteeringLagConfig = SteeringLagConfig()
    brake: BrakeLagConfig = BrakeLagConfig()


class SimulationConfig(BaseModel):
    physics_dt_s: confloat(gt=0, le=0.02)
    seed: int = 1
    vehicle_backend: Literal["planar_bicycle", "four_wheel", "dynamic_four_wheel"] = "planar_bicycle"


class FullKartConfig(BaseModel):
    kart: KartConfig
    tires: TireConfig
    motor: MotorConfig
    brake: BrakeConfig
    steering: SteeringConfig = SteeringConfig()
    brake_system: BrakeSystemConfig = BrakeSystemConfig()
    actuator_lag: ActuatorLagConfig = ActuatorLagConfig()
    battery: BatteryConfig = BatteryConfig()
    power_domains: PowerDomainsConfig = PowerDomainsConfig()
    rear_axle: RearAxleConfig = RearAxleConfig()
    chassis: ChassisConfig = ChassisConfig()
    environment: EnvironmentConfig
    simulation: SimulationConfig


def load_yaml(path: str | Path) -> dict:
    with Path(path).open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError(f"{path}: configuration root must be a mapping")
    return document


def load_kart_config(path: str | Path) -> FullKartConfig:
    """Read and validate a kart configuration, preserving Pydantic error detail."""
    try:
        payload = load_yaml(path)
        config = FullKartConfig.model_validate(payload) if hasattr(FullKartConfig, "model_validate") else FullKartConfig.parse_obj(payload)
        if not (-config.kart.wheelbase_m / 2 <= config.kart.cg.x_m <= config.kart.wheelbase_m / 2):
            raise ValueError("cg.x_m must lie within the wheelbase measured from its midpoint")
        return config
    except ValidationError as exc:
        raise ValueError(f"invalid kart configuration {path}: {exc}") from exc
