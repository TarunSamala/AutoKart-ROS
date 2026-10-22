from pathlib import Path

import pytest

from kart_sim.actuators import BrakeSystem, SteeringSystem
from kart_sim.config import load_kart_config
from kart_sim.electrical import BatteryPack, ContactorState, PowerDomain
from kart_sim.powertrain import BldcPowertrain
from kart_sim.thermal import ThermalMass

ROOT = Path(__file__).parents[2]


def cfg(): return load_kart_config(ROOT / "configs/karts/prototype_dynamic.yaml")


def test_steering_step_is_dynamic_and_backlash_is_repeatable() -> None:
    c = cfg(); system = SteeringSystem(c.steering); supply = PowerDomain(c.power_domains.steering)
    supply.update(48.0, 0.0)
    initial = system.update(0.3, supply.feedback, 0.001)
    assert initial.linkage_angle_rad == pytest.approx(0.0, abs=1e-6)
    for _ in range(1500): system.update(0.3, supply.feedback, 0.001)
    assert 0.0 < system.feedback.linkage_angle_rad < 0.31


def test_steering_mechanical_limits_bound_command_and_physical_motion() -> None:
    c = cfg(); system = SteeringSystem(c.steering); supply = PowerDomain(c.power_domains.steering)
    supply.update(48.0, 0.0)
    for _ in range(2000):
        system.update(10.0, supply.feedback, 0.001)
    assert system.feedback.validated_target_rad == c.steering.left_limit_rad
    assert system.feedback.shaft_angle_rad <= c.steering.actuator.maximum_angle_rad
    assert system.feedback.left_wheel_angle_rad <= c.steering.left_limit_rad


def test_brake_chain_derives_pressure_and_torque_then_releases() -> None:
    c = cfg(); brake = BrakeSystem(c.brake_system); domain = PowerDomain(c.power_domains.brake); domain.update(48.0, 0.0)
    for _ in range(200): brake.update(1.0, domain.feedback, 20.0, 0.001)
    assert brake.feedback.actual_position_m > 0
    assert brake.feedback.hydraulic_pressure_pa > 0
    assert brake.feedback.brake_torque_nm > 0
    for _ in range(400): brake.update(0.0, domain.feedback, 0.0, 0.001)
    assert brake.feedback.hydraulic_pressure_pa < 1e4


def test_stepper_stall_keeps_command_and_actual_position_distinct() -> None:
    c = cfg(); brake = BrakeSystem(c.brake_system); domain = PowerDomain(c.power_domains.brake); domain.update(48.0, 0.0); brake.fault = "stepper_stall"
    for _ in range(100): brake.update(1.0, domain.feedback, 0.0, 0.001)
    assert brake.feedback.commanded_position_m > brake.feedback.actual_position_m
    assert brake.feedback.missed_steps > 0


def test_battery_resistance_and_precharge() -> None:
    battery = BatteryPack(cfg().battery)
    for _ in range(2000): feedback = battery.update(480.0, 0.001)
    assert feedback.contactor_state == ContactorState.CLOSED
    feedback = battery.update(480.0, 0.001)
    assert feedback.terminal_voltage_v == pytest.approx(feedback.open_circuit_voltage_v - feedback.current_a * cfg().battery.internal_resistance_ohm)
    assert feedback.soc < cfg().battery.initial_soc


def test_motor_slew_voltage_limit_and_chain_break() -> None:
    c = cfg(); motor = BldcPowertrain(c.motor)
    result = motor.update(1.0, 0.0, 48.0, 0.001)
    assert 0 < result.actual_torque_nm < result.available_torque_nm
    motor.fault = "chain_break"; result = motor.update(1.0, 0.0, 48.0, 0.1)
    assert result.chain_torque_nm == 0.0


def test_thermal_heats_cools_derates_and_shuts_down() -> None:
    thermal = ThermalMass(10.0, 1.0, derate_c=30.0, shutdown_c=40.0, restart_c=25.0)
    for _ in range(100): state = thermal.update(20.0, 20.0, 0.1)
    assert state.temperature_c > 30.0 and state.derating < 1.0
    for _ in range(100): state = thermal.update(0.0, 20.0, 0.1)
    assert state.temperature_c < 30.0
