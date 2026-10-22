"""Phase-4 integration wrapper: commands drive dynamic subsystems, never tire forces directly."""
from __future__ import annotations

from dataclasses import asdict

from .actuators import BrakeSystem, SteeringSystem
from .config import FullKartConfig
from .electrical import BatteryPack, PowerDomain
from .four_wheel import FourWheelKart
from .powertrain import BldcPowertrain
from .types import PlanarKartState, VehicleCommand


class DynamicFourWheelKart:
    def __init__(self, config: FullKartConfig, state: PlanarKartState | None = None) -> None:
        self.config = config
        self.vehicle = FourWheelKart(config, state)
        self.state, self.wheels = self.vehicle.state, self.vehicle.wheels
        self.steering, self.brakes, self.motor = SteeringSystem(config.steering), BrakeSystem(config.brake_system), BldcPowertrain(config.motor)
        self.battery = BatteryPack(config.battery)
        self.steering_domain = PowerDomain(config.power_domains.steering)
        self.brake_domain = PowerDomain(config.power_domains.brake)
        self.last_forces = self.vehicle.last_forces
        self.faults: set[str] = set()

    def inject_fault(self, target: str) -> None:
        self.faults.add(target)
        if target.startswith("steering:"): self.steering.fault = target.split(":", 1)[1]
        elif target.startswith("brake:"): self.brakes.fault = target.split(":", 1)[1]
        elif target.startswith("motor:"): self.motor.fault = target.split(":", 1)[1]
        elif target == "battery:bms_shutdown": self.battery.bms_shutdown = True
        elif target == "battery:undervoltage": self.battery.voltage_scale = 0.60
        elif target == "battery:contactor_welded": self.battery.welded = True
        if target == "motor:overtemperature":
            self.motor.fault = None
            self.motor.motor_thermal.state.temperature_c = 130.0
            self.motor.motor_thermal.state.shutdown = True
        if target == "controller:overtemperature":
            self.motor.controller_thermal.state.temperature_c = 130.0
            self.motor.controller_thermal.state.shutdown = True

    def step(self, command: VehicleCommand, dt_s: float | None = None, *, motor_enable: bool = True, contactor_open: bool = False) -> PlanarKartState:
        command.validated(); dt = self.config.simulation.physics_dt_s if dt_s is None else dt_s
        steering_power = self.steering.feedback.current_a * max(self.steering.feedback.supply_voltage_v, 0.0)
        brake_power = self.brakes.feedback.current_a * self.config.brake_system.actuator.supply_voltage_v
        steering_supply = self.steering_domain.update(self.battery.feedback.bus_voltage_v, steering_power)
        brake_supply = self.brake_domain.update(self.battery.feedback.bus_voltage_v, brake_power)
        sf = self.steering.update(command.steering_rad, steering_supply, dt)
        bf = self.brakes.update(command.brake, brake_supply, self.vehicle.rear_axle_angular_speed_radps, dt)
        if contactor_open: self.battery.request_open()
        mf = self.motor.update(command.drive, self.vehicle.rear_axle_angular_speed_radps, self.battery.feedback.bus_voltage_v, dt, enabled=motor_enable and not self.battery.bms_shutdown)
        # Phase-3 backend receives achieved mechanical outputs, scaled at its established force/torque boundary.
        drive_fraction = mf.chain_torque_nm / (self.config.motor.max_wheel_force_n * self.config.tires.radius_m)
        brake_fraction = bf.brake_torque_nm / (self.config.brake.max_brake_force_n * self.config.tires.radius_m)
        state = self.vehicle.step(VehicleCommand(drive=max(-1.0, min(1.0, drive_fraction)), brake=max(0.0, min(1.0, brake_fraction)), steering_rad=sf.linkage_angle_rad), dt)
        self.state, self.wheels, self.last_forces = state, self.vehicle.wheels, self.vehicle.last_forces
        aux_power = steering_power + brake_power + 15.0  # VCU/sensors provisional load, separately accounted in future domain detail.
        self.battery.update(mf.electrical_power_w + aux_power, dt)
        return state

    def telemetry(self) -> dict:
        return {"four_wheel": self.vehicle.telemetry(), "steering": self.steering.telemetry(), "brake": self.brakes.telemetry(), "motor": self.motor.telemetry(), "battery": asdict(self.battery.feedback), "power_domains": {"steering": asdict(self.steering_domain.feedback), "brake": asdict(self.brake_domain.feedback)}, "faults": sorted(self.faults)}
