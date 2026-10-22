"""Minimal deterministic scenario runner and JSONL telemetry writer."""

from __future__ import annotations

import json
from dataclasses import asdict
from math import atan, cos, sin
from pathlib import Path
from time import sleep
from typing import Callable

from pydantic import BaseModel, Field, confloat

from .config import load_kart_config, load_yaml
from .scheduler import FixedStepScheduler
from .types import PlanarKartState, VehicleCommand
from .vehicle import PlanarBicycleKart
from .four_wheel import FourWheelKart
from .dynamic_kart import DynamicFourWheelKart
from .electrical import ContactorState
from .vcu import AutonomousGuards, VCU, VcuInputs, MotionCommand, SafetyEnvelope, VcuState
from .track import Track


class CommandConfig(BaseModel):
    drive: float = 0.0
    brake: confloat(ge=0, le=1) = 0.0
    steering_rad: float = 0.0


class ScenarioConfig(BaseModel):
    name: str
    kart_config: str
    duration_s: confloat(gt=0)
    command: CommandConfig = CommandConfig()
    initial_state: dict[str, float] = {}
    control: dict = {"mode": "direct_test"}
    track_config: str | None = None
    command_schedule: list[dict[str, float]] = Field(default_factory=list)


def run_scenario(
    path: str | Path,
    output_dir: str | Path = "runs",
    *,
    realtime: bool = False,
    record_callback: Callable[[dict], None] | None = None,
) -> Path:
    source = Path(path)
    payload = load_yaml(source)
    scenario = ScenarioConfig.model_validate(payload) if hasattr(ScenarioConfig, "model_validate") else ScenarioConfig.parse_obj(payload)
    configuration = load_kart_config(source.parent.parent / scenario.kart_config)
    track_model = Track.from_yaml(source.parent.parent / scenario.track_config) if scenario.track_config else None
    if "surface_friction" in scenario.control:
        configuration.environment.surface_friction_scale = float(scenario.control["surface_friction"])
    if "grade_rad" in scenario.control:
        configuration.environment.grade_rad = float(scenario.control["grade_rad"])
    initial_state = PlanarKartState(**scenario.initial_state)
    if configuration.simulation.vehicle_backend == "dynamic_four_wheel":
        kart = DynamicFourWheelKart(configuration, initial_state)
    else:
        kart = FourWheelKart(configuration, initial_state) if configuration.simulation.vehicle_backend == "four_wheel" else PlanarBicycleKart(configuration, initial_state)
    wheel_friction = scenario.control.get("wheel_friction_scales")
    if wheel_friction and isinstance(kart, DynamicFourWheelKart):
        for wheel_id, scale in dict(wheel_friction).items():
            if wheel_id not in kart.vehicle.surface_friction_scales or float(scale) <= 0:
                raise ValueError("invalid wheel friction scale")
            kart.vehicle.surface_friction_scales[wheel_id] = float(scale)
    scheduler = FixedStepScheduler(kart.config.simulation.physics_dt_s)
    command = VehicleCommand(drive=scenario.command.drive, brake=scenario.command.brake, steering_rad=scenario.command.steering_rad).validated()
    command_schedule = sorted(scenario.command_schedule, key=lambda item: float(item.get("time_s", 0.0)))

    def command_at(time_s: float) -> VehicleCommand:
        values = {"drive": command.drive, "brake": command.brake, "steering_rad": command.steering_rad}
        for item in command_schedule:
            if time_s + 1e-12 < float(item.get("time_s", 0.0)):
                break
            for key in values:
                if key in item:
                    values[key] = float(item[key])
        return VehicleCommand(**values).validated()
    mode = scenario.control.get("mode", "direct_test")
    if mode not in ("direct_test", "vcu"): raise ValueError("control.mode must be direct_test or vcu")
    if mode == "vcu" and any(key in scenario.control for key in ("direct_actuator_command", "actuator_override")):
        raise ValueError("direct actuator writes are prohibited in VCU mode")
    if mode == "direct_test" and bool(scenario.control.get("autonomous", False)):
        raise ValueError("autonomous scenarios must use VCU mode")
    if mode == "vcu" and not isinstance(kart, DynamicFourWheelKart): raise ValueError("VCU mode requires dynamic_four_wheel backend")
    vcu = VCU() if mode == "vcu" else None
    if isinstance(kart, DynamicFourWheelKart) and scenario.control.get("inject_fault"):
        kart.inject_fault(str(scenario.control["inject_fault"]))
    if isinstance(kart, DynamicFourWheelKart) and scenario.control.get("secondary_fault"):
        kart.inject_fault(str(scenario.control["secondary_fault"]))
    if vcu:
        for state in (VcuState.BOOT,VcuState.SELF_TEST,VcuState.SAFE,VcuState.READY): assert vcu.transition(state)
        assert vcu.request_autonomous(AutonomousGuards())
        vcu.controlled_stop.timeout_s = float(scenario.control.get("controlled_stop_timeout_s", 5.0))
    vcu_rate = float(scenario.control.get("vcu_rate_hz", 250.0)); vcu_period=1/vcu_rate; next_vcu=0.0; seq=0; final_command=command; vcu_output=None
    records: list[dict] = []

    def track_feedback() -> tuple[dict | None, float, float]:
        """Project the physical state and wheel contacts into the selected track.

        The Phase-5 surface lookup is the authority for per-wheel friction in a
        track-backed scenario. Dashboard consumers only receive the resulting
        read-only trace.
        """
        if track_model is None:
            return None, float(scenario.control.get("surface_friction", 1.0)), 0.0
        projection = track_model.project(kart.state.x_m, kart.state.y_m, kart.state.z_m)
        geometry = track_model.sample(projection.s_m)
        kart.state.z_m = geometry.z_m
        kart.state.pitch_rad = atan(geometry.grade)
        kart.state.roll_rad = geometry.bank_rad
        wheel_surfaces: dict[str, dict] = {}
        minimum_friction = float("inf")
        if isinstance(kart, DynamicFourWheelKart):
            half_wheelbase = configuration.kart.wheelbase_m / 2.0
            offsets = {
                "front_left": (half_wheelbase, configuration.kart.front_track_m / 2.0),
                "front_right": (half_wheelbase, -configuration.kart.front_track_m / 2.0),
                "rear_left": (-half_wheelbase, configuration.kart.rear_track_m / 2.0),
                "rear_right": (-half_wheelbase, -configuration.kart.rear_track_m / 2.0),
            }
            for wheel_id, (longitudinal, lateral) in offsets.items():
                wx = kart.state.x_m + longitudinal * cos(kart.state.yaw_rad) - lateral * sin(kart.state.yaw_rad)
                wy = kart.state.y_m + longitudinal * sin(kart.state.yaw_rad) + lateral * cos(kart.state.yaw_rad)
                wheel_projection = track_model.project(wx, wy, kart.state.z_m, projection.s_m)
                lookup = track_model.surface_at(wheel_projection.s_m, wheel_projection.lateral_m)
                friction = lookup.material.friction_longitudinal
                minimum_friction = min(minimum_friction, friction)
                kart.vehicle.surface_friction_scales[wheel_id] = friction
                wheel_surfaces[wheel_id] = {
                    "s_m": wheel_projection.s_m,
                    "lateral_m": wheel_projection.lateral_m,
                    "material_id": lookup.material.id,
                    "friction_longitudinal": friction,
                    "friction_lateral": lookup.material.friction_lateral,
                    "surface_height_m": lookup.surface_height_m,
                }
        return {
            "name": track_model.name,
            "projection": asdict(projection),
            "geometry": asdict(geometry),
            "wheel_surfaces": wheel_surfaces,
        }, minimum_friction if minimum_friction != float("inf") else 1.0, geometry.curvature_1pm

    def tick(time_s: float, dt_s: float) -> None:
        nonlocal next_vcu,seq,final_command,vcu_output
        current_track, track_friction, track_curvature = track_feedback()
        current_command = command_at(time_s)
        if vcu and time_s >= float(scenario.control.get("physical_estop_time_s", 1e99)):
            vcu.independent.estop = True
        if vcu and time_s >= float(scenario.control.get("safety_task_freeze_time_s", 1e99)):
            vcu.independent.trip("SAFETY_TASK_WATCHDOG")
        frozen = bool(vcu and time_s >= float(scenario.control.get("vcu_freeze_time_s", 1e99)))
        if vcu and not frozen and time_s + 1e-12 >= next_vcu:
            vcu.independent.pulse(time_s); seq+=1
            renew_until=float(scenario.control.get("command_stop_time_s", scenario.duration_s))
            software_estop=bool(scenario.control.get("estop",False)) and time_s>=float(scenario.control.get("estop_start_time_s",0.0))
            scheduled_brake=0.0 if time_s>=float(scenario.control.get("command_brake_release_time_s",1e99)) else current_command.brake
            scheduled_drive=current_command.drive if time_s>=float(scenario.control.get("command_drive_start_time_s",0.0)) else 0.0
            high=MotionCommand("scenario",seq,time_s,time_s+float(scenario.control.get("command_timeout_s",.15)),scheduled_drive*8,current_command.steering_rad,scheduled_brake,software_estop).signed() if time_s <= renew_until else None
            wheel_speeds=tuple(w.angular_speed_radps*configuration.tires.radius_m for w in kart.wheels.values())
            wheel_mode=str(scenario.control.get("wheel_feedback_fault","normal"))
            if wheel_mode in ("one_false_zero","all_false_zero"):
                wheel_speeds=(0.0,*wheel_speeds[1:]) if wheel_mode=="one_false_zero" else tuple(0.0 for _ in wheel_speeds)
            steering_primary=kart.steering.feedback.linkage_angle_rad
            steering_secondary=kart.steering.feedback.left_wheel_angle_rad
            steering_sensor_fault=str(scenario.control.get("steering_feedback_fault","normal"))
            if steering_sensor_fault=="primary_frozen_zero": steering_primary=0.0
            if steering_sensor_fault=="secondary_drift": steering_secondary+=0.1
            if steering_sensor_fault=="primary_offset_jump" and time_s>=float(scenario.control.get("steering_feedback_fault_time_s",0.2)): steering_primary+=0.2
            pressure=kart.brakes.feedback.hydraulic_pressure_pa
            position=kart.brakes.feedback.actual_position_m
            brake_sensor_fault=str(scenario.control.get("brake_feedback_fault","normal"))
            if brake_sensor_fault=="pressure_frozen_zero": pressure=0.0
            if brake_sensor_fault=="position_frozen_zero": position=0.0
            motor_current=kart.motor.feedback.motor_current_a
            motor_feedback_fault=str(scenario.control.get("motor_feedback_fault","normal"))
            if motor_feedback_fault=="current_bias": motor_current+=50.0
            inputs=VcuInputs(
                time_s,high,kart.state.u_mps,steering_primary,
                pressure,kart.motor.feedback.actual_torque_nm,
                position,vcu.independent.permit(time_s),
                wheel_speeds,wheel_mode!="all_invalid",motor_current,
                True,True,steering_sensor_fault!="primary_frozen_zero",
                kart.battery.feedback.contactor_state in (ContactorState.CLOSED,ContactorState.WELDED),time_s,
                steering_secondary,kart.steering.feedback.current_a,
                kart.brakes.feedback.brake_torque_nm,kart.state.ax_mps2,
                float(scenario.control.get("surface_friction",1.0)),
                kart.motor.feedback.chain_torque_nm,
                motor_feedback_fault!="motor_speed_lost",
                kart.steering.feedback.shaft_angle_rad,
            )
            friction=min(float(scenario.control.get("surface_friction",1.0)), track_friction)
            envelope = SafetyEnvelope(
                clear_distance_m=float(scenario.control.get("clear_distance_m",100)),
                guaranteed_decel_mps2=max(0.3,3.0*friction),
                curvature_1pm=track_curvature,
            )
            vcu_output=vcu.cycle(inputs,envelope)
            final_command=vcu_output.command; next_vcu += vcu_period
        # Independent safety task executes every physics tick and overrides stale/frozen VCU outputs.
        independent_override = bool(vcu and not vcu.independent.permit(time_s))
        if independent_override:
            final_command = VehicleCommand(brake=1.0, steering_rad=kart.steering.feedback.linkage_angle_rad)
            state = kart.step(final_command, dt_s, motor_enable=False, contactor_open=True)
        else:
            state = kart.step(final_command, dt_s, motor_enable=vcu_output.motor_enable, contactor_open=vcu_output.contactor_open) if vcu else kart.step(current_command, dt_s)
        record = {"state": asdict(state), "forces": asdict(kart.last_forces)}
        if hasattr(kart, "telemetry"):
            record["subsystems"] = kart.telemetry()
        updated_track, track_friction, track_curvature = track_feedback()
        if updated_track:
            record["track"] = updated_track
            record["track_s_m"] = updated_track["projection"]["s_m"]
            record["track_lateral_m"] = updated_track["projection"]["lateral_m"]
        requested = asdict(current_command)
        record["commands"] = {
            "requested": requested,
            "validated": requested,
            "safety_limited": asdict(vcu_output.command) if vcu_output else requested,
            "final": asdict(final_command),
        }
        if vcu_output:
            envelope = SafetyEnvelope(
                clear_distance_m=float(scenario.control.get("clear_distance_m",100)),
                guaranteed_decel_mps2=max(0.3,3.0*track_friction),
                curvature_1pm=track_curvature,
            )
            clearance_limit = SafetyEnvelope(
                clear_distance_m=envelope.clear_distance_m,
                guaranteed_decel_mps2=envelope.guaranteed_decel_mps2,
            ).speed_limit()
            curvature_limit = SafetyEnvelope(curvature_1pm=track_curvature).speed_limit()
            record["vcu"] = {
                "state": "EMERGENCY_STOP" if independent_override else vcu_output.state.value,
                "final_command": asdict(final_command),
                "motor_enable": False if independent_override else vcu_output.motor_enable,
                "contactor_open": True if independent_override else vcu_output.contactor_open,
                "faults": (
                    "SAFETY-WDG",
                    f"SAFETY-{vcu.independent.fault}",
                ) if independent_override else vcu_output.active_faults,
                "controlled_stop_phase": vcu_output.controlled_stop_phase,
                "active_speed_limit_mps": vcu_output.active_speed_limit_mps,
                "limiting_reason": vcu_output.limiting_reason,
                "command_age_s": 0.0 if time_s <= float(scenario.control.get("command_stop_time_s", scenario.duration_s)) else time_s - float(scenario.control.get("command_stop_time_s", scenario.duration_s)),
                "safety_limits_mps": {
                    "state": 8.0 if vcu_output.state == VcuState.AUTONOMOUS else vcu_output.active_speed_limit_mps,
                    "clear_distance": clearance_limit,
                    "curvature": curvature_limit,
                    "brake_capability": envelope.speed_limit(curvature=0.0),
                    "surface_friction": 8.0 * min(1.0, track_friction ** 0.5),
                    "thermal": 8.0,
                    "localization": 8.0,
                    "perception": clearance_limit,
                    "active": vcu_output.active_speed_limit_mps,
                },
                "stopping_distance": {
                    "delay_m": abs(kart.state.u_mps) * envelope.delay_s,
                    "braking_m": kart.state.u_mps ** 2 / (2.0 * envelope.guaranteed_decel_mps2),
                    "uncertainty_m": envelope.uncertainty_m,
                    "margin_m": envelope.margin_m,
                    "total_m": envelope.stop_distance(abs(kart.state.u_mps)),
                    "clear_distance_m": envelope.clear_distance_m,
                },
                "independent_safety": {
                    "stop_latched": vcu.independent.stop_latched,
                    "fault": vcu.independent.fault,
                    "motor_isolation_requested": vcu.independent.motor_isolation_requested,
                    "brake_fallback_requested": vcu.independent.brake_fallback_requested,
                },
            }
        if mode == "direct_test": record["event"]="VCU_BYPASS_ACTIVE"
        records.append(record)
        if record_callback is not None:
            record_callback(record)
        if realtime:
            sleep(dt_s)

    scheduler.add_callback(tick)
    scheduler.run_until(scenario.duration_s)
    destination = Path(output_dir) / scenario.name
    destination.mkdir(parents=True, exist_ok=True)
    with (destination / "telemetry.jsonl").open("w", encoding="utf-8") as stream:
        for record in records:
            stream.write(json.dumps(record, sort_keys=True) + "\n")
    metadata = {
        "scenario": scenario.name,
        "physics_dt_s": scheduler.dt_s,
        "steps": scheduler.step_count,
        "seed": kart.config.simulation.seed,
        "kart": configuration.kart.name,
        "track": track_model.name if track_model else None,
        "track_config": scenario.track_config,
        "control_mode": mode,
    }
    (destination / "metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return destination
