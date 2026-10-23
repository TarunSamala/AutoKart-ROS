"""Gym-style deterministic fast racing environment routed through the VCU."""

from __future__ import annotations

from dataclasses import asdict
from math import cos, hypot, sin
from pathlib import Path
from time import perf_counter
from typing import Any, Callable, Iterable

from .config import FullKartConfig, load_kart_config
from .dynamic_kart import DynamicFourWheelKart
from .electrical import ContactorState
from .sensor_suite import LidarConfiguration, PlanarLidar
from .track import Projection, Track
from .types import PlanarKartState, SimulationError, VehicleCommand
from .vcu import AutonomousGuards, MotionCommand, SafetyEnvelope, VCU, VcuInputs, VcuOutputs, VcuState


class AutoKartFastEnv:
    """Dependency-free RL API with VCU and independent-safety enforcement.

    Actions are ``(target_speed_mps, steering_rad)`` or a mapping with those
    keys. The plant receives only ``VcuOutputs.command``.
    """

    metadata = {"render_modes": ["none", "ansi"], "name": "AutoKartFast-v0"}

    def __init__(
        self,
        track: str | Path | Track,
        kart: str | Path | FullKartConfig,
        *,
        control_dt_s: float = 0.02,
        maximum_steps: int = 5000,
        lidar_beams: int = 108,
        safety_evaluation: bool = True,
        namespace: str = "/kart_01",
    ) -> None:
        self.track = track if isinstance(track, Track) else Track.from_yaml(track)
        self.config = kart if isinstance(kart, FullKartConfig) else load_kart_config(kart)
        if self.config.simulation.vehicle_backend != "dynamic_four_wheel":
            raise SimulationError("fast integrated mode requires the dynamic actuator plant")
        ratio = control_dt_s / self.config.simulation.physics_dt_s
        if control_dt_s <= 0 or abs(ratio - round(ratio)) > 1e-9:
            raise SimulationError("control period must be an integer multiple of physics dt")
        if not namespace.startswith("/"):
            raise SimulationError("kart namespace must be absolute")
        self.control_dt_s = control_dt_s
        self.substeps = int(round(ratio))
        self.maximum_steps = maximum_steps
        self.safety_evaluation = safety_evaluation
        self.namespace = namespace.rstrip("/")
        self.lidar_config = LidarConfiguration(beams=lidar_beams)
        self._seed = self.config.simulation.seed
        self._epoch = 0
        self._sequence = 0
        self._steps = 0
        self._previous_action = (0.0, 0.0)
        self._previous_s_m = 0.0
        self._unwrapped_progress_m = 0.0
        self._lap_count = 0
        self._actors: tuple[tuple[float, float, float, float], ...] = ()
        self.active_faults: set[str] = set()
        self.kart: DynamicFourWheelKart
        self.vcu: VCU
        self.vcu_output: VcuOutputs | None
        self.projection: Projection
        self.lidar: PlanarLidar
        self._physics_observers: list[Callable[["AutoKartFastEnv"], None]] = []
        self._timing_monitor: Any | None = None
        self.reset(seed=self._seed)

    @property
    def state(self) -> PlanarKartState:
        return self.kart.state

    def set_local_actors(self, actors: Iterable[tuple[float, float, float, float]]) -> None:
        self._actors = tuple(actors)

    def add_physics_observer(self, callback: Callable[["AutoKartFastEnv"], None]) -> None:
        """Observe completed plant steps without gaining command authority."""
        if callback in self._physics_observers:
            raise SimulationError("physics observer is already registered")
        self._physics_observers.append(callback)

    def set_timing_monitor(self, monitor: Any | None) -> None:
        """Attach an optional monitor exposing ``record(task, elapsed_s)``."""
        if monitor is not None and not callable(getattr(monitor, "record", None)):
            raise SimulationError("timing monitor must expose record(task, elapsed_s)")
        self._timing_monitor = monitor

    def request_controlled_stop(self, reason: str = "path_complete") -> None:
        """Request the VCU-owned controlled-stop state after a high-level event."""
        if not reason or self.vcu.state not in {VcuState.AUTONOMOUS, VcuState.DEGRADED}:
            raise SimulationError("controlled stop requires an active autonomous state and reason")
        if not self.vcu.transition(VcuState.CONTROLLED_STOP):
            raise SimulationError("VCU rejected controlled-stop request")

    def _initial_state(self) -> PlanarKartState:
        point = self.track.sample(0.0)
        return PlanarKartState(x_m=point.x_m, y_m=point.y_m, z_m=point.z_m, yaw_rad=point.heading_rad)

    def _startup_vcu(self) -> VCU:
        vcu = VCU()
        for state in (VcuState.BOOT, VcuState.SELF_TEST, VcuState.SAFE):
            if not vcu.transition(state):
                raise SimulationError("VCU failed deterministic startup guards")
        return vcu

    def arm_autonomy(self, *, operator_acknowledged: bool) -> None:
        if not operator_acknowledged:
            raise SimulationError("autonomous enable requires explicit operator acknowledgement")
        if self.vcu.state != VcuState.SAFE or not self.vcu.transition(VcuState.READY):
            raise SimulationError("autonomous enable requires SAFE state")
        if not self.vcu.request_autonomous(AutonomousGuards()):
            raise SimulationError("VCU rejected deterministic autonomy guards")

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[dict[str, Any], dict[str, Any]]:
        options = options or {}
        if hasattr(self, "kart") and abs(self.state.u_mps) >= 0.05 and not options.get("physical_stopped_confirmation", False):
            raise SimulationError("reset requires physical stopped confirmation")
        if self.active_faults:
            if not options.get("repair_acknowledged", False):
                raise SimulationError("latched simulation faults require repair acknowledgement")
            self.active_faults.clear()
        if seed is not None:
            self._seed = int(seed)
        self._epoch += 1
        self._sequence = 0
        self._steps = 0
        self._previous_action = (0.0, 0.0)
        self._previous_s_m = 0.0
        self._unwrapped_progress_m = 0.0
        self._lap_count = 0
        self.kart = DynamicFourWheelKart(self.config, self._initial_state())
        self.vcu = self._startup_vcu()
        self.vcu.reset_epoch = self._epoch
        self.vcu_output = None
        self.projection = self.track.project(self.state.x_m, self.state.y_m, self.state.z_m)
        self.lidar = PlanarLidar(
            f"{self.namespace}/lidar",
            f"{self.namespace}/lidar_link",
            1.0 / self.control_dt_s,
            self.lidar_config,
            seed=self._seed,
        )
        if options.get("arm", False):
            self.arm_autonomy(operator_acknowledged=bool(options.get("operator_acknowledged", False)))
        # Reset intentionally has no command. A caller must step with a fresh,
        # post-epoch action before propulsion authority exists.
        observation = self._observation()
        return observation, self._info(reset=True)

    def inject_fault(self, fault: str) -> None:
        """Inject a platform fault using stable user-facing fault names."""
        plant_faults = {
            "steering_jam": "steering:jammed",
            "steering_linkage_break": "steering:linkage_break",
            "brake_cable_break": "brake:cable_break",
            "brake_fade": "brake:fade",
            "brake_release_failure": "brake:not_releasing",
            "stuck_motor_torque": "motor:torque_stuck_positive",
            "contactor_welded": "battery:contactor_welded",
            "battery_sag": "battery:undervoltage",
            "power_domain_failure": "battery:bms_shutdown",
        }
        supported = {
            *plant_faults,
            "communication_loss",
            "command_timeout",
            "vcu_freeze",
            "sensor_freeze",
            "sensor_drift",
            "sensor_latency",
            "sensor_dropout",
            "compute_overload",
            "wet_surface",
            "fog",
            "low_light",
            "obstacle_intrusion",
            "physical_estop",
        }
        if fault not in supported:
            raise SimulationError(f"unsupported fast-mode fault: {fault}")
        self.active_faults.add(fault)
        if fault in plant_faults:
            self.kart.inject_fault(plant_faults[fault])
        if fault == "sensor_freeze":
            self.lidar.fault = "frozen"
        elif fault in {"sensor_dropout", "fog"}:
            self.lidar.fault = "dropout"
        elif fault == "physical_estop":
            self.vcu.independent.estop = True

    def _parse_action(self, action: Any) -> tuple[float, float]:
        if isinstance(action, dict):
            speed = float(action["target_speed_mps"])
            steering = float(action.get("steering_rad", action.get("curvature_1pm", 0.0)))
        elif isinstance(action, (tuple, list)) and len(action) == 2:
            speed, steering = map(float, action)
        else:
            raise SimulationError("action must be a speed/steering pair or mapping")
        if not 0.0 <= speed <= 8.0 or not -0.6 <= steering <= 0.6:
            raise SimulationError("action exceeds configured VCU input range")
        return speed, steering

    def _feedback(self, timestamp_s: float, command: MotionCommand | None) -> VcuInputs:
        wheel_speeds = tuple(
            wheel.angular_speed_radps * self.config.tires.radius_m for wheel in self.kart.wheels.values()
        )
        steering = self.kart.steering.feedback
        brakes = self.kart.brakes.feedback
        motor = self.kart.motor.feedback
        return VcuInputs(
            timestamp_s=timestamp_s,
            motion_command=command,
            vehicle_speed_mps=self.state.u_mps,
            steering_angle_rad=steering.linkage_angle_rad,
            brake_pressure_pa=brakes.hydraulic_pressure_pa,
            motor_torque_nm=motor.actual_torque_nm,
            brake_actuator_position_m=brakes.actual_position_m,
            independent_healthy=self.vcu.independent.permit(timestamp_s),
            wheel_speeds_mps=wheel_speeds,
            wheel_feedback_valid=True,
            motor_current_a=motor.motor_current_a,
            motor_feedback_valid=True,
            brake_feedback_valid=True,
            steering_feedback_valid=True,
            contactor_feedback_closed=self.kart.battery.feedback.contactor_state in (
                ContactorState.CLOSED,
                ContactorState.WELDED,
            ),
            feedback_timestamp_s=timestamp_s,
            steering_secondary_angle_rad=steering.left_wheel_angle_rad,
            steering_current_a=steering.current_a,
            brake_torque_nm=brakes.brake_torque_nm,
            vehicle_acceleration_mps2=self.state.ax_mps2,
            surface_friction=self.track.surface_at(self.projection.s_m, self.projection.lateral_m).material.friction_longitudinal,
            chain_torque_nm=motor.chain_torque_nm,
            motor_speed_feedback_valid=True,
            steering_actuator_angle_rad=steering.shaft_angle_rad,
        )

    def step(self, action: Any) -> tuple[dict[str, Any], float, bool, bool, dict[str, Any]]:
        if self.vcu.state not in {VcuState.AUTONOMOUS, VcuState.DEGRADED, VcuState.CONTROLLED_STOP, VcuState.EMERGENCY_STOP, VcuState.FAULT_LOCKOUT}:
            raise SimulationError("kart is not autonomously armed after reset")
        speed, steering = self._parse_action(action)
        start_s = self.projection.s_m
        for _ in range(self.substeps):
            physics_started = perf_counter() if self._timing_monitor is not None else 0.0
            timestamp_s = self.state.time_s
            if "vcu_freeze" not in self.active_faults:
                safety_started = perf_counter() if self._timing_monitor is not None else 0.0
                self.vcu.independent.pulse(timestamp_s)
                if self._timing_monitor is not None:
                    self._timing_monitor.record(
                        "independent_safety", perf_counter() - safety_started
                    )
            self._sequence = (self._sequence + 1) % 256
            command: MotionCommand | None = MotionCommand(
                "baseline_controller",
                self._sequence,
                timestamp_s,
                timestamp_s + max(0.05, 2 * self.config.simulation.physics_dt_s),
                speed,
                steering,
                reset_epoch=self._epoch,
            ).signed()
            if {"communication_loss", "command_timeout"}.intersection(self.active_faults):
                command = None
            point = self.track.sample(self.projection.s_m)
            surface = self.track.surface_at(self.projection.s_m, self.projection.lateral_m)
            friction_scale = 0.5 if "wet_surface" in self.active_faults else 1.0
            for wheel_id in self.kart.vehicle.surface_friction_scales:
                self.kart.vehicle.surface_friction_scales[wheel_id] = (
                    surface.material.friction_longitudinal * friction_scale
                )
            envelope = SafetyEnvelope(
                clear_distance_m=max(0.0, min(self.projection.left_margin_m, self.projection.right_margin_m) + 20.0),
                guaranteed_decel_mps2=max(0.3, 3.0 * surface.material.friction_longitudinal * friction_scale),
                curvature_1pm=point.curvature_1pm,
            )
            if "vcu_freeze" not in self.active_faults or self.vcu_output is None:
                vcu_started = perf_counter() if self._timing_monitor is not None else 0.0
                feedback_command = command
                if feedback_command is None:
                    # _feedback accepts Optional at runtime and forwards it unchanged.
                    self.vcu_output = self.vcu.cycle(self._feedback(timestamp_s, feedback_command), envelope)
                else:
                    self.vcu_output = self.vcu.cycle(self._feedback(timestamp_s, feedback_command), envelope)
                if self._timing_monitor is not None:
                    self._timing_monitor.record("vcu", perf_counter() - vcu_started)
            assert self.vcu_output is not None
            final = self.vcu_output.command
            if not self.vcu.independent.permit(timestamp_s):
                final = VehicleCommand(brake=1.0, steering_rad=self.kart.steering.feedback.linkage_angle_rad)
                self.kart.step(final, motor_enable=False, contactor_open=True)
            else:
                self.kart.step(
                    final,
                    motor_enable=self.vcu_output.motor_enable,
                    contactor_open=self.vcu_output.contactor_open,
                )
            if self._timing_monitor is not None:
                self._timing_monitor.record("physics", perf_counter() - physics_started)
            for observer in self._physics_observers:
                observer(self)
            self.projection = self.track.project(
                self.state.x_m,
                self.state.y_m,
                self.state.z_m,
                self.projection.s_m,
            )
        self._steps += 1
        self._previous_action = (speed, steering)
        delta_s = self.projection.s_m - start_s
        if self.track.closed:
            if delta_s < -0.5 * self.track.length_m:
                delta_s += self.track.length_m
                self._lap_count += 1
            elif delta_s > 0.5 * self.track.length_m:
                delta_s -= self.track.length_m
        self._unwrapped_progress_m += delta_s
        out_of_bounds = min(self.projection.left_margin_m, self.projection.right_margin_m) < 0.0
        safety_stop = self.vcu.state in {VcuState.EMERGENCY_STOP, VcuState.FAULT_LOCKOUT}
        terminated = out_of_bounds or safety_stop
        truncated = self._steps >= self.maximum_steps
        reward = delta_s - (5.0 if out_of_bounds else 0.0) - (10.0 if safety_stop else 0.0)
        return self._observation(), reward, terminated, truncated, self._info(
            out_of_bounds=out_of_bounds,
            safety_stop=safety_stop,
        )

    def _actors_local(self) -> tuple[tuple[float, float, float, float], ...]:
        result = []
        for world_x, world_y, radius, reflectivity in self._actors:
            dx, dy = world_x - self.state.x_m, world_y - self.state.y_m
            result.append((
                cos(self.state.yaw_rad) * dx + sin(self.state.yaw_rad) * dy,
                -sin(self.state.yaw_rad) * dx + cos(self.state.yaw_rad) * dy,
                radius,
                reflectivity,
            ))
        return tuple(result)

    def _observation(self) -> dict[str, Any]:
        scan = self.lidar.measure({
            "left_margin_m": max(0.0, self.projection.left_margin_m),
            "right_margin_m": max(0.0, self.projection.right_margin_m),
            "actors_local": self._actors_local(),
        })
        return {
            "lidar": tuple(scan["ranges_m"]),
            "pose": (self.state.x_m, self.state.y_m, self.state.yaw_rad),
            "velocity": (self.state.u_mps, self.state.v_mps, self.state.yaw_rate_radps),
            "track_progress": self._unwrapped_progress_m / self.track.length_m,
            "previous_action": self._previous_action,
            "safety_state": self.vcu.state.value,
        }

    def _info(self, **extra: Any) -> dict[str, Any]:
        return {
            "namespace": self.namespace,
            "seed": self._seed,
            "reset_epoch": self._epoch,
            "physics_backend": "internal",
            "vcu_routed": True,
            "safety_evaluation": self.safety_evaluation,
            "lap_count": self._lap_count,
            "track_s_m": self.projection.s_m,
            "lateral_m": self.projection.lateral_m,
            "vcu_faults": tuple(self.vcu.faults),
            "simulation_faults": tuple(sorted(self.active_faults)),
            **extra,
        }

    def render(self) -> str:
        return (
            f"{self.namespace} t={self.state.time_s:.2f}s "
            f"s={self.projection.s_m:.2f}m v={self.state.u_mps:.2f}m/s "
            f"VCU={self.vcu.state.value}"
        )


class VectorAutoKartEnv:
    """Synchronous deterministic vector wrapper without hidden shared state."""

    def __init__(self, environments: Iterable[AutoKartFastEnv]) -> None:
        self.environments = tuple(environments)
        if not self.environments:
            raise SimulationError("vector environment cannot be empty")
        namespaces = [env.namespace for env in self.environments]
        if len(set(namespaces)) != len(namespaces):
            raise SimulationError("vector environments require unique namespaces")

    def reset(self, *, seed: int | None = None) -> tuple[tuple[dict[str, Any], ...], tuple[dict[str, Any], ...]]:
        results = tuple(env.reset(seed=None if seed is None else seed + index) for index, env in enumerate(self.environments))
        return tuple(item[0] for item in results), tuple(item[1] for item in results)

    def step(self, actions: Iterable[Any]) -> tuple[tuple[Any, ...], ...]:
        actions_tuple = tuple(actions)
        if len(actions_tuple) != len(self.environments):
            raise SimulationError("action count does not match vector environment count")
        results = tuple(env.step(action) for env, action in zip(self.environments, actions_tuple))
        return tuple(tuple(result[index] for result in results) for index in range(5))
