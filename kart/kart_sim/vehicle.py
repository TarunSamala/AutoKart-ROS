"""Phase-2 planar analytical vehicle model; not yet a validated physical kart model."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, copysign, sin

from .config import FullKartConfig
from .types import PlanarKartState, SimulationError, VehicleCommand

GRAVITY_MPS2 = 9.80665
LOW_SPEED_MPS = 0.05


@dataclass(frozen=True)
class ForceBreakdown:
    drive_n: float
    brake_n: float
    rolling_n: float
    aerodynamic_n: float
    grade_n: float
    lateral_front_n: float
    lateral_rear_n: float


class PlanarBicycleKart:
    """Single-track bicycle model with force saturation and semi-implicit Euler integration."""

    def __init__(self, config: FullKartConfig, state: PlanarKartState | None = None) -> None:
        self.config = config
        self.state = state or PlanarKartState()
        self.last_forces = ForceBreakdown(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        # Actuator-lag filter state (see ActuatorLagConfig): the steering angle and
        # brake fraction actually reaching the tyres this step, after rate limiting,
        # first-order lag, and brake dead time.
        self._steer_actual_rad = 0.0
        self._brake_actual = 0.0
        self._brake_dead_time_remaining_s = config.actuator_lag.brake.dead_time_s

    def _axle_distances(self) -> tuple[float, float]:
        # CG x is measured from the wheelbase midpoint; a: CG to front, b: CG to rear.
        wheelbase = self.config.kart.wheelbase_m
        a_m = wheelbase / 2 - self.config.kart.cg.x_m
        b_m = wheelbase - a_m
        return a_m, b_m

    def _update_actuator_lag(self, command: VehicleCommand, dt: float) -> None:
        kart, lag = self.config.kart, self.config.actuator_lag
        steer_cfg, brake_cfg = lag.steering, lag.brake

        target_steer_rad = max(-kart.max_road_wheel_angle_rad, min(kart.max_road_wheel_angle_rad, command.steering_rad))
        if steer_cfg.lag_s > 0.0:
            alpha = dt / (steer_cfg.lag_s + dt)
            lagged_rad = self._steer_actual_rad + alpha * (target_steer_rad - self._steer_actual_rad)
        else:
            lagged_rad = target_steer_rad
        max_steer_step = steer_cfg.rate_limit_radps * dt
        self._steer_actual_rad += max(-max_steer_step, min(max_steer_step, lagged_rad - self._steer_actual_rad))

        target_brake = max(0.0, min(1.0, command.brake))
        max_brake_step = brake_cfg.ramp_rate_per_s * dt
        if target_brake <= 1e-9:
            # Releasing the brake resets the dead time, so the next press incurs it again.
            self._brake_dead_time_remaining_s = brake_cfg.dead_time_s
            self._brake_actual = max(0.0, self._brake_actual - max_brake_step)
        elif self._brake_dead_time_remaining_s > 0.0:
            self._brake_dead_time_remaining_s = max(0.0, self._brake_dead_time_remaining_s - dt)
        else:
            self._brake_actual += max(0.0, min(max_brake_step, target_brake - self._brake_actual))

    def step(self, command: VehicleCommand, dt_s: float | None = None) -> PlanarKartState:
        command.validated()
        dt = self.config.simulation.physics_dt_s if dt_s is None else dt_s
        if not 0 < dt <= 0.02:
            raise SimulationError("physics time step must be in (0, 0.02] seconds")
        s, cfg = self.state, self.config
        mass = cfg.kart.mass_kg
        speed = max(abs(s.u_mps), LOW_SPEED_MPS)
        self._update_actuator_lag(command, dt)
        steering = self._steer_actual_rad
        a_m, b_m = self._axle_distances()

        alpha_front = atan2(s.v_mps + a_m * s.yaw_rate_radps, speed) - steering
        alpha_rear = atan2(s.v_mps - b_m * s.yaw_rate_radps, speed)
        static_front_n = mass * GRAVITY_MPS2 * b_m / cfg.kart.wheelbase_m
        static_rear_n = mass * GRAVITY_MPS2 * a_m / cfg.kart.wheelbase_m
        max_front_n = cfg.tires.friction_peak * cfg.environment.surface_friction_scale * static_front_n
        max_rear_n = cfg.tires.friction_peak * cfg.environment.surface_friction_scale * static_rear_n
        fy_front = max(-max_front_n, min(max_front_n, -cfg.tires.cornering_stiffness_n_per_rad * alpha_front))
        fy_rear = max(-max_rear_n, min(max_rear_n, -cfg.tires.cornering_stiffness_n_per_rad * alpha_rear))

        direction = copysign(1.0, s.u_mps) if abs(s.u_mps) >= LOW_SPEED_MPS else 0.0
        drive_n = command.drive * cfg.motor.max_wheel_force_n
        brake_n = self._brake_actual * cfg.brake.max_brake_force_n * direction
        rolling_n = cfg.environment.rolling_resistance_coefficient * mass * GRAVITY_MPS2 * direction
        aerodynamic_n = 0.5 * cfg.environment.air_density_kg_per_m3 * cfg.environment.drag_coefficient_area_m2 * s.u_mps * abs(s.u_mps)
        grade_n = mass * GRAVITY_MPS2 * sin(cfg.environment.grade_rad)
        fx = drive_n - brake_n - rolling_n - aerodynamic_n - grade_n - fy_front * sin(steering)
        fy = fy_rear + fy_front * cos(steering)
        yaw_moment = a_m * fy_front * cos(steering) - b_m * fy_rear
        ax = fx / mass + s.yaw_rate_radps * s.v_mps
        ay = fy / mass - s.yaw_rate_radps * s.u_mps
        yaw_accel = yaw_moment / cfg.kart.inertia.yaw_kg_m2

        s.u_mps += ax * dt
        s.v_mps += ay * dt
        s.yaw_rate_radps += yaw_accel * dt
        s.x_m += (s.u_mps * cos(s.yaw_rad) - s.v_mps * sin(s.yaw_rad)) * dt
        s.y_m += (s.u_mps * sin(s.yaw_rad) + s.v_mps * cos(s.yaw_rad)) * dt
        s.yaw_rad += s.yaw_rate_radps * dt
        s.time_s += dt
        s.ax_mps2 = ax
        s.ay_mps2 = ay
        s.assert_finite()
        self.last_forces = ForceBreakdown(drive_n, brake_n, rolling_n, aerodynamic_n, grade_n, fy_front, fy_rear)
        return s
