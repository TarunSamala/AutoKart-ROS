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

    def _axle_distances(self) -> tuple[float, float]:
        # CG x is measured from the wheelbase midpoint; a: CG to front, b: CG to rear.
        wheelbase = self.config.kart.wheelbase_m
        a_m = wheelbase / 2 - self.config.kart.cg.x_m
        b_m = wheelbase - a_m
        return a_m, b_m

    def step(self, command: VehicleCommand, dt_s: float | None = None) -> PlanarKartState:
        command.validated()
        dt = self.config.simulation.physics_dt_s if dt_s is None else dt_s
        if not 0 < dt <= 0.02:
            raise SimulationError("physics time step must be in (0, 0.02] seconds")
        s, cfg = self.state, self.config
        mass = cfg.kart.mass_kg
        speed = max(abs(s.u_mps), LOW_SPEED_MPS)
        steering = max(-cfg.kart.max_road_wheel_angle_rad, min(cfg.kart.max_road_wheel_angle_rad, command.steering_rad))
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
        brake_n = command.brake * cfg.brake.max_brake_force_n * direction
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
