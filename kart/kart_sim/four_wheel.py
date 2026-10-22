"""Phase-3 four-wheel analytical kart backend with a constrained solid rear axle.

This is a bounded engineering prototype, not a validated multibody model.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from math import atan2, cos, copysign, sin, tan

from .config import FullKartConfig
from .tires import LinearFrictionCircleTire, TireInput
from .types import PlanarKartState, SimulationError, VehicleCommand
from .vehicle import ForceBreakdown, GRAVITY_MPS2, LOW_SPEED_MPS


@dataclass
class WheelTelemetry:
    name: str
    steering_angle_rad: float = 0.0
    angular_speed_radps: float = 0.0
    vertical_load_n: float = 0.0
    slip_ratio: float = 0.0
    slip_angle_rad: float = 0.0
    longitudinal_force_n: float = 0.0
    lateral_force_n: float = 0.0
    utilisation: float = 0.0
    contact_valid: bool = True


@dataclass(frozen=True)
class FourWheelDiagnostics:
    chassis_twist_rad: float
    rear_axle_angular_speed_radps: float
    inside_rear_unloaded: bool


class FourWheelKart:
    """Four contact patches, load transfer approximation, and equal rear shaft speed."""

    WHEEL_NAMES = ("front_left", "front_right", "rear_left", "rear_right")

    def __init__(self, config: FullKartConfig, state: PlanarKartState | None = None) -> None:
        self.config = config
        self.state = state or PlanarKartState()
        self.tire = LinearFrictionCircleTire(config.tires)
        self.wheels = {name: WheelTelemetry(name) for name in self.WHEEL_NAMES}
        self.surface_friction_scales = {name: 1.0 for name in self.WHEEL_NAMES}
        self.rear_axle_angular_speed_radps = self.state.u_mps / config.tires.radius_m
        self.last_forces = ForceBreakdown(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.diagnostics = FourWheelDiagnostics(0.0, self.rear_axle_angular_speed_radps, False)

    def _axle_distances(self) -> tuple[float, float]:
        wheelbase = self.config.kart.wheelbase_m
        front = wheelbase / 2 - self.config.kart.cg.x_m
        return front, wheelbase - front

    def _steering_angles(self, commanded_rad: float) -> tuple[float, float]:
        maximum = self.config.kart.max_road_wheel_angle_rad
        delta = max(-maximum, min(maximum, commanded_rad))
        if abs(delta) < 1e-10:
            return 0.0, 0.0
        radius = self.config.kart.wheelbase_m / abs(tan(delta))
        half_track = self.config.kart.front_track_m / 2
        inner = atan2(self.config.kart.wheelbase_m, max(1e-9, radius - half_track))
        outer = atan2(self.config.kart.wheelbase_m, radius + half_track)
        return (inner, outer) if delta > 0 else (-outer, -inner)

    def _vertical_loads(self) -> tuple[dict[str, float], float]:
        """Quasi-static loads using previous acceleration; negative loads are never exposed."""
        s, cfg = self.state, self.config
        a_m, b_m = self._axle_distances()
        mass, wheelbase, cg_h = cfg.kart.mass_kg, cfg.kart.wheelbase_m, cfg.kart.cg.z_m
        front_total = mass * GRAVITY_MPS2 * b_m / wheelbase - mass * s.ax_mps2 * cg_h / wheelbase
        rear_total = mass * GRAVITY_MPS2 * a_m / wheelbase + mass * s.ax_mps2 * cg_h / wheelbase
        front_delta = mass * s.ay_mps2 * cg_h * (b_m / wheelbase) / cfg.kart.front_track_m
        rear_delta = mass * s.ay_mps2 * cg_h * (a_m / wheelbase) / cfg.kart.rear_track_m
        loads = {
            "front_left": max(0.0, front_total / 2 - front_delta / 2),
            "front_right": max(0.0, front_total / 2 + front_delta / 2),
            "rear_left": max(0.0, rear_total / 2 - rear_delta / 2),
            "rear_right": max(0.0, rear_total / 2 + rear_delta / 2),
        }
        lateral_load_difference = (loads["front_right"] - loads["front_left"]) - (loads["rear_right"] - loads["rear_left"])
        twist = lateral_load_difference * cg_h / max(1e-9, cfg.chassis.torsional_stiffness_nm_per_rad)
        return loads, twist

    def step(self, command: VehicleCommand, dt_s: float | None = None) -> PlanarKartState:
        command.validated()
        dt = self.config.simulation.physics_dt_s if dt_s is None else dt_s
        if not 0 < dt <= 0.02:
            raise SimulationError("physics time step must be in (0, 0.02] seconds")
        s, cfg = self.state, self.config
        a_m, b_m = self._axle_distances()
        fl_steer, fr_steer = self._steering_angles(command.steering_rad)
        steering = {"front_left": fl_steer, "front_right": fr_steer, "rear_left": 0.0, "rear_right": 0.0}
        positions = {
            "front_left": (a_m, cfg.kart.front_track_m / 2),
            "front_right": (a_m, -cfg.kart.front_track_m / 2),
            "rear_left": (-b_m, cfg.kart.rear_track_m / 2),
            "rear_right": (-b_m, -cfg.kart.rear_track_m / 2),
        }
        loads, twist = self._vertical_loads()
        longitudinal: dict[str, float] = {}
        lateral: dict[str, float] = {}
        for name in self.WHEEL_NAMES:
            x_m, y_m = positions[name]
            vx_body = s.u_mps - s.yaw_rate_radps * y_m
            vy_body = s.v_mps + s.yaw_rate_radps * x_m
            angle = steering[name]
            vx_wheel = vx_body * cos(angle) + vy_body * sin(angle)
            vy_wheel = -vx_body * sin(angle) + vy_body * cos(angle)
            if name.startswith("rear"):
                omega = self.rear_axle_angular_speed_radps
            else:
                # Non-driven wheel approximation; future wheel rotational dynamics replaces this.
                omega = vx_wheel / cfg.tires.radius_m
            denominator = max(abs(vx_wheel), LOW_SPEED_MPS)
            kappa = (omega * cfg.tires.radius_m - vx_wheel) / denominator
            alpha = atan2(vy_wheel, denominator)
            contact = loads[name] > 0.0
            friction_scale = cfg.environment.surface_friction_scale * self.surface_friction_scales[name]
            force = self.tire.force(TireInput(loads[name], kappa, alpha, friction_scale)) if contact else self.tire.force(TireInput(0.0, 0.0, 0.0, 1.0))
            wheel = self.wheels[name]
            wheel.steering_angle_rad = angle
            wheel.angular_speed_radps = omega
            wheel.vertical_load_n = loads[name]
            wheel.slip_ratio = kappa
            wheel.slip_angle_rad = alpha
            wheel.longitudinal_force_n = force.longitudinal_n
            wheel.lateral_force_n = force.lateral_n
            wheel.utilisation = force.utilisation
            wheel.contact_valid = contact
            longitudinal[name], lateral[name] = force.longitudinal_n, force.lateral_n

        # Rear axle has one constrained angular speed. Drive/brake torques affect shaft speed,
        # which produces rear tire slip; geometric left/right speed disagreement is rear scrub.
        drive_torque = command.drive * cfg.motor.max_wheel_force_n * cfg.tires.radius_m
        # Braking torque opposes shaft rotation; it must not propel or reverse the axle.
        axle_direction = copysign(1.0, self.rear_axle_angular_speed_radps) if abs(self.rear_axle_angular_speed_radps) > 1e-6 else 0.0
        brake_torque = command.brake * cfg.brake.max_brake_force_n * cfg.tires.radius_m * axle_direction
        resistance_torque = cfg.rear_axle.bearing_loss_nm * copysign(1.0, self.rear_axle_angular_speed_radps) if abs(self.rear_axle_angular_speed_radps) > 1e-6 else 0.0
        tire_torque = (longitudinal["rear_left"] + longitudinal["rear_right"]) * cfg.tires.radius_m
        axle_acceleration = (drive_torque - brake_torque * 0.5 - tire_torque - resistance_torque) / cfg.rear_axle.inertia_kg_m2
        self.rear_axle_angular_speed_radps += axle_acceleration * dt

        total_fx = total_fy = total_mz = 0.0
        for name in self.WHEEL_NAMES:
            angle = steering[name]
            fx_body = longitudinal[name] * cos(angle) - lateral[name] * sin(angle)
            fy_body = longitudinal[name] * sin(angle) + lateral[name] * cos(angle)
            x_m, y_m = positions[name]
            total_fx += fx_body
            total_fy += fy_body
            total_mz += x_m * fy_body - y_m * fx_body
        direction = copysign(1.0, s.u_mps) if abs(s.u_mps) >= LOW_SPEED_MPS else 0.0
        rolling = cfg.environment.rolling_resistance_coefficient * cfg.kart.mass_kg * GRAVITY_MPS2 * direction
        aerodynamic = 0.5 * cfg.environment.air_density_kg_per_m3 * cfg.environment.drag_coefficient_area_m2 * s.u_mps * abs(s.u_mps)
        grade = cfg.kart.mass_kg * GRAVITY_MPS2 * sin(cfg.environment.grade_rad)
        total_fx -= rolling + aerodynamic + grade
        ax = total_fx / cfg.kart.mass_kg + s.yaw_rate_radps * s.v_mps
        ay = total_fy / cfg.kart.mass_kg - s.yaw_rate_radps * s.u_mps
        yaw_acceleration = total_mz / cfg.kart.inertia.yaw_kg_m2
        previous_u = s.u_mps
        s.u_mps += ax * dt
        # A service brake dissipates energy; with no opposing propulsion it cannot reverse the kart.
        if command.brake > 0.0 and abs(command.drive) < 1e-12 and previous_u * s.u_mps < 0.0:
            s.u_mps = 0.0
            self.rear_axle_angular_speed_radps = 0.0
        # Static brake hold: once nearly stopped, adequate commanded mechanical
        # brake prevents grade-induced rollback. This is not direct deceleration.
        if command.brake > 0.1 and abs(command.drive) < 1e-12 and abs(previous_u) < LOW_SPEED_MPS:
            s.u_mps = 0.0
            self.rear_axle_angular_speed_radps = 0.0
        s.v_mps += ay * dt
        s.yaw_rate_radps += yaw_acceleration * dt
        s.x_m += (s.u_mps * cos(s.yaw_rad) - s.v_mps * sin(s.yaw_rad)) * dt
        s.y_m += (s.u_mps * sin(s.yaw_rad) + s.v_mps * cos(s.yaw_rad)) * dt
        s.yaw_rad += s.yaw_rate_radps * dt
        s.time_s += dt
        s.ax_mps2, s.ay_mps2 = ax, ay
        s.assert_finite()
        rear_unloaded = min(loads["rear_left"], loads["rear_right"]) <= 1e-9
        self.diagnostics = FourWheelDiagnostics(twist, self.rear_axle_angular_speed_radps, rear_unloaded)
        self.last_forces = ForceBreakdown(drive_torque / cfg.tires.radius_m, brake_torque / cfg.tires.radius_m, rolling, aerodynamic, grade, total_fy, 0.0)
        return s

    def telemetry(self) -> dict:
        return {"wheels": {name: asdict(wheel) for name, wheel in self.wheels.items()}, "diagnostics": asdict(self.diagnostics)}
