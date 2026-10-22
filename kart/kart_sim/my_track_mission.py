"""my_track mission state machine: blind creep, Pure Pursuit + speed planning
over the algorithm's output path, obstacle detour admission and locking,
finish braking, and the PATH_LOST safety fallback.

Ported from docs/my_track_spec/MY_TRACK_SIM.py's plan_step()/simulate() control
cycle, but reusing the existing controllers rather than the draft's own
inline Pure Pursuit/PI math: kart_sim.autonomy.PurePursuitController and
LongitudinalPidController. Odometry here is the kart's TRUE pose/speed (no
separate encoder/gyro dead-reckoning noise model) -- see docs/MY_TRACK.md for
why that is scoped out.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import cos, sin, sqrt

import numpy as np

from .autonomy import ControllerOutput, LongitudinalPidController, MessageHeader, PurePursuitController, Trajectory, TrajectoryPoint
from .hp60c import TrackedCone
from .my_track_algorithm import (
    AlgorithmConfig,
    DetourLock,
    PlanResult,
    add_single_side_fallback_gates,
    body_frame_points,
    build_gates,
    classify_obstacles,
    cluster_indices,
    detect_finish,
    evaluate_centreline,
    fit_centreline,
    footprint_clearance_m,
)

MISSION_STATES = ("READY", "RUN", "OBSTACLE_ASSESS", "DETOUR", "RECENTER", "FINISH_BRAKING", "CONTROLLED_STOP", "PATH_LOST", "STOPPED")


@dataclass(frozen=True)
class MissionConfig:
    cruise_speed_mps: float = 0.5
    blind_creep_speed_mps: float = 0.3
    blind_creep_distance_m: float = 6.0
    blind_creep_min_gates: int = 2
    lookahead_min_m: float = 1.8
    lookahead_max_m: float = 3.0
    lookahead_base_m: float = 1.2
    lookahead_speed_gain_s: float = 2.0
    finish_stop_margin_m: float = 0.3
    brake_margin_m: float = 1.5
    path_lost_timeout_s: float = 2.0
    # Speed-command ramp limiter: the draft's own ad hoc PI overshot the 0.5
    # m/s target by ~56% (hit 0.78 m/s). Rather than re-tuning the shared
    # LongitudinalPidController's gains (used by every other scenario too),
    # the target speed fed to it is slew-rate limited. ASSUMED.
    speed_command_ramp_mps2: float = 0.3
    algorithm: AlgorithmConfig = field(default_factory=AlgorithmConfig)


@dataclass
class MissionCommand:
    state: str
    steering_rad: float
    drive: float
    brake: float
    valid: bool
    gates: int
    path_lost: bool
    obstacle: bool
    detour_offset_m: float
    finish: bool
    target_speed_mps: float
    centreline_offset_m: float


def _stopping_distance_m(speed_mps: float, delay_s: float, decel_mps2: float, margin_m: float) -> float:
    """d = v*tau + v^2/(2a) + margin."""
    return speed_mps * delay_s + speed_mps**2 / (2 * decel_mps2) + margin_m


def _max_speed_for_stop_distance_m(distance_m: float, delay_s: float, decel_mps2: float, margin_m: float) -> float:
    """Inverts d = v*tau + v^2/(2a) + margin for v, given an available distance d."""
    available_m = distance_m - margin_m
    if available_m <= 0.0:
        return 0.0
    return max(0.0, -decel_mps2 * delay_s + sqrt((decel_mps2 * delay_s) ** 2 + 2 * decel_mps2 * available_m))


class MissionController:
    def __init__(self, config: MissionConfig, *, wheelbase_m: float, front_extent_m: float, rear_overhang_m: float, half_width_m: float,
                 brake_delay_s: float, brake_decel_mps2: float) -> None:
        self.config = config
        self.front_extent_m = front_extent_m
        self.rear_overhang_m = rear_overhang_m
        self.half_width_m = half_width_m
        self.brake_delay_s = brake_delay_s
        self.brake_decel_mps2 = brake_decel_mps2
        self.pure_pursuit = PurePursuitController(
            wheelbase_m=wheelbase_m, lookahead_m=config.lookahead_base_m, speed_lookahead_gain_s=config.lookahead_speed_gain_s,
            maximum_steering_rad=0.3490658504,  # 20 deg software clamp, see KartConfig.max_road_wheel_angle_rad
        )
        self.speed_pid = LongitudinalPidController()
        self.locks: list[DetourLock] = []
        self.state = "READY"
        self.fault: str | None = None
        self._path_lost_s = 0.0
        self._ramped_target_mps = 0.0
        self._sequence = 0

    def reset_speed_ramp(self) -> None:
        self._ramped_target_mps = 0.0
        self.speed_pid.reset()

    def _plan(self, tracks: list[TrackedCone], pose: tuple[float, float, float], speed_mps: float, odometry_m: float, world_cluster_centroid) -> PlanResult:
        cfg = self.config.algorithm
        points = body_frame_points(tracks, pose, cfg)
        if len(points) < 3:
            return self._blind(odometry_m, gates=0)

        finish_x, finish_indices = detect_finish(points, odometry_m, cfg)
        obstacle_indices = classify_obstacles(points, finish_indices, cfg)
        boundary_indices = [i for i in range(len(points)) if i not in obstacle_indices and i not in finish_indices]
        if len(boundary_indices) < 3:
            return self._blind(odometry_m, gates=0)
        boundary_points = [points[i] for i in boundary_indices]

        real_gates = build_gates(boundary_points, cfg)
        fallback_gates = add_single_side_fallback_gates(boundary_points, real_gates, cfg)
        all_gates = real_gates + fallback_gates
        if len(all_gates) < cfg.centreline_min_gates:
            return self._blind(odometry_m, gates=len(all_gates))

        fit = fit_centreline(all_gates)
        if fit is None:
            return self._blind(odometry_m, gates=len(all_gates))
        x_lo, x_hi = fit[1], fit[2]
        widths_m = [g.width_m for g in all_gates]
        half_lane_m = float(np.median(widths_m)) / 2
        lookahead_m = max(self.config.lookahead_min_m, min(self.config.lookahead_max_m, self.config.lookahead_base_m + self.config.lookahead_speed_gain_s * max(speed_mps, 0.0)))

        xs = np.arange(0.0, max(x_hi, 1.0) + lookahead_m + 3.0, 0.1)
        ys = evaluate_centreline(fit, xs)
        offset = np.zeros_like(xs)

        clusters = cluster_indices(obstacle_indices, points, cfg.cluster_link_m)
        stop_distance_m: float | None = None
        detour_active = False
        new_lock: DetourLock | None = None
        for component in clusters:
            cx = np.array([points[i][0] for i in component])
            cy = np.array([points[i][1] for i in component])
            x_min_m, x_max_m = float(cx.min()), float(cx.max())
            if x_max_m < -3.0:
                continue
            lateral_from_centreline = cy - evaluate_centreline(fit, cx)
            side_sign = 1 if lateral_from_centreline.mean() > 0 else -1
            clearance_to_centreline_m = float(np.min(np.abs(lateral_from_centreline)))
            ramp_height_m = (-side_sign * half_lane_m + side_sign * clearance_to_centreline_m) / 2
            free_width_m = half_lane_m + clearance_to_centreline_m - 2 * cfg.cone_radius_m
            admitted, chosen_ramp_m, chosen_lead_m = False, None, None
            if free_width_m >= self.half_width_m * 2 + 2 * 0.25:
                for ramp_m in cfg.detour_ramp_lengths_m:
                    lead_m = 0.0
                    while x_min_m - lead_m - ramp_m >= -0.2:
                        ramp_offset, ramp_start_x_m = self._ramp(xs, x_min_m, x_max_m, lead_m, ramp_m, ramp_height_m)
                        candidate_ys = ys + ramp_offset
                        heading = np.arctan2(np.gradient(candidate_ys), np.gradient(xs))
                        swept_ok = True
                        for k in range(0, len(xs), 2):
                            for px_m, py_m, _, _ in points:
                                if px_m > xs[k] + self.front_extent_m + 1 or px_m < xs[k] - 2:
                                    continue
                                clearance_m = footprint_clearance_m(px_m, py_m, xs[k], candidate_ys[k], heading[k], self.rear_overhang_m, self.front_extent_m, self.half_width_m) - cfg.cone_radius_m
                                if clearance_m < cfg.footprint_margin_m:
                                    swept_ok = False
                                    break
                            if not swept_ok:
                                break
                        if swept_ok:
                            admitted, chosen_ramp_m, chosen_lead_m = True, ramp_m, lead_m
                            new_lock = DetourLock(world_cluster_centroid(component), ramp_m, lead_m)
                            break
                        lead_m += cfg.detour_lead_step_m
                    if admitted:
                        break
            if admitted:
                ramp_offset, _ = self._ramp(xs, x_min_m, x_max_m, chosen_lead_m, chosen_ramp_m, ramp_height_m)
                offset += ramp_offset
                detour_active = True
            else:
                candidate_stop_m = x_min_m - self.front_extent_m - 0.5
                stop_distance_m = candidate_stop_m if stop_distance_m is None else min(stop_distance_m, candidate_stop_m)

        path_xy = tuple(zip(xs.tolist(), (ys + offset).tolist()))
        return PlanResult(
            valid=True, gates=len(all_gates), path_xy=path_xy,
            obstacle=bool(clusters), detour=detour_active, detour_offset0_m=float(offset[0]) if len(offset) else 0.0,
            finish_detected=finish_x is not None, finish_x_body_m=finish_x,
            blind=False, path_lost=False, stop_distance_m=stop_distance_m, new_lock=new_lock,
        )

    @staticmethod
    def _ramp(xs: np.ndarray, x_min_m: float, x_max_m: float, lead_m: float, ramp_m: float, height_m: float) -> tuple[np.ndarray, float]:
        ramp_end_x_m = x_min_m - lead_m
        ramp_start_x_m = ramp_end_x_m - ramp_m
        decel_end_x_m = x_max_m + 0.4
        decel_full_x_m = decel_end_x_m + ramp_m
        offset = height_m * (_smoothstep((xs - ramp_start_x_m) / max(ramp_end_x_m - ramp_start_x_m, 1e-6))
                              - _smoothstep((xs - decel_end_x_m) / max(decel_full_x_m - decel_end_x_m, 1e-6)))
        return offset, ramp_start_x_m

    def _blind(self, odometry_m: float, gates: int) -> PlanResult:
        if odometry_m < self.config.blind_creep_distance_m and gates < self.config.blind_creep_min_gates:
            return PlanResult(False, gates, None, False, False, 0.0, False, None, True, False, None, None)
        return PlanResult(False, gates, None, False, False, 0.0, False, None, False, True, None, None)

    def step(self, *, tracks: list[TrackedCone], pose: tuple[float, float, float], speed_mps: float, odometry_m: float,
              dt_s: float, timestamp_s: float, world_cluster_centroid) -> MissionCommand:
        plan = self._plan(tracks, pose, speed_mps, odometry_m, world_cluster_centroid)
        if plan.new_lock is not None:
            self.locks.append(plan.new_lock)

        if self.state == "READY":
            self.state = "RUN"

        if plan.path_lost:
            self._path_lost_s += dt_s
        else:
            self._path_lost_s = 0.0
        if self._path_lost_s > self.config.path_lost_timeout_s and self.fault is None:
            self.fault = "PATH_LOST > 2 s"

        suggested_state = self._suggest_state(plan)
        if self.fault is not None:
            suggested_state = "CONTROLLED_STOP"
        if self.state != "STOPPED":
            self.state = suggested_state

        target_speed_mps = 0.0
        steering_rad = 0.0
        centreline_offset_m = 0.0
        if plan.blind:
            target_speed_mps = self.config.blind_creep_speed_mps
        elif plan.valid and plan.path_xy:
            xs = np.array([p[0] for p in plan.path_xy])
            ys = np.array([p[1] for p in plan.path_xy])
            xe_m, ye_m, pe_rad = pose
            c, s = cos(pe_rad), sin(pe_rad)
            world_points = [(xe_m + c * x - s * y, ye_m + s * x + c * y) for x, y in zip(xs.tolist(), ys.tolist())]
            points_tuple = tuple(
                TrajectoryPoint(timestamp_s, wx, wy, speed_mps=self.config.cruise_speed_mps) for wx, wy in world_points
            )
            trajectory = Trajectory(MessageHeader(timestamp_s, "map", self._sequence, "my_track_mission"), points_tuple, timestamp_s + 1.0)
            self._sequence += 1
            output: ControllerOutput = self.pure_pursuit.compute(_FakeState(xe_m, ye_m, pe_rad), trajectory, dt_s)
            steering_rad = output.steering_rad
            centreline_offset_m = output.cross_track_error_m

            curvature = np.abs(np.gradient(np.arctan2(np.gradient(ys), np.gradient(xs))) / 0.1)
            near_curvature = float(curvature[: min(50, len(curvature))].max()) if len(curvature) else 0.0
            target_speed_mps = self.config.cruise_speed_mps
            if near_curvature > 1e-3:
                target_speed_mps = min(target_speed_mps, sqrt(0.6 / near_curvature))
            if plan.stop_distance_m is not None:
                target_speed_mps = min(target_speed_mps, _max_speed_for_stop_distance_m(
                    plan.stop_distance_m, self.brake_delay_s, self.brake_decel_mps2, self.config.brake_margin_m))
            if plan.finish_detected and plan.finish_x_body_m is not None:
                stop_gap_m = plan.finish_x_body_m - self.front_extent_m
                target_speed_mps = min(target_speed_mps, _max_speed_for_stop_distance_m(
                    stop_gap_m, self.brake_delay_s, self.brake_decel_mps2, self.config.finish_stop_margin_m))

        if self.fault is not None or self.state == "READY":
            target_speed_mps = 0.0

        ramp_m = self.config.speed_command_ramp_mps2 * dt_s
        self._ramped_target_mps += max(-ramp_m, min(ramp_m, target_speed_mps - self._ramped_target_mps))
        longitudinal = self.speed_pid.compute(self._ramped_target_mps, speed_mps, dt_s, safety_speed_limit_mps=2.0)

        if plan.valid and plan.finish_detected and plan.finish_x_body_m is not None:
            stop_gap_m = plan.finish_x_body_m - self.front_extent_m - self.config.finish_stop_margin_m
            if stop_gap_m < 0.1 and speed_mps < 0.03:
                self.state = "STOPPED"

        return MissionCommand(
            self.state, steering_rad, longitudinal.drive, longitudinal.brake, plan.valid, plan.gates, plan.path_lost,
            bool(plan.obstacle), plan.detour_offset0_m, bool(plan.finish_detected), target_speed_mps, centreline_offset_m,
        )

    @staticmethod
    def _suggest_state(plan: PlanResult) -> str:
        if plan.path_lost:
            return "PATH_LOST"
        if not plan.valid:
            return "RUN"  # blind creep is reported as RUN per the draft's state machine
        if plan.finish_detected:
            return "FINISH_BRAKING"
        if plan.stop_distance_m is not None:
            return "CONTROLLED_STOP"
        if plan.detour:
            return "DETOUR"
        if plan.obstacle:
            return "OBSTACLE_ASSESS"
        return "RUN"


def _smoothstep(t: np.ndarray) -> np.ndarray:
    tc = np.clip(t, 0.0, 1.0)
    return tc * tc * (3 - 2 * tc)


@dataclass(frozen=True)
class _FakeState:
    """Adapts a (x, y, yaw) pose to the PlanarKartState shape PurePursuitController.compute() reads."""

    x_m: float
    y_m: float
    yaw_rad: float
    u_mps: float = 0.0
