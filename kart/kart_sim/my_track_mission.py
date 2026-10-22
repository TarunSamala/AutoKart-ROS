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
from math import cos, hypot, sin, sqrt

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
    # Obstacle-classification hysteresis: once an orange cone is seen as part
    # of an obstacle cluster, keep treating it as one for this long even if a
    # later cycle's flood-fill doesn't reform the cluster. See the longer
    # note by _obstacle_memory in MissionController.__init__.
    obstacle_memory_s: float = 3.0
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
    steering_saturated: bool = False
    active_lock_age_s: float | None = None


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
        # Latched once the finish line is first detected, same anti-flicker
        # principle as the detour lock: the map's noisy, partially-merged
        # finish-cone positions can cause detect_finish() to intermittently
        # pass/fail frame to frame right at the finish, which would otherwise
        # make the kart swing between braking and cruising instead of
        # committing to a smooth stop.
        self._finish_world_xy_m: tuple[float, float] | None = None
        # Obstacle-cluster memory: classify_obstacles() re-derives the
        # cluster fresh every 0.05 s cycle via flood-fill on the *current*
        # map-blended positions, with no temporal persistence. A link
        # distance right at the 0.85 m threshold can flicker the whole
        # cluster in and out of existence between consecutive cycles even
        # though the same cones are still there (confirmed by tracing a real
        # collision: classify_obstacles returned a 5-cone cluster, then an
        # empty set 0.05 s later, and never recovered before contact). Once
        # any orange cone is seen as part of an obstacle, its world position
        # is remembered for obstacle_memory_s seconds so a momentary flicker
        # doesn't make the kart drive straight through it.
        self._obstacle_memory: dict[int, float] = {}  # track_id -> last_confirmed_s

    def reset_speed_ramp(self) -> None:
        self._ramped_target_mps = 0.0
        self.speed_pid.reset()

    def _plan(self, tracks: list[TrackedCone], pose: tuple[float, float, float], speed_mps: float, odometry_m: float, timestamp_s: float) -> PlanResult:
        cfg = self.config.algorithm
        xe_m, ye_m, pe_rad = pose
        c, s = cos(pe_rad), sin(pe_rad)

        def to_world(x_body_m: float, y_body_m: float) -> tuple[float, float]:
            return xe_m + c * x_body_m - s * y_body_m, ye_m + s * x_body_m + c * y_body_m

        points = body_frame_points(tracks, pose, cfg)

        finish_x, finish_indices = (None, set())
        if len(points) >= 3:
            finish_x, finish_indices = detect_finish(points, odometry_m, cfg)
        if finish_x is not None and self._finish_world_xy_m is None:
            self._finish_world_xy_m = to_world(finish_x, 0.0)
        if self._finish_world_xy_m is not None:
            # Recompute the body-frame forward distance to the latched world
            # point every cycle, rather than re-detecting (and risking the
            # same flicker the detour lock avoids) or reusing a stale
            # body-frame value as the kart moves.
            wx_m, wy_m = self._finish_world_xy_m
            finish_x = c * (wx_m - xe_m) + s * (wy_m - ye_m)
            # Once the finish is latched, commit to a straight final approach
            # rather than falling back to blind/PATH_LOST if the boundary
            # thins out in the last few metres (fewer cones remain ahead
            # that close to the track's end) -- the lane-following gates are
            # no longer the limiting factor this close to a known stop point.
            approach_path = tuple((x, 0.0) for x in np.linspace(0.0, max(finish_x, 0.5), 20))
            return PlanResult(
                valid=True, gates=len(points), path_xy=approach_path, obstacle=False, detour=False,
                detour_offset0_m=0.0, finish_detected=True, finish_x_body_m=finish_x,
                blind=False, path_lost=False, stop_distance_m=None, new_lock=None,
            )
        if len(points) < 3:
            return self._blind(odometry_m, gates=0)

        obstacle_indices = classify_obstacles(points, finish_indices, cfg)

        # Hysteresis: remember which TRACK IDs were just confirmed as
        # obstacles, then pull back in any orange cone whose track id is
        # still fresh in memory, even if this cycle's flood-fill alone
        # didn't flag it. Keyed by track id rather than a snapshotted world
        # position: the cone map keeps refining a track's position as more
        # detections arrive (especially soon after it's first confirmed),
        # so a position snapshot can drift outside a fixed match radius
        # before the memory itself expires -- confirmed by tracing a real
        # collision where the memory was still non-empty and well within
        # its time window, but no entry matched the drifted position.
        for i in obstacle_indices:
            self._obstacle_memory[points[i][3]] = timestamp_s
        self._obstacle_memory = {tid: t for tid, t in self._obstacle_memory.items() if timestamp_s - t <= self.config.obstacle_memory_s}
        for i, (x_m, y_m, colour, track_id) in enumerate(points):
            if i in obstacle_indices or colour not in ("large_orange", "orange") or i in finish_indices:
                continue
            if track_id in self._obstacle_memory:
                obstacle_indices = obstacle_indices | {i}

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
        active_lock_age_s: float | None = None
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

            cluster_centre_world_xy = to_world(float(cx.mean()), float(cy.mean()))
            existing_lock = next(
                (lock for lock in self.locks if hypot(lock.cluster_centre_world_xy[0] - cluster_centre_world_xy[0],
                                                        lock.cluster_centre_world_xy[1] - cluster_centre_world_xy[1]) < 2.0),
                None,
            )
            if existing_lock is not None:
                # A detour already admitted for this cluster is reused verbatim
                # rather than re-searched: re-searching as the kart gets closer
                # (and the geometry gets less favourable) can flip admission
                # off mid-maneuver and snap the path back toward the obstacle.
                admitted, chosen_ramp_m, chosen_lead_m = True, existing_lock.ramp_length_m, existing_lock.lead_distance_m
                active_lock_age_s = timestamp_s - existing_lock.created_at_s
            elif free_width_m >= self.half_width_m * 2 + 2 * 0.25:
                for ramp_m in cfg.detour_ramp_lengths_m:
                    lead_m = 0.0
                    while x_min_m - lead_m - ramp_m >= -0.2:
                        ramp_offset, _ = self._ramp(xs, x_min_m, x_max_m, lead_m, ramp_m, ramp_height_m)
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
                            new_lock = DetourLock(cluster_centre_world_xy, ramp_m, lead_m, timestamp_s)
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
            active_lock_age_s=active_lock_age_s,
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
              dt_s: float, timestamp_s: float) -> MissionCommand:
        plan = self._plan(tracks, pose, speed_mps, odometry_m, timestamp_s)
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
            # PlanarBicycleKart disables brake force below LOW_SPEED_MPS
            # (0.05 m/s, see vehicle.py) and coasts there indefinitely rather
            # than reaching exactly zero; 0.03 never fires. 0.06 is just above
            # that floor.
            if stop_gap_m < 0.1 and speed_mps < 0.06:
                self.state = "STOPPED"

        steering_saturated = abs(steering_rad) >= self.pure_pursuit.maximum_steering_rad - 1e-6
        return MissionCommand(
            self.state, steering_rad, longitudinal.drive, longitudinal.brake, plan.valid, plan.gates, plan.path_lost,
            bool(plan.obstacle), plan.detour_offset0_m, bool(plan.finish_detected), target_speed_mps, centreline_offset_m,
            steering_saturated, plan.active_lock_age_s,
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
