"""Ties the my_track scenario together: PlanarBicycleKart dynamics (Prompt 1's
shared/built profiles), the HP60C camera + cone map (Step 2), the perception
->path algorithm and mission controller (Steps 3-4), and ground-truth safety
metrics (collisions, clearance, finish gap). This is the one place that
drives all of those pieces through a single time-stepped loop; nothing here
duplicates the dynamics, controllers, or algorithm primitives themselves.

Physics runs at the profile's physics_dt_s (1 kHz for shared/built), the
mission controller at 20 Hz, and the HP60C sensor at its configured rate
(4.2 Hz nominal).
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from math import cos, degrees, sin
from pathlib import Path

from .config import FullKartConfig, load_kart_config
from .hp60c import CameraFrame, ConeMap, HP60CConfig
from .hp60c import HP60CSensor
from .my_track_algorithm import footprint_clearance_m
from .my_track_layout import MyTrackLayout, load_my_track_layout
from .my_track_mission import MissionConfig, MissionController
from .types import PlanarKartState, VehicleCommand
from .vehicle import PlanarBicycleKart

CONTROL_DT_S = 0.05  # 20 Hz
CAMERA_X_OFFSET_FROM_FRONT_M = 0.15  # ASSUMED: camera sits 0.15 m behind the front bumper, per the draft
LOG_DT_S = 0.1  # 10 Hz telemetry row rate
DAC_MAX_V = 5.0  # ASSUMED, matches the draft's throttle-path convention


@dataclass
class RunConfig:
    kart_config: FullKartConfig
    camera_config: HP60CConfig
    layout: MyTrackLayout
    mission_config: MissionConfig = field(default_factory=MissionConfig)
    seed: int = 1
    cruise_speed_mps: float = 0.5
    max_duration_s: float = 400.0
    cone_map_association_radius_m: float = 0.6
    cone_map_expiry_s: float = 10.0
    detour_enabled: bool = True


@dataclass
class RunMetrics:
    scenario: str
    kart_profile: str
    outcome: str
    run_time_s: float
    collisions: int
    min_clearance_m: float
    max_steer_deg: float
    max_speed_mps: float
    stop_gap_to_finish_m: float | None
    frames_captured: int
    frames_delivered: int
    detection_recall_in_view: float


@dataclass
class RunResult:
    metrics: RunMetrics
    telemetry_rows: list[dict]
    events: list[tuple[float, str, str]]


def _axle_distances(kart_config: FullKartConfig) -> tuple[float, float]:
    wheelbase_m = kart_config.kart.wheelbase_m
    a_m = wheelbase_m / 2 - kart_config.kart.cg.x_m
    b_m = wheelbase_m - a_m
    return a_m, b_m


def run_my_track_scenario(config: RunConfig) -> RunResult:
    kart_config = config.kart_config
    _, b_m = _axle_distances(kart_config)
    start_x_m, start_y_m, start_heading_rad = config.layout.start_pose
    initial_state = PlanarKartState(
        x_m=start_x_m + b_m * cos(start_heading_rad),
        y_m=start_y_m + b_m * sin(start_heading_rad),
        yaw_rad=start_heading_rad,
    )
    kart = PlanarBicycleKart(kart_config, initial_state)

    front_extent_m = kart_config.kart.front_extent_m
    rear_overhang_m = kart_config.kart.rear_overhang_m
    half_width_m = kart_config.kart.width_m / 2
    camera_x_m = front_extent_m - CAMERA_X_OFFSET_FROM_FRONT_M
    brake_delay_s = kart_config.actuator_lag.brake.dead_time_s
    brake_decel_mps2 = kart_config.brake.max_brake_force_n / kart_config.kart.mass_kg

    sensor = HP60CSensor(config.camera_config, camera_x_m=camera_x_m, seed=config.seed)
    cone_map = ConeMap(association_radius_m=config.cone_map_association_radius_m, expiry_s=config.cone_map_expiry_s)
    algorithm_config = config.mission_config.algorithm if config.detour_enabled else replace(config.mission_config.algorithm, detour_ramp_lengths_m=())
    mission_config = replace(config.mission_config, cruise_speed_mps=config.cruise_speed_mps, algorithm=algorithm_config)
    mission = MissionController(
        mission_config, wheelbase_m=kart_config.kart.wheelbase_m, front_extent_m=front_extent_m,
        rear_overhang_m=rear_overhang_m, half_width_m=half_width_m, brake_delay_s=brake_delay_s, brake_decel_mps2=brake_decel_mps2,
    )

    cones = list(config.layout.cones)
    finish_cones = [c for c in cones if config.layout.roles.get(c.cone_id) == "finish line"]
    finish_x_world_m = sum(c.x_m for c in finish_cones) / len(finish_cones)

    dt_s = kart_config.simulation.physics_dt_s
    pending_frames: list[CameraFrame] = []
    events: list[tuple[float, str, str]] = []
    telemetry_rows: list[dict] = []

    t_s = 0.0
    next_control_s = 0.0
    next_log_s = 0.0
    odometry_m = 0.0
    hit_cone_ids: set[str] = set()
    min_clearance_m = 9.0
    max_steer_deg = 0.0
    max_speed_mps = 0.0
    frames_captured = 0
    frames_delivered = 0
    truth_visible = 0
    detections_delivered = 0
    command = VehicleCommand()
    stop_since_s: float | None = None
    outcome_state = "RUN"
    current_clearance_m = 9.0
    nearest_cone_id = ""
    nearest_corner = ""
    last_frame_age_ms = 0.0
    last_log_steer_deg = 0.0
    last_log_speed_mps = 0.0

    while t_s < config.max_duration_s:
        a_m, b_m = _axle_distances(kart_config)
        rear_axle_m = (kart.state.x_m - b_m * cos(kart.state.yaw_rad), kart.state.y_m - b_m * sin(kart.state.yaw_rad))

        frame = sensor.maybe_capture(t_s, kart.state, rear_axle_m, cones)
        if frame is not None:
            frames_captured += 1
            truth_visible += len(frame.detections)
            pending_frames.append(frame)
        still_pending = []
        for pending in pending_frames:
            if t_s >= pending.deliver_time_s:
                cone_map.update(pending)
                frames_delivered += 1
                detections_delivered += sum(1 for d in pending.detections if d.detected)
                last_frame_age_ms = (pending.deliver_time_s - pending.capture_time_s) * 1000.0
            else:
                still_pending.append(pending)
        pending_frames = still_pending

        if t_s >= next_control_s:
            next_control_s += CONTROL_DT_S

            mission_command = mission.step(
                tracks=cone_map.tracks, pose=(rear_axle_m[0], rear_axle_m[1], kart.state.yaw_rad),
                speed_mps=kart.state.u_mps, odometry_m=odometry_m, dt_s=CONTROL_DT_S, timestamp_s=t_s,
            )
            steering_rad = 0.0 if mission.state == "READY" else mission_command.steering_rad
            steering_rad = max(-kart_config.kart.max_road_wheel_angle_rad, min(kart_config.kart.max_road_wheel_angle_rad, steering_rad))
            command = VehicleCommand(mission_command.drive, mission_command.brake, steering_rad)
            outcome_state = mission.state

            if next_log_s <= t_s:
                next_log_s += LOG_DT_S
                section = "F"
                for name, (a, b) in config.layout.sections.items():
                    if a <= odometry_m < b:
                        section = name
                        break
                steer_actual_deg = degrees(kart._steer_actual_rad)
                decel_mps2 = max(0.0, (last_log_speed_mps - kart.state.u_mps) / LOG_DT_S) if kart._brake_actual > 0.02 else 0.0
                cone_map_age_s = None
                if nearest_cone_id:
                    nearest_cone = next((c for c in cones if c.cone_id == nearest_cone_id), None)
                    if nearest_cone is not None and cone_map.tracks:
                        closest_track = min(cone_map.tracks, key=lambda tr: (tr.x_m - nearest_cone.x_m) ** 2 + (tr.y_m - nearest_cone.y_m) ** 2)
                        cone_map_age_s = round(t_s - closest_track.last_seen_s, 3)
                telemetry_rows.append({
                    "t": round(t_s, 2), "dist_m": round(odometry_m, 3),
                    "x": round(rear_axle_m[0], 3), "y": round(rear_axle_m[1], 3),
                    "heading_deg": round(degrees(kart.state.yaw_rad), 2),
                    "speed_mps": round(kart.state.u_mps, 3), "speed_kmh": round(kart.state.u_mps * 3.6, 2),
                    "speed_target_mps": round(mission_command.target_speed_mps, 3),
                    "steer_cmd_deg": round(degrees(steering_rad), 2), "steer_cmd_unclamped_deg": round(degrees(mission_command.steering_rad), 2),
                    "steer_deg": round(steer_actual_deg, 2), "steer_saturated": "yes" if mission_command.steering_saturated else "no",
                    "steer_rate_dps": round((steer_actual_deg - last_log_steer_deg) / LOG_DT_S, 1),
                    "brake_cmd_pct": round(mission_command.brake * 100, 1), "brake_pct": round(kart._brake_actual * 100, 1),
                    "decel_mps2": round(decel_mps2, 2),
                    "throttle_pct": round(mission_command.drive * 100, 1), "dac_V": round(mission_command.drive * DAC_MAX_V, 2),
                    "section": section, "state": mission_command.state, "gates": mission_command.gates,
                    "map_cones": len(cone_map.tracks), "frame_age_ms": round(last_frame_age_ms, 1),
                    "centreline_offset_m": round(mission_command.centreline_offset_m, 3), "clearance_m": round(current_clearance_m, 3),
                    "obstacle": "yes" if mission_command.obstacle else "no", "detour_offset_m": round(mission_command.detour_offset_m, 2),
                    "nearest_cone_id": nearest_cone_id, "nearest_corner": nearest_corner,
                    "detour_lock_age_s": None if mission_command.active_lock_age_s is None else round(mission_command.active_lock_age_s, 2),
                    "cone_map_age_s": cone_map_age_s,
                })
                last_log_steer_deg = steer_actual_deg
                last_log_speed_mps = kart.state.u_mps

        kart.step(command, dt_s)
        odometry_m += max(kart.state.u_mps, 0.0) * dt_s
        max_speed_mps = max(max_speed_mps, kart.state.u_mps)
        max_steer_deg = max(max_steer_deg, abs(degrees(kart._steer_actual_rad)))

        if int(round(t_s / dt_s)) % 20 == 0:  # ~20 ms cadence: full-cone-set clearance is the dominant cost
            current_clearance_m = 9.0
            nearest_cone_id = ""
            nearest_corner = ""
            for cone in cones:
                clearance_m = footprint_clearance_m(cone.x_m, cone.y_m, rear_axle_m[0], rear_axle_m[1], kart.state.yaw_rad,
                                                     rear_overhang_m, front_extent_m, half_width_m) - cone.collision_radius_m
                min_clearance_m = min(min_clearance_m, clearance_m)
                if clearance_m < current_clearance_m:
                    current_clearance_m = clearance_m
                    nearest_cone_id = cone.cone_id
                    dx, dy = cone.x_m - rear_axle_m[0], cone.y_m - rear_axle_m[1]
                    c0, s0 = cos(kart.state.yaw_rad), sin(kart.state.yaw_rad)
                    bx, by = c0 * dx + s0 * dy, -s0 * dx + c0 * dy
                    nearest_corner = ("F" if bx > front_extent_m / 2 - rear_overhang_m / 2 else "R") + ("L" if by > 0 else "R")
                if clearance_m < 0.0 and cone.cone_id not in hit_cone_ids:
                    hit_cone_ids.add(cone.cone_id)
                    events.append((t_s, "COLLISION", f"kart footprint touched cone {cone.cone_id}"))

        front_bumper_x_m = rear_axle_m[0] + front_extent_m * cos(kart.state.yaw_rad)
        if outcome_state == "STOPPED":
            if stop_since_s is None:
                stop_since_s = t_s
                events.append((t_s, "STOP", "mission reports stopped"))
            if t_s - stop_since_s > 3.0:
                break
        elif outcome_state == "CONTROLLED_STOP" and mission.fault is not None and kart.state.u_mps < 0.06:
            # Only a genuine fault (PATH_LOST timeout) ends the run here.
            # CONTROLLED_STOP can also mean "temporarily stopped in front of
            # an obstacle while detour admission keeps retrying" (no fault
            # set) -- that can still resolve on a later cycle (see the
            # detour-lock fix), so it must not be cut off just because speed
            # dipped low during a retry. See the matching note in
            # my_track_mission.py for the 0.06 vs. the model's 0.05 floor.
            if stop_since_s is None:
                stop_since_s = t_s
            if t_s - stop_since_s > 2.0:
                events.append((t_s, "END", "controlled stop reached, run ended"))
                break
        elif front_bumper_x_m > finish_x_world_m + 3.0:
            # Driven well past the finish without ever stopping there
            # (overshoot) -- no point running to max_duration_s.
            events.append((t_s, "END", "overshot the finish without stopping"))
            break
        else:
            stop_since_s = None

        t_s += dt_s

    # COMPLETE is a ground-truth check against the real finish line position,
    # not whether the mission's own (fallible) finish detection fired --
    # several early runs reported COMPLETE via a stale/mis-latched finish
    # belief while the front bumper was actually 6-7 m short of the real
    # line; see docs/MY_TRACK.md.
    front_bumper_x_m = rear_axle_m[0] + front_extent_m * cos(kart.state.yaw_rad)
    stop_gap_to_finish_m = finish_x_world_m - front_bumper_x_m
    stopped = kart.state.u_mps < 0.06
    within_1_5m_before_finish = 0.0 <= stop_gap_to_finish_m <= 1.5
    no_collision = len(hit_cone_ids) == 0
    no_fault = mission.fault is None

    if stopped and within_1_5m_before_finish and no_collision and no_fault:
        outcome = "COMPLETE"
    elif not no_collision:
        outcome = f"ABORTED: collision ({len(hit_cone_ids)} cone(s))"
    elif stop_gap_to_finish_m < 0.0:
        outcome = f"ABORTED: overshot finish by {-stop_gap_to_finish_m:.2f} m"
    elif not no_fault:
        outcome = f"ABORTED: {mission.fault}"
    elif not stopped:
        outcome = f"ABORTED: did not stop (speed {kart.state.u_mps:.3f} m/s at run end, state {outcome_state})"
    else:
        outcome = f"ABORTED: stopped {stop_gap_to_finish_m:.2f} m short of the finish (state {outcome_state})"

    metrics = RunMetrics(
        "my_track", kart_config.kart.name, outcome, round(t_s, 1), len(hit_cone_ids), round(min_clearance_m, 3),
        round(max_steer_deg, 1), round(max_speed_mps, 3), round(stop_gap_to_finish_m, 3),
        frames_captured, frames_delivered, round(detections_delivered / max(truth_visible, 1), 3),
    )
    return RunResult(metrics, telemetry_rows, events)


def default_run_config(profile: str, *, seed: int = 1, cruise_speed_mps: float = 0.5, detour_enabled: bool = True) -> RunConfig:
    repo_kart_dir = Path(__file__).parents[1] / "configs/karts"
    sensors_dir = Path(__file__).parents[1] / "configs/sensors"
    spec_dir = Path(__file__).parents[2] / "docs/my_track_spec"
    from .hp60c import load_hp60c_config
    return RunConfig(
        kart_config=load_kart_config(repo_kart_dir / f"{profile}.yaml"),
        camera_config=load_hp60c_config(sensors_dir / "hp60c.yaml"),
        layout=load_my_track_layout(spec_dir / "my_track_layout.json"),
        seed=seed, cruise_speed_mps=cruise_speed_mps, detour_enabled=detour_enabled,
    )
