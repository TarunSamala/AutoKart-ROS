"""Read-only mission state and stop-envelope adapter over observed shared-core geometry."""

from __future__ import annotations

from math import asin, atan2, cos, hypot, isfinite, pi, sin, tan

from autokart_perception.shared_core import BrakeCapability, PoseEstimate
from autokart_vehicle_geometry.geometry import GEOMETRY


def _wrap(angle: float) -> float:
    return (angle + pi) % (2 * pi) - pi


def _polyline_closest(x: float, y: float,
                      points: list[tuple[float, float]]) -> tuple[float, float, float]:
    nearest = (float("inf"), 0.0, 0.0)
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        if length_sq <= 0:
            continue
        fraction = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length_sq))
        px, py = ax + fraction * dx, ay + fraction * dy
        candidate = (hypot(x - px, y - py), px, py)
        if candidate[0] < nearest[0]:
            nearest = candidate
    return nearest


class SkidpadProgress:
    """Gate-confirmed right/right/left/left route progress for observed rings."""

    def __init__(self):
        self.state = "ENTRY"
        self.right_laps = 0
        self.left_laps = 0
        self.previous_angle = None
        self.travel = 0.0
        self.previous_stamp = -1

    def update(self, pose: PoseEstimate, topology: dict, stop_feedback_valid: bool = False) -> list[str]:
        if self.state in {"STOP", "HOLD"}:
            return ["MISSION_FINISHED"] if self.state == "STOP" else ["MISSION_HOLD"]
        if self.state == "STOP_PENDING":
            if stop_feedback_valid and pose.speed_mps <= 0.1:
                self.state = "STOP"
                return []
            return ["STOP_FEEDBACK_PENDING"]
        if self.state == "EXIT":
            return []
        if self.state == "TRANSITION":
            left, right = topology["left"], topology["right"]
            axis = topology["separation_axis_xy"]
            midpoint = ((left["center_x_m"] + right["center_x_m"]) / 2,
                        (left["center_y_m"] + right["center_y_m"]) / 2)
            projected = ((pose.x_m - midpoint[0]) * axis[0] +
                         (pose.y_m - midpoint[1]) * axis[1])
            left_error = abs(hypot(pose.x_m - left["center_x_m"],
                                   pose.y_m - left["center_y_m"]) - left["radius_m"])
            if projected < -0.1 and left_error <= min(1.0, left["track_width_m"] / 3):
                self.state = "LEFT_1"
                self.previous_angle = None
                self.travel = 0.0
            else:
                return ["TRANSITION_PENDING"]
        branch = "right" if self.state in {"ENTRY", "RIGHT_1", "RIGHT_2", "TRANSITION"} else "left"
        ring = topology[branch]
        center = (ring["center_x_m"], ring["center_y_m"])
        radius = hypot(pose.x_m - center[0], pose.y_m - center[1])
        if abs(radius - ring["radius_m"]) > min(1.0, ring["track_width_m"] / 3):
            return ["WRONG_LOBE_OR_OFF_TRACK"]
        angle = atan2(pose.y_m - center[1], pose.x_m - center[0])
        direction = -1 if branch == "right" else 1
        gate = pi if branch == "right" else 0.0
        phase = (direction * (angle - gate)) % (2 * pi)
        if self.state == "ENTRY":
            if phase > 0.35:
                return ["ENTRY_GATE_NOT_REACHED"]
            self.state = "RIGHT_1"
        if pose.timestamp_ns <= self.previous_stamp:
            self.state = "HOLD"
            return ["NONMONOTONE_MISSION_POSE"]
        self.previous_stamp = pose.timestamp_ns
        if self.previous_angle is not None:
            delta = direction * _wrap(angle - self.previous_angle)
            if delta < -0.08 or delta > pi / 4:
                self.state = "HOLD"
                return ["WRONG_DIRECTION_OR_POSE_JUMP"]
            self.travel += max(0.0, delta)
            previous_phase = (direction * (self.previous_angle - gate)) % (2 * pi)
            crossing = previous_phase > 5.5 and phase < 0.7
            if crossing:
                if self.travel < 2 * pi - 0.25:
                    self.state = "HOLD"
                    return ["PREMATURE_OR_DUPLICATE_GATE"]
                if branch == "right":
                    self.right_laps += 1
                    self.state = "RIGHT_2" if self.right_laps == 1 else "TRANSITION"
                else:
                    self.left_laps += 1
                    self.state = "LEFT_2" if self.left_laps == 1 else "EXIT"
                self.travel = 0.0
                self.previous_angle = None
                return []
        self.previous_angle = angle
        return []

    def snapshot(self):
        return {"state": self.state, "right_laps": self.right_laps,
                "left_laps": self.left_laps, "progress_turns": self.travel / (2 * pi)}


class PlannerEngine:
    def __init__(self, mission: str, source: str, brake: BrakeCapability | None,
                 *, speed_limit_mps: float = 1.0, footprint_width_m: float | None = None,
                 camera_to_bumper_m: float | None = None, stop_gap_from_bumper_m: float | None = None,
                 lateral_margin_m: float | None = None, dead_reckoning_margin_rate: float = 0.5,
                 dead_reckoning_max_m: float = 2.0):
        if mission not in {"SKIDPAD", "BRAKING_TEST"}:
            raise ValueError("unsupported mission")
        self.mission, self.source, self.brake = mission, source, brake
        self.speed_limit_mps = speed_limit_mps
        # All of these default from the single geometry source (kart/ros2_ws/src/autokart_bringup/config/
        # vehicle_geometry.yaml via autokart_vehicle_geometry) rather than a second hardcoded literal here.
        self.lateral_margin_m = GEOMETRY.lateral_margin_m if lateral_margin_m is None else lateral_margin_m
        self.footprint_width_m = (GEOMETRY.bumper_width_m + 2.0 * self.lateral_margin_m) \
            if footprint_width_m is None else footprint_width_m
        self.camera_to_bumper_m = GEOMETRY.camera_to_front_bumper_horizontal_m \
            if camera_to_bumper_m is None else camera_to_bumper_m
        self.stop_gap_from_bumper_m = GEOMETRY.stop_gap_from_bumper_m \
            if stop_gap_from_bumper_m is None else stop_gap_from_bumper_m
        if not isfinite(self.stop_gap_from_bumper_m) or self.stop_gap_from_bumper_m < GEOMETRY.stop_gap_from_bumper_m:
            raise ValueError("stop gap must be finite and at least the 0.15 m red zone")
        self.corridor_minimum_width_m = GEOMETRY.bumper_width_m + 2.0 * self.lateral_margin_m
        # Dead reckoning: once a cone is no longer in the obstacles list (e.g. it left the depth-valid field
        # of view near the stop point, see camera_visibility.py), the last sighting is advanced by the
        # odometry-measured distance travelled since, with an extra margin that GROWS with that distance
        # (dead_reckoning_margin_rate, unitless: metres of extra margin per metre travelled blind). Beyond
        # dead_reckoning_max_m of blind travel the extrapolation is no longer trusted and this mission fails
        # safe to STOP rather than continuing to guess.
        self.dead_reckoning_margin_rate = dead_reckoning_margin_rate
        self.dead_reckoning_max_m = dead_reckoning_max_m
        self._last_obstacle_sighting: tuple[float, float, float, float, float] | None = None
        self.progress = SkidpadProgress()
        self.last_stamp = -1
        self.last_exit_along = None

    def reset(self):
        self.progress = SkidpadProgress()
        self.last_stamp = -1
        self.last_exit_along = None
        self._last_obstacle_sighting = None

    def reject(self, reason: str, stamp: int) -> dict:
        self.reset()
        return {"valid": False, "reason_codes": [reason], "path_odom": [],
                "target_speed_mps": 0.0, "decision": "HOLD", "sample_time_ns": stamp,
                "mission_state": self.progress.snapshot(), "stop_point_m": None}

    def evaluate(self, mapping: dict, pose: PoseEstimate, obstacles: list[dict] | None = None,
                 stop_feedback_valid: bool = False) -> dict:
        stamp = int(mapping.get("sample_time_ns", -1))
        if (not mapping.get("valid") or mapping.get("source") != self.source or
                mapping.get("frame_id") != "odom" or stamp <= self.last_stamp or
                not pose.fresh(stamp, 150_000_000)):
            return self.reject("MAP_OR_ODOM_INVALID", stamp)
        if mapping.get("tracked_count") == 0:
            return self.reject("NO_VALID_MAP_CONES", stamp)
        if (self.brake is None or not self.brake.valid_for(self.source) or
                not isfinite(self.speed_limit_mps) or self.speed_limit_mps <= 0 or
                not isfinite(self.footprint_width_m) or self.footprint_width_m <= 0):
            return self.reject("BRAKE_OR_VEHICLE_LIMIT_UNQUALIFIED", stamp)
        self.last_stamp = stamp
        if self.mission == "SKIDPAD":
            return self._skidpad(mapping, pose, stamp, stop_feedback_valid)
        return self._braking(mapping, pose, stamp, obstacles or [])

    def _skidpad(self, mapping: dict, pose: PoseEstimate, stamp: int,
                 stop_feedback_valid: bool) -> dict:
        topology = mapping.get("geometry", {}).get("skidpad_topology")
        if not isinstance(topology, dict) or not topology.get("valid"):
            return self.reject("OBSERVED_LOOPS_UNAVAILABLE", stamp)
        left, right = topology["left"], topology["right"]
        gap = hypot(left["center_x_m"] - right["center_x_m"],
                    left["center_y_m"] - right["center_y_m"]) - left["radius_m"] - right["radius_m"]
        if (abs(gap) > 1.0 or
                right["boundaries"]["blue"]["radius_m"] <= right["boundaries"]["yellow"]["radius_m"] or
                left["boundaries"]["blue"]["radius_m"] >= left["boundaries"]["yellow"]["radius_m"]):
            return self.reject("AMBIGUOUS_CROSSOVER_OR_COLORS", stamp)
        reasons = self.progress.update(pose, topology, stop_feedback_valid)
        state = self.progress.state
        if state == "EXIT":
            corridor = mapping.get("geometry", {}).get("exit_centerline_odom", [])
            if len(corridor) < 3:
                return {"valid": False, "reason_codes": ["EXIT_CORRIDOR_UNOBSERVED"],
                        "path_odom": [], "target_speed_mps": 0.0, "decision": "HOLD",
                        "sample_time_ns": stamp, "mission_state": self.progress.snapshot(),
                        "stop_point_m": None}
            points = [(float(x), float(y)) for x, y in corridor]
            if any(not all(isfinite(value) for value in point) for point in points):
                return self.reject("EXIT_CORRIDOR_INVALID", stamp)
            ex, ey = topology["separation_axis_xy"]
            tx, ty = -ey, ex
            origin = ((left["center_x_m"] + right["center_x_m"]) / 2,
                      (left["center_y_m"] + right["center_y_m"]) / 2)
            along = (pose.x_m - origin[0]) * tx + (pose.y_m - origin[1]) * ty
            lateral = (pose.x_m - origin[0]) * ex + (pose.y_m - origin[1]) * ey
            forwards = [(x - origin[0]) * tx + (y - origin[1]) * ty for x, y in points]
            if (any(b - a < 0.8 or b - a > 4.0 for a, b in zip(forwards, forwards[1:])) or
                    along < -0.5 or (self.last_exit_along is not None and along < self.last_exit_along - 0.15)):
                return {"valid": False, "reason_codes": ["EXIT_PROGRESS_INVALID"],
                        "path_odom": [], "target_speed_mps": 0.0, "decision": "HOLD",
                        "sample_time_ns": stamp, "mission_state": self.progress.snapshot(),
                        "stop_point_m": None}
            nearest = min(range(len(points)), key=lambda index: hypot(pose.x_m - points[index][0],
                                                                       pose.y_m - points[index][1]))
            on_connector = along <= forwards[0] and abs(lateral) <= 1.5
            if not on_connector and hypot(pose.x_m - points[nearest][0], pose.y_m - points[nearest][1]) > 2.0:
                return {"valid": False, "reason_codes": ["EXIT_POSE_OFF_CORRIDOR"],
                        "path_odom": [], "target_speed_mps": 0.0, "decision": "HOLD",
                        "sample_time_ns": stamp, "mission_state": self.progress.snapshot(),
                        "stop_point_m": None}
            self.last_exit_along = along if self.last_exit_along is None else max(self.last_exit_along, along)
            remaining = points[nearest:]
            if on_connector and along < forwards[0] - 0.5:
                steps = max(1, int((forwards[0] - along) / 0.5))
                remaining = [(origin[0] + (points[0][0] - origin[0]) * (along + (forwards[0] - along) * i / steps) / forwards[0],
                              origin[1] + (points[0][1] - origin[1]) * (along + (forwards[0] - along) * i / steps) / forwards[0])
                             for i in range(1, steps)] + points
            distance_to_end = hypot(pose.x_m - points[-1][0], pose.y_m - points[-1][1])
            if distance_to_end <= 0.6 and pose.speed_mps <= 0.1:
                self.progress.state = "STOP_PENDING"
                if stop_feedback_valid:
                    self.progress.state = "STOP"
                return {"valid": False,
                        "reason_codes": [] if self.progress.state == "STOP" else ["STOP_FEEDBACK_PENDING"],
                        "path_odom": [], "target_speed_mps": 0.0,
                        "decision": "STOP" if self.progress.state == "STOP" else "HOLD",
                        "sample_time_ns": stamp, "mission_state": self.progress.snapshot(),
                        "stop_point_m": None}
            if distance_to_end <= self.brake.stopping_distance(pose.speed_mps) + 0.5:
                return {"valid": False, "reason_codes": ["EXIT_TERMINAL_BRAKE"],
                        "path_odom": [], "target_speed_mps": 0.0, "decision": "STOP_CANDIDATE",
                        "sample_time_ns": stamp, "mission_state": self.progress.snapshot(),
                        "stop_point_m": points[-1], "stop_point_frame": "odom"}
            return {"valid": True, "reason_codes": [], "path_odom": remaining,
                    "target_speed_mps": min(self.speed_limit_mps, 0.5),
                    "decision": "EXIT_PATH_CANDIDATE", "sample_time_ns": stamp,
                    "mission_state": self.progress.snapshot(), "stop_point_m": points[-1],
                    "stop_point_frame": "odom"}
        if state == "TRANSITION" and reasons == ["TRANSITION_PENDING"]:
            transition = self._transition_path(topology, pose)
            if transition:
                return {"valid": True, "reason_codes": [], "path_odom": transition,
                        "target_speed_mps": min(self.speed_limit_mps, 0.5),
                        "decision": "TRANSITION_PATH_CANDIDATE", "sample_time_ns": stamp,
                        "mission_state": self.progress.snapshot(), "stop_point_m": None}
        if reasons or state in {"TRANSITION", "STOP", "STOP_PENDING", "HOLD"}:
            return {"valid": False, "reason_codes": reasons or [state], "path_odom": [],
                    "target_speed_mps": 0.0, "decision": "HOLD" if state != "STOP" else "STOP",
                    "sample_time_ns": stamp, "mission_state": self.progress.snapshot(),
                    "stop_point_m": None}
        branch = "right" if state.startswith("RIGHT") else "left"
        ring = topology[branch]
        direction = -1 if branch == "right" else 1
        angle = atan2(pose.y_m - ring["center_y_m"], pose.x_m - ring["center_x_m"])
        points = [(ring["center_x_m"] + ring["radius_m"] * cos(angle + direction * i * 0.06),
                   ring["center_y_m"] + ring["radius_m"] * sin(angle + direction * i * 0.06))
                  for i in range(1, 17)]
        return {"valid": True, "reason_codes": [], "path_odom": points,
                "target_speed_mps": min(self.speed_limit_mps, 1.0), "decision": "PATH_CANDIDATE",
                "sample_time_ns": stamp, "mission_state": self.progress.snapshot(),
                "stop_point_m": None}

    def _transition_path(self, topology: dict, pose: PoseEstimate) -> list[tuple[float, float]]:
        """Cubic Hermite crossover between observed circle tangents, for SIL only."""
        right, left = topology["right"], topology["left"]
        ex, ey = topology["separation_axis_xy"]
        tx, ty = -ey, ex
        center = ((right["center_x_m"] + left["center_x_m"]) / 2,
                  (right["center_y_m"] + left["center_y_m"]) / 2)
        along = (pose.x_m - center[0]) * tx + (pose.y_m - center[1]) * ty
        if not -0.6 <= along <= 2.0 or hypot(pose.x_m - center[0], pose.y_m - center[1]) > 2.2:
            return []
        length = min(2.0, 0.2 * right["radius_m"], 0.2 * left["radius_m"])
        if length <= 0:
            return []
        right_angle = asin(length / right["radius_m"])
        left_angle = asin(length / left["radius_m"])
        right_x = ((right["center_x_m"] - center[0]) * ex +
                   (right["center_y_m"] - center[1]) * ey -
                   right["radius_m"] * cos(right_angle))
        left_x = ((left["center_x_m"] - center[0]) * ex +
                  (left["center_y_m"] - center[1]) * ey +
                  left["radius_m"] * cos(left_angle))
        right_slope, left_slope = -tan(right_angle), -tan(left_angle)
        points = []
        start = max(-length, along)
        for index in range(1, 17):
            y = start + (length - start) * index / 16
            u = (y + length) / (2 * length)
            h00 = 2 * u**3 - 3 * u**2 + 1
            h10 = u**3 - 2 * u**2 + u
            h01 = -2 * u**3 + 3 * u**2
            h11 = u**3 - u**2
            x = (h00 * right_x + h10 * 2 * length * right_slope +
                 h01 * left_x + h11 * 2 * length * left_slope)
            points.append((center[0] + x * ex + y * tx,
                           center[1] + x * ey + y * ty))
        for index in range(1, 9):
            a = left_angle + index * 0.05
            x = ((left["center_x_m"] - center[0]) * ex +
                 (left["center_y_m"] - center[1]) * ey + left["radius_m"] * cos(a))
            y = left["radius_m"] * sin(a)
            points.append((center[0] + x * ex + y * tx,
                           center[1] + x * ey + y * ty))
        if hypot(points[0][0] - pose.x_m, points[0][1] - pose.y_m) > 1.0:
            return []
        return points

    def _braking(self, mapping: dict, pose: PoseEstimate, stamp: int, obstacles: list[dict]) -> dict:
        """All stop logic here is measured at the FRONT BUMPER, not the camera: cone/obstacle coordinates
        arrive in the perception/mapping contract's frame (camera-referenced forward distance); every
        distance used to decide whether to stop is first converted with self.camera_to_bumper_m before the
        stopping-distance formula (reused from BrakeCapability.stopping_distance: v*t_latency + v^2/(2a) +
        margin) is applied. See docs/VCU_PROTOCOL.md and kart/ros2_ws/src/autokart_bringup/config/
        vehicle_geometry.yaml.
        """
        centerline = mapping.get("geometry", {}).get("centerline", [])
        if len(centerline) < 3 or mapping.get("geometry", {}).get("centerline_status") != "PROVISIONAL":
            return self.reject("STRAIGHT_CORRIDOR_UNAVAILABLE", stamp)
        if any(not all(isfinite(float(v)) for v in point) for point in centerline):
            return self.reject("INVALID_PATH_GEOMETRY", stamp)

        # Corridor clearance: abort/STOP if the gap between the boundary cones is narrower than the bumper
        # plus the lateral margin on each side. Optional: mapping does not yet always populate
        # corridor_width_m for this mission (a known gap, see docs/BENCH_TESTS.md); when absent this check
        # is simply skipped rather than inventing a value.
        corridor_width = mapping.get("geometry", {}).get("corridor_width_m")
        if corridor_width is not None:
            corridor_width = float(corridor_width)
            if not isfinite(corridor_width) or corridor_width < self.corridor_minimum_width_m:
                return {"valid": False, "reason_codes": ["CORRIDOR_NARROWER_THAN_BUMPER"], "path_odom": [],
                        "target_speed_mps": 0.0, "decision": "STOP_CANDIDATE", "sample_time_ns": stamp,
                        "mission_state": {"state": "BRAKE_PENDING"}, "stop_point_m": None,
                        "stopping_distance_m": None}

        horizon = max(float(point[0]) for point in centerline)
        stop_distance = self.brake.stopping_distance(pose.speed_mps)
        if horizon < stop_distance:
            return self.reject("STOP_HORIZON_INSUFFICIENT", stamp)

        # Dead reckoning: the cone may have left the camera's depth-valid field of view near the stop point
        # (see camera_visibility.py) -- obstacles is then empty even though the cone is still physically
        # there. If we have a prior sighting, advance it by the odometry-measured distance travelled since,
        # with an extra margin that grows with that distance. Too much blind travel and this fails safe to
        # STOP rather than keep extrapolating.
        effective_obstacles = list(obstacles)
        visible_stop_cone = False
        for item in obstacles:
            try:
                x, y, radius = (float(item[key]) for key in ("x_m", "y_m", "radius_m"))
            except (KeyError, TypeError, ValueError):
                return self.reject("INVALID_OBSTACLE", stamp)
            if not all(isfinite(v) for v in (x, y, radius)) or radius < 0:
                return self.reject("INVALID_OBSTACLE", stamp)
            distance, _, _ = _polyline_closest(x, y, centerline)
            visible_stop_cone |= x > 0 and distance <= radius + self.footprint_width_m / 2
        dead_reckoned_extra_margin_m = 0.0
        used_dead_reckoning = False
        if not visible_stop_cone and self._last_obstacle_sighting is not None:
            last_x, last_y, last_radius, last_pose_x, last_pose_y = self._last_obstacle_sighting
            visible_at_stop_m = self.camera_to_bumper_m + self.stop_gap_from_bumper_m
            if last_x - last_radius > visible_at_stop_m:
                return {"valid": False, "reason_codes": ["STOP_CONE_LOST_BEYOND_VISIBLE_STOP_RANGE"],
                        "path_odom": [], "target_speed_mps": 0.0, "decision": "STOP_CANDIDATE",
                        "sample_time_ns": stamp, "mission_state": {"state": "BRAKE_PENDING"},
                        "stop_point_m": None, "stopping_distance_m": stop_distance,
                        "cone_forward_distance_m": last_x - last_radius,
                        "bumper_to_cone_m": last_x - last_radius - self.camera_to_bumper_m,
                        "dead_reckoned": True}
            travelled = hypot(pose.x_m - last_pose_x, pose.y_m - last_pose_y)
            if travelled > self.dead_reckoning_max_m:
                return {"valid": False, "reason_codes": ["DEAD_RECKONING_EXPIRED"], "path_odom": [],
                        "target_speed_mps": 0.0, "decision": "STOP_CANDIDATE", "sample_time_ns": stamp,
                        "mission_state": {"state": "BRAKE_PENDING"}, "stop_point_m": None,
                        "stopping_distance_m": None}
            effective_obstacles.append({"x_m": last_x - travelled, "y_m": last_y, "radius_m": last_radius})
            dead_reckoned_extra_margin_m = self.dead_reckoning_margin_rate * travelled
            used_dead_reckoning = True

        blocking = []
        for item in effective_obstacles:
            try:
                x, y, radius = (float(item[key]) for key in ("x_m", "y_m", "radius_m"))
            except (KeyError, TypeError, ValueError):
                return self.reject("INVALID_OBSTACLE", stamp)
            if not all(isfinite(v) for v in (x, y, radius)) or radius < 0:
                return self.reject("INVALID_OBSTACLE", stamp)
            distance, _, path_y = _polyline_closest(x, y, centerline)
            if x > 0 and distance <= radius + self.footprint_width_m / 2:
                blocking.append((x, path_y, radius))
        if blocking and obstacles:
            nearest_x, nearest_y, nearest_radius = min(blocking)
            self._last_obstacle_sighting = (nearest_x, nearest_y, nearest_radius, pose.x_m, pose.y_m)
        if blocking:
            x, y, radius = min(blocking)
            # Bumper-referenced: nearest edge of the cone, minus the camera-to-bumper offset, minus the
            # stop gap (and any dead-reckoning extra margin), compared against the physics-based stopping
            # distance -- not an immediate stop the moment any cone is merely observed ahead.
            bumper_to_cone = (x - radius) - self.camera_to_bumper_m
            available = bumper_to_cone - self.stop_gap_from_bumper_m - dead_reckoned_extra_margin_m
            if available <= stop_distance:
                stop_x = max(0.0, x - radius - stop_distance)  # reported in the received (camera) frame
                return {"valid": False, "reason_codes": ["CORRIDOR_BLOCKED"], "path_odom": [],
                        "target_speed_mps": 0.0, "decision": "STOP_CANDIDATE",
                        "sample_time_ns": stamp, "mission_state": {"state": "BRAKE_PENDING"},
                        "stop_point_m": [stop_x, y], "stopping_distance_m": stop_distance,
                        "bumper_available_m": available, "dead_reckoned": used_dead_reckoning,
                        "cone_forward_distance_m": x - radius, "bumper_to_cone_m": bumper_to_cone}
            # Obstacle observed but still farther than the required stopping distance: keep cruising.
        c, s = cos(pose.yaw_rad), sin(pose.yaw_rad)
        points = [(pose.x_m + c * x - s * y, pose.y_m + s * x + c * y)
                  for x, y in centerline]
        return {"valid": True, "reason_codes": [], "path_odom": points,
                "target_speed_mps": min(self.speed_limit_mps, 1.0), "decision": "PATH_CANDIDATE",
                "sample_time_ns": stamp, "mission_state": {"state": "BRAKING_APPROACH"},
                "stop_point_m": None, "stopping_distance_m": stop_distance,
                "cone_forward_distance_m": min(blocking)[0] - min(blocking)[2] if blocking else None,
                "bumper_to_cone_m": min(blocking)[0] - min(blocking)[2] - self.camera_to_bumper_m if blocking else None}
