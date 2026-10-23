"""One nonauthorizing autonomy algorithm path for CORE and LAB.

Coordinates are SI: cone observations are base_link X-forward/Y-left; pose and
tracked cones are in odom. Outputs are diagnostic candidates only. Nothing in
this module can publish a vehicle command or grant motion authority.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from hashlib import sha256
import json
from math import atan2, cos, hypot, isfinite, pi, sin
from typing import Iterable

import numpy as np

from .boundaries import estimate_boundaries
from .centerline import generate_centerline
from .curvature import profile_centerline
from .gabriel import filter_gabriel
from .gates import candidate_gates, select_track_gates
from .geometry import triangulate
from .safety import topology_reasons
from .types import Cone


ALGORITHM_VERSION = "shared-python-1"
MISSIONS = frozenset({"STRAIGHT_TRACK", "AUTOCROSS", "SKIDPAD", "BRAKING_TEST"})


def fit_skidpad_topology(observed_cones: Iterable[dict]) -> dict:
    """Fit two spatially separated rings from odom-frame cone observations.

    This is a diagnostic hypothesis, never a surveyed map or vehicle authority.
    Full angular coverage and low residual are required to reject partial arcs.
    """
    observations = [item for item in observed_cones
                    if item.get("label") in {"blue", "yellow"} and
                    item.get("sigma_m", float("inf")) <= 0.5]
    points = np.asarray([(item["x_m"], item["y_m"]) for item in observations],
                        dtype=float).reshape(-1, 2)
    if len(points) < 32 or not np.isfinite(points).all():
        return {"valid": False, "reason": "INSUFFICIENT_OBSERVED_CONES"}
    centered = points - np.mean(points, axis=0)
    _, eigenvectors = np.linalg.eigh(np.cov(centered, rowvar=False))
    axis = eigenvectors[:, -1]
    dominant = int(np.argmax(np.abs(axis)))
    if axis[dominant] < 0:
        axis = -axis
    projection = points @ axis
    split = float(np.median(projection))
    # At the crossover the outer boundary of one loop can lie on the other
    # side of the median plane. Exclude that narrow ambiguous band from the
    # circle fit; the remaining observed arc must still satisfy the existing
    # angular-coverage and residual checks. No reference-course coordinates
    # are inserted into the observed map.
    crossover_margin_m = 1.6
    fitted = []
    for side in (-1, 1):
        boundaries = []
        for label in ("blue", "yellow"):
            group = np.asarray([(item["x_m"], item["y_m"]) for item in observations
                                if item["label"] == label and
                                abs(item["x_m"] * axis[0] + item["y_m"] * axis[1] - split) >= crossover_margin_m and
                                (((item["x_m"] * axis[0] + item["y_m"] * axis[1]) < split)
                                 if side < 0 else
                                 ((item["x_m"] * axis[0] + item["y_m"] * axis[1]) >= split))],
                               dtype=float).reshape(-1, 2)
            if len(group) < 8:
                return {"valid": False, "reason": "INSUFFICIENT_LOOP_COVERAGE"}
            ring = _fit_observed_ring(group)
            if ring is None:
                return {"valid": False, "reason": "LOOP_FIT_UNQUALIFIED"}
            boundaries.append(ring)
        first, second = boundaries
        center_separation = hypot(first["center_x_m"] - second["center_x_m"],
                                  first["center_y_m"] - second["center_y_m"])
        width = abs(first["radius_m"] - second["radius_m"])
        if center_separation > 0.8 or not 2.0 <= width <= 3.8:
            return {"valid": False, "reason": "LOOP_BOUNDARIES_INCONSISTENT"}
        fitted.append({"center_x_m": (first["center_x_m"] + second["center_x_m"]) / 2,
                       "center_y_m": (first["center_y_m"] + second["center_y_m"]) / 2,
                       "radius_m": (first["radius_m"] + second["radius_m"]) / 2,
                       "track_width_m": width,
                       "observed_count": first["observed_count"] + second["observed_count"],
                       "boundaries": {"blue": first, "yellow": second}})
    separation = ((fitted[1]["center_x_m"] - fitted[0]["center_x_m"]) * axis[0] +
                  (fitted[1]["center_y_m"] - fitted[0]["center_y_m"]) * axis[1])
    if separation < 2.0:
        return {"valid": False, "reason": "LOOP_IDENTITIES_AMBIGUOUS"}
    return {"valid": True, "provenance": "OBSERVED_CONES",
            "separation_axis_xy": [float(axis[0]), float(axis[1])],
            "left": fitted[0], "right": fitted[1]}


def _fit_observed_ring(group: np.ndarray) -> dict | None:
        matrix = np.column_stack((2 * group[:, 0], 2 * group[:, 1], np.ones(len(group))))
        if np.linalg.matrix_rank(matrix) < 3:
            return None
        cx, cy, offset = np.linalg.lstsq(matrix, np.sum(group ** 2, axis=1), rcond=None)[0]
        radius_sq = offset + cx * cx + cy * cy
        if radius_sq <= 0:
            return None
        radius = float(np.sqrt(radius_sq))
        distances = np.linalg.norm(group - (cx, cy), axis=1)
        residual = float(np.sqrt(np.mean((distances - radius) ** 2)))
        angles = np.sort(np.arctan2(group[:, 1] - cy, group[:, 0] - cx))
        largest_gap = float(np.max(np.diff(np.r_[angles, angles[0] + 2 * pi])))
        if not (2.0 <= radius <= 25.0 and residual <= 0.7 and largest_gap <= pi / 2 + 0.05):
            return None
        return {"center_x_m": float(cx), "center_y_m": float(cy),
                "radius_m": radius, "rms_residual_m": residual,
                "observed_count": len(group), "largest_gap_rad": largest_gap}


@dataclass(frozen=True)
class PoseEstimate:
    x_m: float
    y_m: float
    yaw_rad: float
    speed_mps: float
    timestamp_ns: int
    position_std_m: float
    frame_id: str = "odom"
    valid: bool = True

    def fresh(self, now_ns: int, maximum_age_ns: int) -> bool:
        return (self.valid and self.frame_id == "odom" and
                all(isfinite(x) for x in (self.x_m, self.y_m, self.yaw_rad,
                                           self.speed_mps, self.position_std_m)) and
                self.speed_mps >= 0 and 0 <= self.position_std_m <= 0.5 and
                isinstance(self.timestamp_ns, int) and
                0 <= now_ns - self.timestamp_ns <= maximum_age_ns)


@dataclass(frozen=True)
class BrakeCapability:
    minimum_deceleration_mps2: float
    maximum_delay_s: float
    provenance: str
    margin_m: float = 1.5

    def valid_for(self, source: str) -> bool:
        return (all(isfinite(x) for x in (self.minimum_deceleration_mps2,
                                          self.maximum_delay_s, self.margin_m)) and
                self.minimum_deceleration_mps2 > 0 and self.maximum_delay_s >= 0 and
                self.margin_m >= 0 and self.provenance in {"SIMULATED", "MEASURED"} and
                (source == "SIMULATED" or self.provenance == "MEASURED"))

    def stopping_distance(self, speed_mps: float) -> float:
        return (speed_mps * self.maximum_delay_s +
                speed_mps ** 2 / (2 * self.minimum_deceleration_mps2) + self.margin_m)


@dataclass
class TrackedCone:
    id: int
    label: str
    x_m: float
    y_m: float
    confidence: float
    sigma_m: float
    last_seen_ns: int


class LocalConeMap:
    """Bounded, pose-aware local map; no update when odometry is invalid."""

    def __init__(self, maximum_cones: int = 256, expiry_ns: int = 10_000_000_000):
        self.maximum_cones = maximum_cones
        self.expiry_ns = expiry_ns
        self._items: dict[int, TrackedCone] = {}
        self._next_id = 1

    def update(self, cones: Iterable[Cone], pose: PoseEstimate, now_ns: int) -> None:
        self._items = {key: item for key, item in self._items.items()
                       if 0 <= now_ns - item.last_seen_ns <= self.expiry_ns}
        if not pose.fresh(now_ns, 150_000_000):
            return
        c, s = cos(pose.yaw_rad), sin(pose.yaw_rad)
        for cone in cones:
            if cone.label not in {"blue", "yellow", "orange"}:
                continue
            x_m = pose.x_m + c * cone.xy[0] - s * cone.xy[1]
            y_m = pose.y_m + s * cone.xy[0] + c * cone.xy[1]
            eligible = [(hypot(item.x_m - x_m, item.y_m - y_m), key)
                        for key, item in self._items.items() if item.label == cone.label]
            distance, key = min(eligible, default=(float("inf"), -1))
            if distance <= max(0.35, 2 * cone.sigma_m) and key >= 0:
                old = self._items[key]
                weight = 0.35
                old.x_m = old.x_m * (1 - weight) + x_m * weight
                old.y_m = old.y_m * (1 - weight) + y_m * weight
                old.confidence = max(old.confidence, cone.confidence)
                old.sigma_m = min(old.sigma_m, cone.sigma_m)
                old.last_seen_ns = now_ns
            else:
                key = self._next_id
                self._next_id += 1
                self._items[key] = TrackedCone(key, cone.label, x_m, y_m,
                                               cone.confidence, cone.sigma_m, now_ns)
        if len(self._items) > self.maximum_cones:
            excess = len(self._items) - self.maximum_cones
            for key in sorted(self._items, key=lambda k: self._items[k].last_seen_ns)[:excess]:
                del self._items[key]

    def snapshot(self) -> list[dict]:
        return [asdict(self._items[key]) for key in sorted(self._items)]


class WindingTracker:
    """Count directed revolutions around an observed or surveyed circle center."""

    def __init__(self, required_left: int = 2, required_right: int = 2):
        if required_left <= 0 or required_right <= 0:
            raise ValueError("lap requirements must be positive")
        self.required_left = required_left
        self.required_right = required_right
        self.left_laps = 0
        self.right_laps = 0
        self.state = "LEFT"
        self._previous_angle: float | None = None
        self._progress_rad = 0.0

    def update(self, center: tuple[float, float], position: tuple[float, float], branch: str) -> None:
        if self.state == "COMPLETE" or branch != self.state:
            return
        radius = hypot(position[0] - center[0], position[1] - center[1])
        if not 1.0 <= radius <= 25.0:
            self._previous_angle = None
            self._progress_rad = 0.0
            return
        angle = atan2(position[1] - center[1], position[0] - center[0])
        if self._previous_angle is not None:
            delta = (angle - self._previous_angle + pi) % (2 * pi) - pi
            if abs(delta) > pi / 4:
                self._progress_rad = 0.0
            else:
                signed = delta if self.state == "LEFT" else -delta
                self._progress_rad = max(0.0, self._progress_rad + signed)
        self._previous_angle = angle
        if self._progress_rad >= 2 * pi - 0.04:
            if self.state == "LEFT":
                self.left_laps += 1
                if self.left_laps >= self.required_left:
                    self.state = "RIGHT"
            else:
                self.right_laps += 1
                if self.right_laps >= self.required_right:
                    self.state = "COMPLETE"
            self._progress_rad = 0.0
            self._previous_angle = None

    def snapshot(self) -> dict:
        return {"state": self.state, "left_laps": self.left_laps,
                "right_laps": self.right_laps, "winding_progress_turns": self._progress_rad / (2 * pi)}


class AlgorithmCore:
    def __init__(self, config: dict, *, wheelbase_m: float, maximum_pose_age_ms: float = 150):
        if not isinstance(config, dict) or not isfinite(wheelbase_m) or wheelbase_m <= 0:
            raise ValueError("invalid shared algorithm configuration")
        self.config = dict(config)
        self.wheelbase_m = wheelbase_m
        self.maximum_pose_age_ns = int(maximum_pose_age_ms * 1_000_000)
        self.map = LocalConeMap()
        self.winding = WindingTracker()
        self._last_now_ns: int | None = None
        self._controller = None
        self.configuration_sha256 = sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()

    def evaluate(self, cones: Iterable[Cone], pose: PoseEstimate | None, mission: str,
                 *, now_ns: int, source: str = "SIMULATED", obstacles: Iterable[dict] = (),
                 brake_capability: BrakeCapability | None = None,
                 stop_distance_m: float | None = None) -> dict:
        if mission not in MISSIONS:
            raise ValueError("unsupported mission")
        if not isinstance(now_ns, int) or now_ns < 0:
            raise ValueError("invalid evaluation timestamp")
        reasons: list[str] = []
        if self._last_now_ns is not None and now_ns <= self._last_now_ns:
            reasons.append("STALE_FRAME")
        self._last_now_ns = now_ns
        clean: list[Cone] = []
        seen_ids: set[int] = set()
        seen_xy: list[np.ndarray] = []
        for cone in cones:
            try:
                xy = np.asarray(cone.xy, dtype=float)
            except (TypeError, ValueError):
                reasons.append("INVALID_GEOMETRY")
                continue
            if (type(cone.id) is not int or cone.id < 0 or cone.id in seen_ids or
                    xy.shape != (2,) or not np.isfinite(xy).all() or
                    not isfinite(cone.confidence) or not 0 <= cone.confidence <= 1 or
                    not isfinite(cone.sigma_m) or cone.sigma_m < 0):
                reasons.append("INVALID_GEOMETRY")
                continue
            if any(np.linalg.norm(xy - previous) < 1e-6 for previous in seen_xy):
                reasons.append("INVALID_GEOMETRY")
                continue
            if source == "LIVE" and (
                not isinstance(cone.observed_ns, int) or cone.observed_ns <= 0 or
                not 0 <= now_ns - cone.observed_ns <= self.maximum_pose_age_ns
            ):
                reasons.append("STALE_FRAME")
                continue
            clean.append(cone)
            seen_ids.add(cone.id)
            seen_xy.append(xy)
        input_invalid = "INVALID_GEOMETRY" in reasons
        graph = triangulate(clean)
        gabriel = filter_gabriel(graph)
        by_id = {cone.id: cone for cone in clean}
        graph_cones = [by_id[key] for key in graph.ids]
        boundaries = estimate_boundaries(graph_cones, self.config)
        gates = select_track_gates(graph_cones, graph.edges, boundaries, self.config)
        selected_pairs = {(gate.left_id, gate.right_id) for gate in gates}
        rejected_gates = [
            {"left_id": gate.left_id, "right_id": gate.right_id,
             "reason": "AMBIGUOUS" if gate.status != "CANDIDATE" else "NOT_SELECTED"}
            for gate in candidate_gates(graph_cones, graph.edges, self.config)
            if (gate.left_id, gate.right_id) not in selected_pairs
        ]
        rejected_gates.extend(
            {"left_id": gate.left_id, "right_id": gate.right_id, "reason": gate.status}
            for gate in gates if gate.status != "CANDIDATE"
        )
        centerline, center_status = generate_centerline(
            gates, self.config["min_centerline_gates"], self.config["max_lateral_jump_m"],
            self.config["max_heading_change_deg"], self.config["max_width_change_m"], graph_cones)
        profile, curvature_status = profile_centerline(
            centerline, max_abs_curvature_m_inv=self.config.get("max_abs_curvature_m_inv", 2.0))
        topology = topology_reasons(gates, centerline, graph_cones,
                                    self.config["min_track_width_m"], self.config["max_track_width_m"])
        if topology:
            reasons.append("INVALID_GEOMETRY")
        if center_status != "PROVISIONAL" or len(profile) < 3:
            reasons.append("NO_VALID_PATH")
        pose_valid = pose is not None and pose.fresh(now_ns, self.maximum_pose_age_ns)
        if not pose_valid:
            reasons.append("INVALID_POSE")
        if pose_valid and "STALE_FRAME" not in reasons:
            self.map.update(clean, pose, now_ns)
        else:
            self.map.update((), PoseEstimate(0, 0, 0, 0, -1, 1, valid=False), now_ns)

        skidpad_topology = None
        if mission == "SKIDPAD":
            skidpad_topology = fit_skidpad_topology(self.map.snapshot())
            # The forward-X corridor status does not define a closed loop.
            if not input_invalid:
                reasons = [reason for reason in reasons if reason not in {"INVALID_GEOMETRY", "NO_VALID_PATH"}]
            else:
                reasons = [reason for reason in reasons if reason != "NO_VALID_PATH"]
            centerline = []
            if not skidpad_topology["valid"]:
                reasons.append("LOOP_REFERENCE_UNAVAILABLE")
            elif pose_valid and pose is not None:
                branch = self.winding.state
                if branch != "COMPLETE":
                    ring = skidpad_topology["left" if branch == "LEFT" else "right"]
                    center = (ring["center_x_m"], ring["center_y_m"])
                    radius = ring["radius_m"]
                    distance = hypot(pose.x_m - center[0], pose.y_m - center[1])
                    if abs(distance - radius) > 2.0:
                        reasons.append("SKIDPAD_BRANCH_NOT_REACHED")
                    else:
                        self.winding.update(center, (pose.x_m, pose.y_m), branch)
                        direction = 1 if branch == "LEFT" else -1
                        angle = atan2(pose.y_m - center[1], pose.x_m - center[0])
                        c, s = cos(pose.yaw_rad), sin(pose.yaw_rad)
                        for step in range(1, 13):
                            a = angle + direction * step * 0.075
                            dx = center[0] + radius * cos(a) - pose.x_m
                            dy = center[1] + radius * sin(a) - pose.y_m
                            centerline.append((c * dx + s * dy, -s * dx + c * dy))
                        if any(point[0] <= 0 for point in centerline[:3]):
                            reasons.append("SKIDPAD_HEADING_INVALID")
                else:
                    reasons.append("MISSION_COMPLETE")
            profile, curvature_status = profile_centerline(
                centerline, max_abs_curvature_m_inv=self.config.get("max_abs_curvature_m_inv", 2.0))
            center_status = "PROVISIONAL" if len(centerline) >= 3 and not reasons else "INVALID"

        stopping_distance = None
        if mission == "BRAKING_TEST":
            if brake_capability is None or not brake_capability.valid_for(source):
                reasons.append("BRAKE_CAPABILITY_UNQUALIFIED")
            elif stop_distance_m is None or not isfinite(stop_distance_m) or stop_distance_m < 0:
                reasons.append("BRAKING_ZONE_UNKNOWN")
            else:
                stopping_distance = brake_capability.stopping_distance(pose.speed_mps if pose_valid else 0.0)

        for obstacle in obstacles:
            try:
                ox, oy, radius = (float(obstacle[key]) for key in ("x_m", "y_m", "radius_m"))
            except (KeyError, TypeError, ValueError):
                reasons.append("INVALID_OBSTACLE")
                continue
            if not all(isfinite(value) for value in (ox, oy, radius)) or radius < 0:
                reasons.append("INVALID_OBSTACLE")
                continue
            if any(hypot(ox - point[0], oy - point[1]) < radius + 0.75 for point in centerline):
                reasons.append("PATH_BLOCKED")

        reasons = sorted(set(reasons))
        path_valid = not reasons
        control_valid = path_valid
        target_speed = 0.0
        steering = 0.0
        target_point_base_m = None
        if path_valid and pose is not None:
            try:
                from kart_sim.autonomy import (MessageHeader, PurePursuitController,
                                               Trajectory, TrajectoryPoint)
                from kart_sim.types import PlanarKartState
                if self._controller is None:
                    self._controller = PurePursuitController(wheelbase_m=self.wheelbase_m)
                c, s = cos(pose.yaw_rad), sin(pose.yaw_rad)
                now_s = now_ns / 1e9
                points = tuple(TrajectoryPoint(
                    now_s + 0.2 * (index + 1),
                    pose.x_m + c * xy[0] - s * xy[1],
                    pose.y_m + s * xy[0] + c * xy[1],
                    heading_rad=pose.yaw_rad + sample.heading_rad,
                    speed_mps=1.0,
                    curvature_1pm=sample.curvature_m_inv or 0.0,
                ) for index, (xy, sample) in enumerate(zip(centerline, profile)))
                trajectory = Trajectory(MessageHeader(now_s, "odom", 0, "shared_core"),
                                        points, now_s + 1.0)
                trajectory.validate(current_time_s=now_s, expected_frame="odom")
                output = self._controller.compute(
                    PlanarKartState(x_m=pose.x_m, y_m=pose.y_m, yaw_rad=pose.yaw_rad,
                                    u_mps=pose.speed_mps), trajectory, 0.1)
                target_speed, steering = output.target_speed_mps, output.steering_rad
                if output.target_xy_m is not None:
                    target_dx = output.target_xy_m[0] - pose.x_m
                    target_dy = output.target_xy_m[1] - pose.y_m
                    target_point_base_m = (c * target_dx + s * target_dy,
                                           -s * target_dx + c * target_dy)
                if mission == "BRAKING_TEST" and stopping_distance is not None and stop_distance_m <= stopping_distance:
                    target_speed = 0.0
            except ImportError:
                reasons.append("CONTROL_DEPENDENCY_UNAVAILABLE")
                control_valid = False
            except (ValueError, RuntimeError) as exc:
                reasons.append(f"CONTROLLER_UNAVAILABLE:{type(exc).__name__}")
                path_valid = False
                control_valid = False

        return {
            "algorithm_version": ALGORITHM_VERSION,
            "configuration_sha256": self.configuration_sha256,
            "timestamp_ns": now_ns,
            "source": source,
            "frames": {"cones": "base_link", "pose": "odom", "map": "odom"},
            "mission": mission,
            "mission_state": self.winding.snapshot() if mission == "SKIDPAD" else {"state": "DIAGNOSTIC"},
            "reasons": sorted(set(reasons)),
            "observations": [asdict(item) for item in clean],
            "tracked_cones": self.map.snapshot(),
            "geometry": {
                "status": graph.status,
                "delaunay_edges": list(graph.edges),
                "delaunay_triangles": list(graph.triangles),
                "gabriel_edges": list(gabriel),
                "boundaries": asdict(boundaries),
                "gates": [asdict(gate) for gate in gates],
                "rejected_candidates": rejected_gates,
                "centerline": centerline,
                "centerline_status": center_status,
                "curvature_status": curvature_status,
                "path_profile": [asdict(sample) for sample in profile],
                "topology_reasons": topology,
                "skidpad_topology": skidpad_topology,
            },
            "planning": {"path_valid": path_valid, "path_frame": "base_link",
                         "local_path": centerline if path_valid else [],
                         "stopping_distance_m": stopping_distance,
                         "speed_profile_provenance": "SIMULATED" if source == "SIMULATED" else "UNQUALIFIED"},
            "control_candidate": {"valid": control_valid, "target_speed_mps": target_speed if control_valid else 0.0,
                                  "steering_angle_rad": steering if control_valid else 0.0,
                                  "target_point_base_m": target_point_base_m if control_valid else None,
                                  "expiry_ns": now_ns + 100_000_000 if control_valid else now_ns,
                                  "published_to_vcu": False},
            "vehicle_authorized": False,
        }
