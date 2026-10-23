"""Pose-aware observed cone map engine: thin, ROS-free, testable glue over the real
``autokart_perception`` geometry algorithms.

Ported in behavior (not imported) from the validation/sequencing logic in the existing
``kart/ros2_ws/src/autokart_nodes/autokart_mapping/autokart_mapping/node.py``'s ``MappingEngine``
and ``observed_exit_centerline``. That module is the live production ROS package for a different
workspace (``kart/ros2_ws``), not a reusable pure-Python library -- it has no ``setup.py``/importable
distribution of its own outside that workspace's build, and it is explicitly out of scope to modify
or depend on here (see the mission statement: do not touch other existing node packages). The
*algorithm* it wraps (Delaunay/Gabriel/gates/centerline, the observed two-ring skidpad fit) is reused
unchanged via :mod:`autokart_mapping.algorithm`; this module re-implements only the thin ROS-adapter
validation/bookkeeping glue (frame/timestamp/odometry qualification, duplicate-id rejection) that the
existing node also defines locally rather than importing from a shared library -- the same role
``autokart_hp60c/node.py``'s own fault-handling loop plays relative to the reused HP60C SDK.

Any discrepancy in the reused geometry/skidpad algorithm itself (not this glue) is not re-derived or
silently "fixed" here -- see reports/SENIOR_REVIEWER_REPORT.md.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, pi, sin, sqrt
from typing import Iterable, Optional


LABELS = {1: "blue", 2: "yellow", 3: "orange", 4: "large_orange"}
CLASSES = {value: key for key, value in LABELS.items()}


@dataclass(frozen=True)
class ConeObservation:
    """ROS-free input: one cone observation in ``base_link`` (metric, X-forward/Y-left)."""

    id: int
    semantic_class: int
    x_m: float
    y_m: float
    confidence: float
    sigma_m: float


def observed_exit_centerline(tracked: list[dict], topology: dict) -> list[tuple[float, float]]:
    """Pair observed exit cones in odom; leave the route absent if ambiguous.

    Behaviorally identical to the existing adapter's helper of the same name (same ordered
    blue/yellow pairing, same forward/lateral gates, same minimum corridor length and spacing
    checks) -- this is thin geometry glue consuming the already-fitted ``skidpad_topology`` dict
    from the reused ``fit_skidpad_topology``, not an independent algorithm.
    """
    if not topology.get("valid"):
        return []
    left, right = topology["left"], topology["right"]
    ex, ey = topology["separation_axis_xy"]
    tx, ty = -ey, ex
    origin = ((left["center_x_m"] + right["center_x_m"]) / 2,
              (left["center_y_m"] + right["center_y_m"]) / 2)
    sides = {"blue": [], "yellow": []}
    for cone in tracked:
        label = cone.get("label")
        if label not in sides:
            continue
        x, y = float(cone["x_m"]), float(cone["y_m"])
        lateral = (x - origin[0]) * ex + (y - origin[1]) * ey
        forward = (x - origin[0]) * tx + (y - origin[1]) * ty
        if forward >= 4.5 and (lateral < 0 if label == "blue" else lateral > 0) and abs(lateral) <= 3.0:
            sides[label].append((forward, lateral, x, y))
    pairs = []
    used_yellow = set()
    for blue in sorted(sides["blue"]):
        matches = [(abs(blue[0] - yellow[0]) + abs(yellow[1] - blue[1] - 3.0), index, yellow)
                   for index, yellow in enumerate(sides["yellow"])
                   if index not in used_yellow and abs(blue[0] - yellow[0]) <= 0.8
                   and 2.5 <= yellow[1] - blue[1] <= 4.5]
        matches.sort()
        if not matches or (len(matches) > 1 and matches[1][0] - matches[0][0] < 0.25):
            continue
        _, index, yellow = matches[0]
        used_yellow.add(index)
        pairs.append(((blue[0] + yellow[0]) / 2,
                      ((blue[2] + yellow[2]) / 2, (blue[3] + yellow[3]) / 2)))
    pairs.sort()
    if len(pairs) < 3 or pairs[-1][0] - pairs[0][0] < 5.0:
        return []
    if any(not 0.8 <= b[0] - a[0] <= 4.0 for a, b in zip(pairs, pairs[1:])):
        return []
    return [point for _, point in pairs]


class MappingEngine:
    """Drives the reused ``AlgorithmCore.evaluate(...)`` (always with mission="STRAIGHT_TRACK", so
    the full Delaunay/Gabriel/gates/centerline diagnostics are always computed), then additionally
    fits the observed skidpad ring topology and exit corridor on top of the resulting tracked-cone
    map -- matching the division of responsibility in the existing, reused codebase: mapping (this
    node) produces geometry and an observed skidpad *hypothesis*; the winding-number/gate-crossing
    *progress* state machine that consumes it lives in mission (node 07, ``autokart_mission``), not
    here.
    """

    def __init__(self, algorithm):
        self.algorithm = algorithm
        self.core = algorithm.new_core_runtime().algorithms
        self.last_stamp = -1
        self.last_pose_stamp = -1
        self.last_pose = None

    def clear(self):
        self.core = self.algorithm.new_core_runtime().algorithms
        self.last_stamp = -1
        self.last_pose_stamp = -1
        self.last_pose = None

    def update(self, cones: Iterable[ConeObservation], status: dict, pose) -> dict:
        stamp = int(status.get("sample_time_ns", -1))
        reasons = []
        if status.get("frame_id", "base_link") != "base_link" or status.get("ground_truth", False):
            reasons.append("UNQUALIFIED_CONE_FRAME")
        if status.get("sample_time_ns") != stamp or not status.get("valid"):
            reasons.append("PERCEPTION_STATUS_MISMATCH")
        if status.get("source") not in {"LIVE", "REPLAY", "SIMULATED", "SYNTHETIC"}:
            reasons.append("SOURCE_UNQUALIFIED")
        if stamp <= self.last_stamp or stamp <= 0:
            reasons.append("NONMONOTONE_CONES")
        if pose.timestamp_ns <= self.last_pose_stamp:
            reasons.append("NONMONOTONE_ODOM")
        if self.last_pose is not None and pose.timestamp_ns > self.last_pose_stamp:
            delta_s = (pose.timestamp_ns - self.last_pose_stamp) / 1e9
            displacement = hypot(pose.x_m - self.last_pose.x_m, pose.y_m - self.last_pose.y_m)
            yaw_delta = abs(atan2(sin(pose.yaw_rad - self.last_pose.yaw_rad),
                                  cos(pose.yaw_rad - self.last_pose.yaw_rad)))
            if displacement > max(1.0, max(pose.speed_mps, self.last_pose.speed_mps) * delta_s + 0.5) or yaw_delta > pi / 2:
                reasons.append("ODOM_DISCONTINUITY")
        if not pose.fresh(stamp, 150_000_000):
            reasons.append("INVALID_OR_STALE_ODOM")
        cones = list(cones)
        if not cones:
            reasons.append("NO_OBSERVED_CONES")
        if reasons:
            self.clear()
            return {"valid": False, "reason_codes": reasons, "tracked_cones": [],
                    "geometry": {}, "source": status.get("source", "UNKNOWN"), "sample_time_ns": stamp}
        observations = []
        seen = set()
        for item in cones:
            label = LABELS.get(item.semantic_class)
            xy = (float(item.x_m), float(item.y_m))
            sigma = float(item.sigma_m)
            if (label is None or item.id in seen or
                    not all(isfinite(value) for value in (*xy, sigma, item.confidence)) or
                    sigma < 0 or sigma > 0.5 or not 0 <= item.confidence <= 1):
                self.clear()
                return {"valid": False, "reason_codes": ["INVALID_CONE"], "tracked_cones": [],
                        "geometry": {}, "source": status["source"], "sample_time_ns": stamp}
            seen.add(item.id)
            observations.append(self.algorithm.Cone(int(item.id), label, float(item.confidence), xy,
                                                     sigma, sqrt(xy[0] ** 2 + xy[1] ** 2),
                                                     (0, 0, 0, 0), stamp))
        raw = self.core.evaluate(observations, pose, "STRAIGHT_TRACK", now_ns=stamp,
                                 source=status["source"])
        self.last_stamp, self.last_pose_stamp = stamp, pose.timestamp_ns
        self.last_pose = pose
        tracked = raw["tracked_cones"]
        geometry = raw["geometry"]

        def odom_xy(x, y):
            return (pose.x_m + cos(pose.yaw_rad) * x - sin(pose.yaw_rad) * y,
                    pose.y_m + sin(pose.yaw_rad) * x + cos(pose.yaw_rad) * y)

        geometry["observation_cones_odom"] = [
            {"id": cone.id, "label": cone.label, "confidence": cone.confidence,
             "x_m": odom_xy(*cone.xy)[0], "y_m": odom_xy(*cone.xy)[1]}
            for cone in observations]
        geometry["centerline_odom"] = [odom_xy(*point) for point in geometry.get("centerline", [])]
        geometry["skidpad_topology"] = self.algorithm.fit_skidpad_topology(tracked)
        geometry["exit_centerline_odom"] = observed_exit_centerline(tracked, geometry["skidpad_topology"])
        return {"valid": True, "reason_codes": [], "tracked_cones": tracked,
                "geometry": geometry, "source": status["source"],
                "sample_time_ns": stamp, "pose_time_ns": pose.timestamp_ns,
                "algorithm_version": raw["algorithm_version"],
                "configuration_sha256": raw["configuration_sha256"]}
