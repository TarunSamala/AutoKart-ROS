"""Cone tracking and straight-corridor construction (no ROS).

Frames: perception gives cones in the vehicle frame (x forward, y left). Tracks live in the odom frame.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

Pose = Tuple[float, float, float]  # x, y, yaw


def vehicle_to_odom(x: float, y: float, pose: Pose) -> Tuple[float, float]:
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return px + c * x - s * y, py + s * x + c * y


def odom_to_vehicle(x: float, y: float, pose: Pose) -> Tuple[float, float]:
    px, py, yaw = pose
    dx, dy = x - px, y - py
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


@dataclass
class Track:
    cls: str
    x: float
    y: float
    hits: int = 1
    last_seen: float = 0.0


@dataclass
class MapConfig:
    left_class: str = "blue"
    right_class: str = "yellow"
    assoc_radius_m: float = 0.5
    ema_alpha: float = 0.4
    min_hits: int = 3
    max_age_s: float = 20.0   # cones are static; they must outlive the camera's minimum range
    pair_tol_m: float = 1.0          # max difference in forward position for a left/right pair
    min_width_m: float = 1.0
    max_width_m: float = 5.0
    nominal_width_m: float = 3.0     # used only when a single side is visible
    min_forward_m: float = -0.5      # ignore cones this far behind the kart
    centerline_step_m: float = 0.25
    min_centerline_len_m: float = 3.0
    max_cone_range_m: float = 10.0


class ConeTracker:
    def __init__(self, cfg: MapConfig):
        self.cfg = cfg
        self.tracks: List[Track] = []

    def update(self, cones_vehicle: List[Tuple[str, float, float]], pose: Pose, now: float) -> None:
        c = self.cfg
        used = set()
        for cls, x, y in cones_vehicle:
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            ox, oy = vehicle_to_odom(x, y, pose)
            best, bd = None, c.assoc_radius_m
            for i, t in enumerate(self.tracks):
                if i in used or t.cls != cls:
                    continue
                d = math.hypot(t.x - ox, t.y - oy)
                if d < bd:
                    best, bd = i, d
            if best is None:
                self.tracks.append(Track(cls, ox, oy, 1, now))
                used.add(len(self.tracks) - 1)
            else:
                t = self.tracks[best]
                a = c.ema_alpha
                t.x += a * (ox - t.x)
                t.y += a * (oy - t.y)
                t.hits += 1
                t.last_seen = now
                used.add(best)
        self.tracks = [t for t in self.tracks if now - t.last_seen <= c.max_age_s]

    def confirmed(self) -> List[Track]:
        return [t for t in self.tracks if t.hits >= self.cfg.min_hits]


@dataclass
class Corridor:
    valid: bool
    status: str
    width_m: float = 0.0
    centerline: List[Tuple[float, float]] = field(default_factory=list)


def _fit_direction(points: List[Tuple[float, float]]) -> Tuple[float, float, float, float]:
    """Principal direction of 2D points. Returns (cx, cy, dx, dy)."""
    n = len(points)
    cx = sum(p[0] for p in points) / n
    cy = sum(p[1] for p in points) / n
    sxx = sum((p[0] - cx) ** 2 for p in points)
    syy = sum((p[1] - cy) ** 2 for p in points)
    sxy = sum((p[0] - cx) * (p[1] - cy) for p in points)
    ang = 0.5 * math.atan2(2 * sxy, sxx - syy)
    return cx, cy, math.cos(ang), math.sin(ang)


def build_corridor(tracks: List[Track], pose: Pose, cfg: MapConfig) -> Corridor:
    heading = (math.cos(pose[2]), math.sin(pose[2]))
    ahead = []
    for t in tracks:
        vx, vy = odom_to_vehicle(t.x, t.y, pose)
        if cfg.min_forward_m <= vx <= cfg.max_cone_range_m:
            ahead.append((t, vx, vy))
    left = sorted([a for a in ahead if a[0].cls == cfg.left_class], key=lambda a: a[1])
    right = sorted([a for a in ahead if a[0].cls == cfg.right_class], key=lambda a: a[1])

    pairs: List[Tuple[Tuple[float, float], float]] = []   # (midpoint odom, width)
    free_r = list(right)
    for l in left:
        best, bd = None, cfg.pair_tol_m
        for r in free_r:
            d = abs(l[1] - r[1])
            if d < bd:
                best, bd = r, d
        if best is not None:
            free_r.remove(best)
            mid = ((l[0].x + best[0].x) / 2, (l[0].y + best[0].y) / 2)
            pairs.append((mid, math.hypot(l[0].x - best[0].x, l[0].y - best[0].y)))

    status = ""
    width = 0.0
    if len(pairs) >= 2:
        pts = [p[0] for p in pairs]
        cx, cy, dx, dy = _fit_direction(pts)
        width = sum(p[1] for p in pairs) / len(pairs)
        status = f"{len(pairs)}_PAIRS"
    elif len(pairs) == 1:
        (cx, cy), width = pairs[0]
        dx, dy = heading
        status = "ONE_PAIR_HEADING_ASSUMED"
    else:
        side = left if len(left) >= 2 else right if len(right) >= 2 else []
        if not side:
            return Corridor(False, "NO_CORRIDOR")
        pts = [(a[0].x, a[0].y) for a in side]
        sx, sy, dx, dy = _fit_direction(pts)
        # offset toward the corridor interior by half the nominal width (left cones: interior is to the right)
        nx, ny = -dy, dx  # left normal
        off = -cfg.nominal_width_m / 2 if side is left else cfg.nominal_width_m / 2
        cx, cy = sx + nx * off, sy + ny * off
        width = cfg.nominal_width_m
        status = "ONE_SIDE_NOMINAL_WIDTH"

    if dx * heading[0] + dy * heading[1] < 0:
        dx, dy = -dx, -dy
    if not (cfg.min_width_m <= width <= cfg.max_width_m):
        return Corridor(False, f"WIDTH_OUT_OF_RANGE_{width:.2f}", width)

    # start at the projection of the kart on the line, run to the farthest cone (any class) ahead
    kx, ky = pose[0], pose[1]
    s0 = (kx - cx) * dx + (ky - cy) * dy
    s_end = s0 + cfg.min_centerline_len_m
    for t, vx, vy in ahead:
        s_end = max(s_end, (t.x - cx) * dx + (t.y - cy) * dy + 0.5)
    pts_out = []
    s = s0
    while s <= s_end:
        pts_out.append((cx + dx * s, cy + dy * s))
        s += cfg.centerline_step_m
    return Corridor(True, status, width, pts_out)
