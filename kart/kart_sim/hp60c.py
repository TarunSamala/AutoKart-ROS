"""HP60C pinhole camera perception model and a persistent, pose-aware cone map.

Ported from docs/my_track_spec/MY_TRACK_SIM.py (CamCfg + project() + the map
association logic in simulate()), generalized with an independent vertical
FOV (the draft reused the horizontal focal length for both axes). There is no
simulated camera model anywhere else in the repo to reuse: the real HP60C
driver (autonomy_stack_phase1/.../camera.py) is a hardware SDK wrapper with no
simulation, and the ROS perception stack consumes real frames only. See
docs/my_track_spec/RECON.md section 3.

Values marked ASSUMED are not measured on the real HP60C/kart and must be
replaced once they are.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from math import atan2, cos, hypot, radians, sin, tan
from pathlib import Path
from random import Random
from typing import Any

import yaml

from .types import PlanarKartState, SimulationError

_TWO_PI = 6.283185307179586


@dataclass(frozen=True)
class HP60CConfig:
    width_px: int = 640
    height_px: int = 480
    # MEASURED: datasheet depth-aligned FOV.
    hfov_deg: float = 64.4
    vfov_deg: float = 51.7
    # MEASURED: datasheet operating depth range.
    zmin_m: float = 0.20
    zmax_m: float = 4.0
    # Mount geometry (config baseline per the draft's own comment: do not mix
    # with the 0.18 m lab value; see RECON.md section 3 for the open question
    # against the real vehicle_geometry.yaml's 0.8382 m / 17.8 deg).
    mount_height_m: float = 0.30
    mount_pitch_deg: float = 4.0
    # ASSUMED: physical cone height used only to compute pixel_height via a
    # second projection of the cone's top point. Not measured on this kart's
    # actual cones.
    cone_height_m: float = 0.325
    # MEASURED (per the task's stated figures): full-pipeline frame rate and
    # p50 latency. Note: this repo's own CHANGELOG.md reports a ~7.2 Hz
    # isolated-capture ceiling, which is a different measurement condition;
    # see RECON.md section 3.
    rate_hz: float = 4.2
    latency_s: float = 0.29
    latency_jitter_s: float = 0.03
    # ASSUMED: detector recall ramps from 0 at recall_px_min pixels of cone
    # height to recall_peak at recall_px_full pixels and above.
    recall_peak: float = 0.90
    recall_px_min: float = 10.0
    recall_px_full: float = 35.0
    # ASSUMED: probability a detected cone's reported colour is wrong.
    label_swap_p: float = 0.02
    # ASSUMED: structured-light-style noise, sigma = r0 + r1 * range^2 (depth)
    # / r0 + r1 * range (lateral).
    depth_sigma_r0_m: float = 0.01
    depth_sigma_r1_m_per_m2: float = 0.008
    lateral_sigma_l0_m: float = 0.01
    lateral_sigma_l1_m_per_m: float = 0.005
    # Sunlight/structured-light degradation switch, off by default (per the
    # task). ASSUMED magnitudes when enabled.
    sunlight_degradation: bool = False
    sunlight_zmax_scale: float = 0.6
    sunlight_extra_dropout_p: float = 0.15

    @property
    def fx_px(self) -> float:
        return (self.width_px / 2) / tan(radians(self.hfov_deg) / 2)

    @property
    def fy_px(self) -> float:
        return (self.height_px / 2) / tan(radians(self.vfov_deg) / 2)

    @property
    def effective_zmax_m(self) -> float:
        return self.zmax_m * self.sunlight_zmax_scale if self.sunlight_degradation else self.zmax_m

    def __post_init__(self) -> None:
        if self.width_px <= 0 or self.height_px <= 0:
            raise SimulationError("camera resolution must be positive")
        if not 0.0 < self.zmin_m < self.zmax_m:
            raise SimulationError("camera depth range must satisfy 0 < zmin < zmax")
        if not 0.0 <= self.recall_peak <= 1.0 or not 0.0 <= self.label_swap_p <= 1.0:
            raise SimulationError("camera recall and label-swap probabilities must be in [0, 1]")


def load_hp60c_config(path: str | Path) -> HP60CConfig:
    payload: dict[str, Any] = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    camera = payload.get("camera", payload)
    return HP60CConfig(**camera)


@dataclass(frozen=True)
class CameraDetection:
    """One cone's entry in a captured frame; always geometrically in view.

    `detected` distinguishes a successful detector roll (solid box in the
    camera panel) from a geometrically-visible-but-missed cone (dashed box).
    `reported_colour`/`noisy_x_m`/`noisy_y_m` (body frame) are only meaningful
    when `detected` is True.
    """

    cone_id: str
    u_px: float
    v_foot_px: float
    pixel_height_px: float
    true_colour: str
    depth_m: float
    detected: bool
    reported_colour: str | None = None
    noisy_x_m: float | None = None
    noisy_y_m: float | None = None


@dataclass(frozen=True)
class CameraFrame:
    capture_time_s: float
    deliver_time_s: float
    pose_at_capture: tuple[float, float, float]  # xr_m, yr_m, yaw_rad (rear axle)
    detections: tuple[CameraDetection, ...]


def _project(config: HP60CConfig, camera_x_m: float, xb_m: float, yb_m: float, z_m: float) -> tuple[float, float, float] | None:
    """Body-frame point (xb forward, yb left, z up) -> (u_px, v_px, depth_m), or
    None if behind the camera. Camera origin is on the kart centreline at
    (camera_x_m, 0, mount_height_m)."""
    pitch_rad = radians(config.mount_pitch_deg)
    xc = xb_m - camera_x_m
    dz = z_m - config.mount_height_m
    depth_m = xc * cos(pitch_rad) - dz * sin(pitch_rad)
    vertical_m = -xc * sin(pitch_rad) - dz * cos(pitch_rad)
    lateral_m = -yb_m
    if depth_m < 1e-3:
        return None
    u_px = config.width_px / 2 + config.fx_px * lateral_m / depth_m
    v_px = config.height_px / 2 + config.fy_px * vertical_m / depth_m
    return u_px, v_px, depth_m


class HP60CSensor:
    """Stateful HP60C simulation: frame scheduling, pinhole projection, noise,
    and the camera-to-world latency pipeline. One instance per scenario run."""

    def __init__(self, config: HP60CConfig, *, camera_x_m: float, seed: int = 1) -> None:
        self.config = config
        self.camera_x_m = camera_x_m
        self.random = Random(seed)
        self._next_capture_s = 1.0 / config.rate_hz

    def maybe_capture(self, t_s: float, state: PlanarKartState, rear_axle_xy: tuple[float, float], cones: list) -> CameraFrame | None:
        """Call once per physics step; returns a new frame when due."""
        if t_s < self._next_capture_s:
            return None
        self._next_capture_s += 1.0 / self.config.rate_hz
        cfg = self.config
        xr_m, yr_m = rear_axle_xy
        c, s = cos(state.yaw_rad), sin(state.yaw_rad)
        zmax_m = cfg.effective_zmax_m
        detections: list[CameraDetection] = []
        for cone in cones:
            dx, dy = cone.x_m - xr_m, cone.y_m - yr_m
            xb_m = c * dx + s * dy
            yb_m = -s * dx + c * dy
            foot = _project(cfg, self.camera_x_m, xb_m, yb_m, 0.0)
            if foot is None:
                continue
            u_px, v_foot_px, depth_m = foot
            if not (cfg.zmin_m <= depth_m <= zmax_m and 0 <= u_px < cfg.width_px and 0 <= v_foot_px < cfg.height_px):
                continue
            top = _project(cfg, self.camera_x_m, xb_m, yb_m, cfg.cone_height_m)
            pixel_height_px = (v_foot_px - top[1]) if top is not None else 0.0
            recall = cfg.recall_peak * _clip01((pixel_height_px - cfg.recall_px_min) / max(cfg.recall_px_full - cfg.recall_px_min, 1e-6))
            if cfg.sunlight_degradation:
                recall *= max(0.0, 1.0 - cfg.sunlight_extra_dropout_p)
            detected = self.random.random() < recall
            if not detected:
                detections.append(CameraDetection(cone.cone_id, u_px, v_foot_px, pixel_height_px, cone.semantic_class.value, depth_m, False))
                continue
            range_m = hypot(xb_m, yb_m)
            depth_sigma = cfg.depth_sigma_r0_m + cfg.depth_sigma_r1_m_per_m2 * range_m * range_m
            lateral_sigma = cfg.lateral_sigma_l0_m + cfg.lateral_sigma_l1_m_per_m * range_m
            along_noise = self.random.gauss(0.0, depth_sigma)
            across_noise = self.random.gauss(0.0, lateral_sigma)
            noisy_xb_m = xb_m + along_noise * xb_m / max(range_m, 1e-6) - across_noise * yb_m / max(range_m, 1e-6)
            noisy_yb_m = yb_m + along_noise * yb_m / max(range_m, 1e-6) + across_noise * xb_m / max(range_m, 1e-6)
            reported_colour = cone.semantic_class.value
            if self.random.random() < cfg.label_swap_p:
                reported_colour = _swap_colour(reported_colour)
            detections.append(CameraDetection(
                cone.cone_id, u_px, v_foot_px, pixel_height_px, cone.semantic_class.value,
                depth_m, True, reported_colour, noisy_xb_m, noisy_yb_m,
            ))
        latency_s = max(0.0, self.config.latency_s + self.random.gauss(0.0, self.config.latency_jitter_s))
        capture_time_s = t_s
        return CameraFrame(capture_time_s, capture_time_s + latency_s, (xr_m, yr_m, state.yaw_rad), tuple(detections))


def _swap_colour(colour: str) -> str:
    return {"orange": "yellow", "yellow": "orange", "blue": "unknown"}.get(colour, "unknown")


def _clip01(value: float) -> float:
    return max(0.0, min(1.0, value))


@dataclass
class TrackedCone:
    track_id: int
    x_m: float
    y_m: float
    hits: int
    votes: dict[str, int]
    last_seen_s: float

    @property
    def colour(self) -> str:
        return max(self.votes, key=self.votes.get)

    @property
    def confirmed(self) -> bool:
        return self.hits >= 2


class ConeMap:
    """Persistent, pose-aware cone map: associates within `association_radius_m`,
    confirms at hits>=2, expires after `expiry_s` unseen, colour by vote."""

    def __init__(self, *, association_radius_m: float = 0.6, expiry_s: float = 10.0) -> None:
        self.association_radius_m = association_radius_m
        self.expiry_s = expiry_s
        self.tracks: list[TrackedCone] = []
        self._next_id = 1

    def update(self, frame: CameraFrame) -> None:
        cx_m, cy_m, cyaw_rad = frame.pose_at_capture
        c, s = cos(cyaw_rad), sin(cyaw_rad)
        now_s = frame.deliver_time_s
        for detection in frame.detections:
            if not detection.detected or detection.reported_colour in (None, "unknown"):
                continue
            wx_m = cx_m + c * detection.noisy_x_m - s * detection.noisy_y_m
            wy_m = cy_m + s * detection.noisy_x_m + c * detection.noisy_y_m
            best, best_distance_m = None, self.association_radius_m
            for track in self.tracks:
                distance_m = hypot(track.x_m - wx_m, track.y_m - wy_m)
                if distance_m < best_distance_m:
                    best, best_distance_m = track, distance_m
            if best is None:
                self.tracks.append(TrackedCone(self._next_id, wx_m, wy_m, 1, {detection.reported_colour: 1}, now_s))
                self._next_id += 1
            else:
                gain = max(1.0 / (best.hits + 1), 0.2)
                best.x_m += gain * (wx_m - best.x_m)
                best.y_m += gain * (wy_m - best.y_m)
                best.hits += 1
                best.votes[detection.reported_colour] = best.votes.get(detection.reported_colour, 0) + 1
                best.last_seen_s = now_s
        self.tracks = [track for track in self.tracks if now_s - track.last_seen_s <= self.expiry_s]

    def confirmed_tracks(self) -> list[TrackedCone]:
        return [track for track in self.tracks if track.confirmed]
