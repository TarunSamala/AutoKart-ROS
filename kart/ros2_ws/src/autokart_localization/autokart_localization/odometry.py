"""Wheel-encoder odometry: speed from the rear axle encoder, pose from a bicycle model using the OBSERVED
(not actuated) steering angle. No ROS dependencies.

There is no IMU in this configuration (docs/architecture/REPOSITORY_MAP.md: "No current verified driver/
connection/calibration or live topic identified"), so yaw is a model-based estimate, never a direct
measurement -- the ``status`` field says so rather than presenting it as equally trustworthy.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

STATUS_ENCODER_STEER_MODEL = "ENCODER_STEER_MODEL"   # speed + steering both valid: full bicycle-model yaw
STATUS_ENCODER_ONLY_NO_YAW = "ENCODER_ONLY_NO_YAW"    # speed valid, steering invalid/missing: yaw frozen
STATUS_INVALID = "INVALID"                            # speed invalid: downstream must treat odometry as absent


@dataclass
class OdometryConfig:
    wheelbase_m: float = 1.0   # MEASURE
    counts_per_m: float = 0.0  # 0 selects firmware CPR x tyre radius fallback (VERIFY)
    encoder_forward_sign: int = 1  # VERIFY from forward hand-push trials


class WheelOdometry:
    def __init__(self, cfg: OdometryConfig):
        if cfg.wheelbase_m <= 0.0 or not math.isfinite(cfg.wheelbase_m):
            raise ValueError("wheelbase_m must be positive and finite")
        if cfg.counts_per_m < 0.0 or not math.isfinite(cfg.counts_per_m):
            raise ValueError("counts_per_m must be nonnegative and finite")
        if cfg.encoder_forward_sign not in (-1, 1):
            raise ValueError("encoder_forward_sign must be -1 or 1")
        self.cfg = cfg
        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0
        self.yaw_rate_radps = 0.0
        self.speed_mps = 0.0
        self.status = STATUS_INVALID

    def update_counts(self, dt_s: float, delta_counts: int, steering_rad: Optional[float]) -> bool:
        """Use calibrated axle displacement; never substitute the firmware's nominal speed here."""
        if self.cfg.counts_per_m <= 0 or not isinstance(delta_counts, int):
            self.status = STATUS_INVALID
            return False
        if dt_s <= 0 or dt_s > 0.2:
            self.status = STATUS_INVALID
            return False
        return self.update(dt_s, delta_counts * self.cfg.encoder_forward_sign /
                           self.cfg.counts_per_m / dt_s, steering_rad)

    def reset(self) -> None:
        self.x_m = self.y_m = self.yaw_rad = self.yaw_rate_radps = self.speed_mps = 0.0
        self.status = STATUS_INVALID

    def update(self, dt_s: float, speed_mps: Optional[float], steering_rad: Optional[float]) -> bool:
        """Advance the pose. Returns True if the result is valid (speed measurement present and finite).

        A missing/invalid/non-finite speed invalidates the whole estimate (status=INVALID) and freezes the
        pose rather than integrating garbage. A missing/invalid steering angle keeps the speed-only estimate
        but freezes yaw (status=ENCODER_ONLY_NO_YAW) instead of guessing a heading change.
        """
        if dt_s <= 0.0 or dt_s > 0.2 or speed_mps is None or not math.isfinite(speed_mps):
            self.status = STATUS_INVALID
            self.yaw_rate_radps = 0.0
            return False
        self.speed_mps = speed_mps
        if steering_rad is None or not math.isfinite(steering_rad):
            self.yaw_rate_radps = 0.0
            self.status = STATUS_ENCODER_ONLY_NO_YAW
        else:
            self.yaw_rate_radps = speed_mps / self.cfg.wheelbase_m * math.tan(steering_rad)
            self.status = STATUS_ENCODER_STEER_MODEL
        yaw_mid = self.yaw_rad + 0.5 * self.yaw_rate_radps * dt_s
        self.x_m += speed_mps * dt_s * math.cos(yaw_mid)
        self.y_m += speed_mps * dt_s * math.sin(yaw_mid)
        self.yaw_rad += self.yaw_rate_radps * dt_s
        return True
