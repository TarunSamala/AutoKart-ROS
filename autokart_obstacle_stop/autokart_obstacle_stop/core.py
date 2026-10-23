"""ROS-independent forward-cone stop geometry and latch logic."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class ConeSample:
    cone_id: int
    x_m: float
    y_m: float
    confidence: float
    position_std_m: float = 0.0


@dataclass(frozen=True)
class GuardConfig:
    minimum_x_m: float = 0.10
    stop_distance_m: float = 3.0
    corridor_half_width_m: float = 0.75
    minimum_confidence: float = 0.50
    perception_timeout_s: float = 0.50
    stop_hold_s: float = 1.0
    clear_frames_required: int = 3
    allow_ground_truth: bool = False

    def validate(self) -> None:
        values = (
            self.minimum_x_m,
            self.stop_distance_m,
            self.corridor_half_width_m,
            self.minimum_confidence,
            self.perception_timeout_s,
            self.stop_hold_s,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("guard configuration must be finite")
        if self.minimum_x_m < 0 or self.stop_distance_m <= self.minimum_x_m:
            raise ValueError("stop_distance_m must exceed non-negative minimum_x_m")
        if self.corridor_half_width_m <= 0:
            raise ValueError("corridor_half_width_m must be positive")
        if not 0.0 <= self.minimum_confidence <= 1.0:
            raise ValueError("minimum_confidence must be within 0..1")
        if self.perception_timeout_s <= 0 or self.stop_hold_s < 0:
            raise ValueError("timeouts must be non-negative and perception timeout positive")
        if self.clear_frames_required < 1:
            raise ValueError("clear_frames_required must be at least one")


@dataclass(frozen=True)
class StopDecision:
    stop_required: bool
    reason: str
    nearest_clearance_m: float | None
    cone_id: int | None
    perception_age_s: float | None
    clear_frames: int


class ObstacleStopGuard:
    """Fails closed on stale/unqualified perception and latches real obstacles."""

    def __init__(self, config: GuardConfig):
        config.validate()
        self.config = config
        self.last_frame_s: float | None = None
        self.last_frame_qualified = False
        self.last_reject_reason = "PERCEPTION_NOT_RECEIVED"
        self.stop_latched = False
        self.hold_until_s = 0.0
        self.clear_frames = 0
        self.nearest_clearance_m: float | None = None
        self.nearest_cone_id: int | None = None

    def _candidate(self, cone: ConeSample) -> tuple[float, int] | None:
        values = (cone.x_m, cone.y_m, cone.confidence, cone.position_std_m)
        if not all(isfinite(value) for value in values):
            return None
        if cone.confidence < self.config.minimum_confidence:
            return None
        radius = max(0.0, cone.position_std_m)
        front_clearance = cone.x_m - radius
        lateral_clearance = abs(cone.y_m) - radius
        if cone.x_m < self.config.minimum_x_m:
            return None
        if front_clearance > self.config.stop_distance_m:
            return None
        if lateral_clearance > self.config.corridor_half_width_m:
            return None
        return max(0.0, front_clearance), int(cone.cone_id)

    def ingest_frame(
        self,
        now_s: float,
        *,
        frame_id: str,
        ground_truth: bool,
        cones: list[ConeSample] | tuple[ConeSample, ...],
    ) -> StopDecision:
        if not isfinite(now_s):
            raise ValueError("now_s must be finite")
        self.last_frame_s = now_s
        reject_reason = None
        if frame_id != "base_link":
            reject_reason = "PERCEPTION_FRAME_NOT_BASE_LINK"
        elif ground_truth and not self.config.allow_ground_truth:
            reject_reason = "GROUND_TRUTH_NOT_ALLOWED_FOR_ACTUATION"

        if reject_reason is not None:
            self.last_frame_qualified = False
            self.last_reject_reason = reject_reason
            self.stop_latched = True
            self.clear_frames = 0
            self.nearest_clearance_m = None
            self.nearest_cone_id = None
            return self.decision(now_s)

        self.last_frame_qualified = True
        self.last_reject_reason = "NONE"
        candidates = [result for cone in cones if (result := self._candidate(cone)) is not None]
        if candidates:
            clearance, cone_id = min(candidates, key=lambda item: item[0])
            self.stop_latched = True
            self.hold_until_s = now_s + self.config.stop_hold_s
            self.clear_frames = 0
            self.nearest_clearance_m = clearance
            self.nearest_cone_id = cone_id
        else:
            self.clear_frames += 1
            clear_hold_complete = now_s >= self.hold_until_s
            enough_clear_frames = self.clear_frames >= self.config.clear_frames_required
            if clear_hold_complete and enough_clear_frames:
                self.stop_latched = False
                self.nearest_clearance_m = None
                self.nearest_cone_id = None
        return self.decision(now_s)

    def decision(self, now_s: float) -> StopDecision:
        age = None if self.last_frame_s is None else max(0.0, now_s - self.last_frame_s)
        if self.last_frame_s is None or age is None or age > self.config.perception_timeout_s:
            return StopDecision(True, "PERCEPTION_MISSING_OR_STALE", None, None, age,
                                self.clear_frames)
        if not self.last_frame_qualified:
            return StopDecision(True, self.last_reject_reason, None, None, age,
                                self.clear_frames)
        if self.stop_latched:
            reason = "OBSTACLE_IN_FORWARD_CORRIDOR"
            if self.nearest_cone_id is None:
                reason = "OBSTACLE_STOP_HOLD"
            return StopDecision(True, reason, self.nearest_clearance_m,
                                self.nearest_cone_id, age, self.clear_frames)
        return StopDecision(False, "FORWARD_CORRIDOR_CLEAR", None, None, age,
                            self.clear_frames)
