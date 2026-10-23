"""Deterministic, transport-neutral ROS 2/SIL adapter contracts.

The real ROS 2 adapter is optional.  The mock bridge below is the CI backend
and records the same message-time semantics without importing ROS middleware.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from importlib.util import find_spec
from math import isfinite, sqrt
from pathlib import Path
from typing import Any

from pydantic import BaseModel, Field

import yaml

from .autonomy import MessageHeader, SensorEnvelope
from .types import SimulationError


@dataclass(frozen=True)
class Ros2Runtime:
    cli_available: bool
    rclpy_available: bool

    @classmethod
    def detect(cls) -> "Ros2Runtime":
        from pathlib import Path

        return cls(Path("/opt/ros/humble/bin/ros2").exists() or Path("/usr/bin/ros2").exists(), find_spec("rclpy") is not None)


@dataclass(frozen=True)
class ClockMessage:
    epoch: int
    time_s: float


@dataclass
class SimulationClock:
    """Authoritative simulated /clock source with explicit pause/reset epochs."""

    dt_s: float
    time_s: float = 0.0
    epoch: int = 0
    paused: bool = False

    def __post_init__(self) -> None:
        if not isfinite(self.dt_s) or self.dt_s <= 0.0:
            raise SimulationError("simulation clock requires positive finite dt")

    def message(self) -> ClockMessage:
        return ClockMessage(self.epoch, self.time_s)

    def pause(self) -> None:
        self.paused = True

    def resume(self) -> None:
        self.paused = False

    def step(self, count: int = 1) -> ClockMessage:
        if count < 0:
            raise SimulationError("clock cannot step backwards")
        if not self.paused:
            self.time_s += count * self.dt_s
        return self.message()

    def reset(self) -> ClockMessage:
        self.epoch += 1
        self.time_s = 0.0
        self.paused = True
        return self.message()


@dataclass(frozen=True)
class Transform:
    parent: str
    child: str
    translation_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    static: bool = True
    timestamp_s: float = 0.0

    def validated(self) -> "Transform":
        if not self.parent or not self.child or self.parent == self.child:
            raise SimulationError("transform must have distinct non-empty frame identifiers")
        if not all(isfinite(value) for value in (*self.translation_m, *self.rotation_xyzw, self.timestamp_s)):
            raise SimulationError("transform contains NaN or infinity")
        norm = sqrt(sum(component * component for component in self.rotation_xyzw))
        if norm < 1e-12:
            raise SimulationError("transform quaternion has zero norm")
        return Transform(self.parent, self.child, self.translation_m,
                         tuple(component / norm for component in self.rotation_xyzw), self.static, self.timestamp_s)


@dataclass
class FrameTree:
    transforms: dict[str, Transform] = field(default_factory=dict)

    def add(self, transform: Transform) -> None:
        validated = transform.validated()
        if validated.child in self.transforms:
            raise SimulationError(f"duplicate child frame: {validated.child}")
        self.transforms[validated.child] = validated
        self.validate()

    def validate(self) -> None:
        for child in self.transforms:
            seen: set[str] = set()
            node = child
            while node in self.transforms:
                if node in seen:
                    raise SimulationError("frame tree contains a cycle")
                seen.add(node)
                node = self.transforms[node].parent


@dataclass(frozen=True)
class QosProfile:
    reliability: str
    durability: str
    depth: int
    deadline_s: float | None = None

    def validate(self) -> None:
        if self.reliability not in {"reliable", "best_effort"} or self.durability not in {"volatile", "transient_local"}:
            raise SimulationError("unsupported QoS policy")
        if self.depth <= 0 or (self.deadline_s is not None and self.deadline_s <= 0.0):
            raise SimulationError("QoS depth and deadline must be positive")


TOPIC_QOS: dict[str, QosProfile] = {
    "/clock": QosProfile("reliable", "volatile", 1),
    "/tf": QosProfile("best_effort", "volatile", 100),
    "/tf_static": QosProfile("reliable", "transient_local", 100),
    "/joint_states": QosProfile("best_effort", "volatile", 10),
    "/vehicle/odometry": QosProfile("best_effort", "volatile", 10),
    "/vehicle/state": QosProfile("best_effort", "volatile", 10),
    "/vehicle/wheel_states": QosProfile("best_effort", "volatile", 10),
    "/vehicle/actuator_feedback": QosProfile("best_effort", "volatile", 10),
    "/sensors/imu": QosProfile("best_effort", "volatile", 10),
    "/sensors/camera/image": QosProfile("best_effort", "volatile", 2),
    "/sensors/depth/image": QosProfile("best_effort", "volatile", 2),
    "/sensors/depth/points": QosProfile("best_effort", "volatile", 2),
    "/sensors/lidar/scan": QosProfile("best_effort", "volatile", 2),
    "/perception/cones": QosProfile("best_effort", "volatile", 10),
    "/perception/obstacles": QosProfile("best_effort", "volatile", 10),
    "/localization/odometry": QosProfile("best_effort", "volatile", 10),
    "/planning/path": QosProfile("reliable", "volatile", 3),
    "/planning/trajectory": QosProfile("reliable", "volatile", 3),
    "/control/motion_command": QosProfile("reliable", "volatile", 1, 0.1),
    "/vcu/validated_command": QosProfile("reliable", "volatile", 1, 0.02),
    "/vcu/safety_state": QosProfile("reliable", "transient_local", 1),
    "/vcu/faults": QosProfile("reliable", "transient_local", 10),
    "/safety/envelope": QosProfile("reliable", "volatile", 3),
    "/safety/estop": QosProfile("reliable", "transient_local", 1),
    "/diagnostics": QosProfile("reliable", "volatile", 10),
    "/vcu/state": QosProfile("reliable", "transient_local", 1),
}


@dataclass(frozen=True)
class PublishedMessage:
    topic: str
    message: Any
    publication_time_s: float


@dataclass
class DeterministicRosBridge:
    """CI/SIL message broker; ground truth is disabled unless explicitly enabled."""

    clock: SimulationClock
    production_mode: bool = True
    messages: list[PublishedMessage] = field(default_factory=list)

    def publish_clock(self) -> ClockMessage:
        message = self.clock.message()
        self.messages.append(PublishedMessage("/clock", message, self.clock.time_s))
        return message

    def publish_sensor(self, topic: str, measurement: SensorEnvelope) -> None:
        if topic.startswith("/debug/") and self.production_mode:
            raise SimulationError("debug ground truth topics are disabled in production SIL mode")
        measurement.validate()
        self.messages.append(PublishedMessage(topic, measurement, self.clock.time_s))

    def publish_debug_ground_truth(self, topic: str, header: MessageHeader, payload: Any) -> None:
        if self.production_mode:
            raise SimulationError("ground truth must not be published by production autonomy launch")
        header.validate()
        self.messages.append(PublishedMessage(f"/debug/ground_truth/{topic}", payload, self.clock.time_s))

    def diagnostics(self) -> dict[str, int]:
        by_topic: dict[str, int] = {}
        for record in self.messages:
            by_topic[record.topic] = by_topic.get(record.topic, 0) + 1
        return by_topic


def validate_qos() -> None:
    for profile in TOPIC_QOS.values():
        profile.validate()


class Ros2Configuration(BaseModel):
    use_sim_time: bool = True
    planning_frame: str = "map"
    base_frame: str = "base_link"
    production_ground_truth_topics_enabled: bool = False
    clock_epoch_on_reset: bool = True

class ControllerConfiguration(BaseModel):
    lateral: str = "pure_pursuit"
    lookahead_m: float = Field(1.5, gt=0.0)
    speed_lookahead_gain_s: float = Field(0.15, ge=0.0)
    longitudinal: str = "pid"
    command_validity_s: float = Field(0.1, gt=0.0)

class SilConfiguration(BaseModel):
    ros2: Ros2Configuration = Field(default_factory=Ros2Configuration)
    controller: ControllerConfiguration = Field(default_factory=ControllerConfiguration)
    sil: dict[str, bool] = Field(default_factory=dict)


def load_sil_configuration(path: str | Path) -> SilConfiguration:
    """Schema-validate transport configuration without importing ROS 2."""
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    try:
        configuration = SilConfiguration.model_validate(payload)  # Pydantic v2
    except AttributeError:
        configuration = SilConfiguration.parse_obj(payload)  # Pydantic v1
    if not configuration.ros2.planning_frame or not configuration.ros2.base_frame:
        raise SimulationError("ROS planning and base frame identifiers must not be empty")
    if configuration.controller.lateral not in {"pure_pursuit", "stanley", "external", "replay", "disabled"}:
        raise SimulationError("unsupported lateral controller")
    return configuration
