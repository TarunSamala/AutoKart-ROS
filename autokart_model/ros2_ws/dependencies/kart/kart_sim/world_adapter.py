"""Optional CARLA/Unreal visual-world contract with simulator physics authority."""

from __future__ import annotations

from dataclasses import dataclass, field
from importlib.util import find_spec
from math import isfinite
from typing import Callable

from .types import PlanarKartState, SimulationError


@dataclass(frozen=True)
class WorldAvailability:
    carla_available: bool
    unreal_available: bool

    @classmethod
    def detect(cls) -> "WorldAvailability":
        return cls(find_spec("carla") is not None, find_spec("unreal") is not None)


@dataclass(frozen=True)
class WorldPose:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float

    def validate(self) -> None:
        if not all(isfinite(value) for value in (self.x_m, self.y_m, self.z_m, self.yaw_rad)):
            raise SimulationError("world pose contains NaN or infinity")


def simulator_to_unreal(pose: WorldPose) -> WorldPose:
    """Map simulator +y-left to Unreal/CARLA +y-right; metres and yaw sign flip."""
    pose.validate()
    return WorldPose(pose.x_m, -pose.y_m, pose.z_m, -pose.yaw_rad)


def unreal_to_simulator(pose: WorldPose) -> WorldPose:
    return simulator_to_unreal(pose)  # This selected mapping is involutory.


@dataclass(frozen=True)
class DynamicActor:
    actor_id: str
    semantic_class: str
    pose: WorldPose
    velocity_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    collision_radius_m: float = 0.25


@dataclass(frozen=True)
class WorldTick:
    simulation_time_s: float
    fixed_delta_s: float
    actor_poses: dict[str, WorldPose]


@dataclass
class VisualFollowerSession:
    """Synchronous visual follower; it never integrates or commands the kart."""

    fixed_delta_s: float
    actors: dict[str, DynamicActor] = field(default_factory=dict)
    ticks: list[WorldTick] = field(default_factory=list)
    paused: bool = False

    def __post_init__(self) -> None:
        if self.fixed_delta_s <= 0.0:
            raise SimulationError("visual session fixed delta must be positive")

    def add_actor(self, actor: DynamicActor) -> None:
        if actor.actor_id in self.actors:
            raise SimulationError("world actor IDs must be unique")
        actor.pose.validate(); self.actors[actor.actor_id] = actor

    def tick_from_authoritative_state(self, state: PlanarKartState) -> WorldTick:
        if self.paused:
            return self.ticks[-1] if self.ticks else WorldTick(state.time_s, self.fixed_delta_s, {})
        pose = simulator_to_unreal(WorldPose(state.x_m, state.y_m, 0.0, state.yaw_rad))
        actor_poses = {actor_id: actor.pose for actor_id, actor in self.actors.items()}
        actor_poses["kart_visual"] = pose
        tick = WorldTick(state.time_s, self.fixed_delta_s, actor_poses)
        self.ticks.append(tick)
        return tick

    def pause(self) -> None: self.paused = True
    def resume(self) -> None: self.paused = False
    def reset(self) -> None: self.ticks.clear(); self.paused = True


@dataclass(frozen=True)
class TrackVisualPoint:
    s_m: float
    center: WorldPose
    left_width_m: float
    right_width_m: float


def export_track_payload(points: tuple[TrackVisualPoint, ...]) -> dict[str, object]:
    """Build a deterministic engine-neutral track payload in Unreal coordinates."""
    if len(points) < 2 or any(point.left_width_m <= 0 or point.right_width_m <= 0 for point in points):
        raise SimulationError("track visual export requires two points and positive widths")
    if any(points[index].s_m <= points[index - 1].s_m for index in range(1, len(points))):
        raise SimulationError("track visual arc length must be strictly increasing")
    return {
        "coordinate_contract": "sim_x_forward_y_left_z_up_to_unreal_y_right",
        "points": [
            {
                "s_m": point.s_m,
                "pose": simulator_to_unreal(point.center).__dict__,
                "left_width_m": point.left_width_m,
                "right_width_m": point.right_width_m,
            }
            for point in points
        ],
    }


@dataclass(frozen=True)
class RenderSensorSample:
    sensor_id: str
    measurement_time_s: float
    delivery_time_s: float
    frame_id: str
    payload: object


@dataclass
class HeadlessSensorCallbackBridge:
    """Deterministic callback contract used when CARLA/Unreal is unavailable."""

    callbacks: dict[str, Callable[[RenderSensorSample], None]] = field(default_factory=dict)
    last_measurement_time_s: dict[str, float] = field(default_factory=dict)

    def register(self, sensor_id: str, callback: Callable[[RenderSensorSample], None]) -> None:
        if not sensor_id or sensor_id in self.callbacks:
            raise SimulationError("render sensor callback ID must be non-empty and unique")
        self.callbacks[sensor_id] = callback

    def deliver(self, sample: RenderSensorSample) -> None:
        if sample.sensor_id not in self.callbacks:
            raise SimulationError("render sensor has no registered callback")
        if sample.delivery_time_s < sample.measurement_time_s:
            raise SimulationError("render sensor delivery precedes measurement")
        previous = self.last_measurement_time_s.get(sample.sensor_id)
        if previous is not None and sample.measurement_time_s < previous:
            raise SimulationError("render sensor measurement time moved backward")
        self.last_measurement_time_s[sample.sensor_id] = sample.measurement_time_s
        self.callbacks[sample.sensor_id](sample)
