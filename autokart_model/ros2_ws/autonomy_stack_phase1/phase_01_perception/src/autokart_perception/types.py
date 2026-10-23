from dataclasses import dataclass, field
from typing import Optional
import numpy as np

@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    def valid(self):
        return all(np.isfinite(x) for x in (self.fx, self.fy, self.cx, self.cy)) and self.fx > 0 and self.fy > 0

@dataclass
class Frame:
    rgb: np.ndarray
    depth: np.ndarray
    timestamp_ns: int
    rgb_timestamp: int
    depth_timestamp: int
    rgb_sequence: int
    depth_sequence: int
    intrinsics: Intrinsics
    depth_scale: float
    aligned: bool
    source: str
    drops: int = 0
    calibration: dict = field(default_factory=dict)
    received_frames: int = 0
    capture_elapsed_ns: int = 0

@dataclass(frozen=True)
class Detection:
    box: tuple[int, int, int, int]
    label: str
    confidence: float
    id: int
    observed_ns: int = 0

@dataclass(frozen=True)
class Cone:
    id: int
    label: str
    confidence: float
    xy: tuple[float, float]
    sigma_m: float
    distance_m: float
    box: tuple[int, int, int, int]
    observed_ns: int = 0

@dataclass(frozen=True)
class Graph:
    ids: tuple[int, ...]
    vertices: np.ndarray
    edges: tuple[tuple[int, int], ...]
    triangles: tuple[tuple[int, int, int], ...]
    status: str

@dataclass(frozen=True)
class Gate:
    left_id: int
    right_id: int
    midpoint: tuple[float, float]
    width: float
    quality: float
    status: str
    id: int = -1
    observed_ns: int = 0

@dataclass(frozen=True)
class Boundaries:
    left_ids: tuple[int, ...]
    right_ids: tuple[int, ...]
    mode: str
    status: str
    confidence: float
    reason: str

@dataclass(frozen=True)
class PathSample:
    xy: tuple[float, float]
    heading_rad: float
    segment_m: float
    curvature_m_inv: Optional[float]
    observed_ns: int = 0

@dataclass
class Result:
    frame: Frame
    detections: list[Detection]
    cones: list[Cone]
    delaunay: Graph
    gabriel_edges: tuple[tuple[int,int], ...]
    gates: list[Gate]
    centerline: list[tuple[float,float]]
    status: str
    diagnostics: dict
    boundaries: Optional[Boundaries] = None
    path_profile: list[PathSample] = field(default_factory=list)
    filtered_detections: list[Detection] = field(default_factory=list)
    filtered_cones: list[Cone] = field(default_factory=list)
    predicted_cones: list[dict] = field(default_factory=list)
    safety: object = None
