"""AutoKart metric cone geometry and optional RGB-D perception.

Base imports require only NumPy and SciPy. Camera, detector, ROS, LAB, and
controller integrations are optional and loaded only by their own modules.
This package has no actuator authority.
"""

from .boundaries import estimate_boundaries
from .centerline import generate_centerline
from .curvature import profile_centerline
from .gabriel import filter_gabriel
from .gates import candidate_gates, select_track_gates
from .geometry import triangulate
from .safety import topology_reasons
from .sdk import (ConfigurationError, ControlCandidate, DependencyError,
                  GeometrySnapshot, InputValidationError, PerceptionGeometrySDK,
                  PlanningSnapshot, SDKError, SDKResult, default_config,
                  process_cones, process_frame, validate_config)
from .shared_core import AlgorithmCore, LocalConeMap, PoseEstimate, WindingTracker, fit_skidpad_topology
from .types import Boundaries, Cone, Gate, Graph, PathSample

__version__ = "0.2.0"
__all__ = [
    "AlgorithmCore", "Boundaries", "Cone", "ConfigurationError", "ControlCandidate",
    "DependencyError", "Gate", "GeometrySnapshot", "Graph", "InputValidationError",
    "LocalConeMap", "PathSample", "PerceptionGeometrySDK", "PlanningSnapshot",
    "PoseEstimate", "SDKError", "SDKResult", "WindingTracker", "candidate_gates",
    "default_config", "process_cones", "process_frame", "validate_config",
    "estimate_boundaries", "filter_gabriel", "fit_skidpad_topology",
    "generate_centerline", "profile_centerline", "select_track_gates",
    "topology_reasons", "triangulate",
]
