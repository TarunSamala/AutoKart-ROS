"""Vehicle/camera/bumper geometry defaults for autokart_planner.

Single authoritative source in this repository: `kart/ros2_ws/src/autokart_bringup/config/
vehicle_geometry.yaml` (measured values; see `docs/decisions/CANONICAL_COMPONENTS.md`, which
names that file the one place these numbers should be written as literals). This package does
NOT import the `autokart_vehicle_geometry` ROS package at build/import time -- that would make
this node's default build depend on an ament package outside this bundle, the same portability
problem `autokart_hp60c` solved by splitting a dependency-free default from an optional reused
backend. Instead:

- `DEFAULT_*` below is a point-in-time COPY of that file's measured values (bumper width, lateral
  margin, camera-to-bumper offset, stop gap), read during this package's development. These are
  MEASURED values, not placeholders -- but this copy can silently drift if `vehicle_geometry.yaml`
  changes later and this file isn't updated to match.
- Setting the `vehicle_geometry_yaml_path` ROS parameter to that file's actual path makes the node
  parse it directly with plain PyYAML (no `autokart_vehicle_geometry`/`ament_index_python` import
  needed) at startup and use those values instead -- the recommended way to stay in sync with the
  authoritative file rather than trusting this copy. See DEPENDENCIES.md.
"""
from __future__ import annotations

import os
from dataclasses import dataclass

# Point-in-time copy of kart/ros2_ws/src/autokart_bringup/config/vehicle_geometry.yaml's
# vehicle_geometry: block, as read 2026-10-22. MEASURED values (see that file's own header:
# "Every number below is USER-MEASURED"), not guesses -- but re-verify against the live file
# before trusting them long after this package was written.
DEFAULT_BUMPER_WIDTH_M = 1.016
DEFAULT_LATERAL_MARGIN_M = 0.15
DEFAULT_CAMERA_TO_FRONT_BUMPER_HORIZONTAL_M = 1.0668
DEFAULT_STOP_GAP_FROM_BUMPER_M = 0.15


class VehicleGeometryError(RuntimeError):
    """Raised when an explicitly configured vehicle_geometry_yaml_path cannot be read or parsed.

    Deliberately never caught-and-ignored by callers: an operator who set this parameter asked
    for the real file to be used, so a bad path must fail closed, not silently fall back to the
    bundled copy (that would hide a real misconfiguration behind what looks like success).
    """


@dataclass(frozen=True)
class GeometryValues:
    bumper_width_m: float
    lateral_margin_m: float
    camera_to_front_bumper_horizontal_m: float
    stop_gap_from_bumper_m: float
    source: str  # "DEFAULT_COPY" or the yaml path actually read

    @property
    def footprint_width_m(self) -> float:
        return self.bumper_width_m + 2.0 * self.lateral_margin_m


def default_geometry() -> GeometryValues:
    return GeometryValues(
        bumper_width_m=DEFAULT_BUMPER_WIDTH_M,
        lateral_margin_m=DEFAULT_LATERAL_MARGIN_M,
        camera_to_front_bumper_horizontal_m=DEFAULT_CAMERA_TO_FRONT_BUMPER_HORIZONTAL_M,
        stop_gap_from_bumper_m=DEFAULT_STOP_GAP_FROM_BUMPER_M,
        source="DEFAULT_COPY",
    )


def load_geometry(path: str) -> GeometryValues:
    """Parse vehicle_geometry.yaml directly. Fails closed (VehicleGeometryError) on any
    missing-file or schema problem -- never guesses a substitute value."""
    if not path:
        raise VehicleGeometryError("EMPTY_VEHICLE_GEOMETRY_YAML_PATH")
    if not os.path.isfile(path):
        raise VehicleGeometryError(f"VEHICLE_GEOMETRY_YAML_NOT_FOUND:{path}")
    try:
        import yaml
        with open(path, "r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle)
        values = raw["vehicle_geometry"]
        return GeometryValues(
            bumper_width_m=float(values["bumper_width_m"]),
            lateral_margin_m=float(values["lateral_margin_m"]),
            camera_to_front_bumper_horizontal_m=float(values["camera_to_front_bumper_horizontal_m"]),
            stop_gap_from_bumper_m=float(values["stop_gap_from_bumper_m"]),
            source=path,
        )
    except VehicleGeometryError:
        raise
    except Exception as exc:  # noqa: BLE001 - any parse/schema error must fail closed, not guess
        raise VehicleGeometryError(
            f"VEHICLE_GEOMETRY_YAML_INVALID:{path}:{type(exc).__name__}:{exc}") from exc
