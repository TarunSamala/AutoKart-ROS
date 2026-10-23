"""Geometry configuration adapter.

Bundled dimensions are historical operator-reported values; they are not survey evidence or
physical qualification. Explicit YAML may select a source file but never verifies its content.
Planner live qualification requires separate measured body extents and limits.
"""
from __future__ import annotations

import os
from dataclasses import dataclass

# Point-in-time copy of kart/ros2_ws/src/autokart_bringup/config/vehicle_geometry.yaml's
# vehicle_geometry: block, as read 2026-10-22. MEASURED values (see that file's own header:
# "Every number below is USER-MEASURED"), historical operator report; independently re-verify
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
