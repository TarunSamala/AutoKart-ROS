"""Single source of truth for kart/camera/bumper geometry. No ROS dependency -- importable from pure-Python
logic (planner, controller cores) and from ROS nodes alike.

The numbers live in exactly one place: kart/ros2_ws/src/autokart_bringup/config/vehicle_geometry.yaml. This
module only loads and derives from them; it must never hardcode a second copy of any of the six raw values.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Optional

CONFIG_PACKAGE = "autokart_bringup"
CONFIG_RELATIVE_PATH = os.path.join("config", "vehicle_geometry.yaml")


def _source_tree_candidates() -> list[str]:
    """Search upward from this file for kart/ros2_ws/src/autokart_bringup/config/vehicle_geometry.yaml --
    the path pytest/plain-python runs use (see docs/runbooks/VERIFY_REPOSITORY.md: no colcon install tree)."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = []
    node = here
    for _ in range(8):
        candidates.append(os.path.join(node, CONFIG_PACKAGE, CONFIG_RELATIVE_PATH))
        node = os.path.dirname(node)
    return candidates


def find_config_path() -> str:
    """Resolve vehicle_geometry.yaml: an installed ament package share directory first (when running
    under a sourced colcon install), then a source-tree search (when running plain pytest, per
    docs/runbooks/VERIFY_REPOSITORY.md)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = os.path.join(get_package_share_directory(CONFIG_PACKAGE), CONFIG_RELATIVE_PATH)
        if os.path.isfile(installed):
            return installed
    except Exception:  # noqa: BLE001 - ament not available/sourced is expected outside a colcon install
        pass
    for candidate in _source_tree_candidates():
        if os.path.isfile(candidate):
            return candidate
    raise FileNotFoundError(
        "vehicle_geometry.yaml not found via ament_index or a source-tree search from "
        f"{os.path.abspath(__file__)} -- is kart/ros2_ws/src/autokart_bringup/config/vehicle_geometry.yaml present?")


@dataclass(frozen=True)
class VehicleGeometry:
    # Raw, user-measured values -- the ONLY place these six numbers should be written as literals.
    camera_model: str
    camera_height_m: float
    camera_pitch_down_deg: float
    camera_to_front_bumper_horizontal_m: float
    bumper_width_m: float
    stop_gap_from_bumper_m: float
    lateral_margin_m: float
    source_path: str = ""

    # Derived (computed, never hand-maintained).
    camera_to_bumper_diagonal_m: float = field(init=False)
    half_bumper_width_m: float = field(init=False)
    camera_forward_distance_at_stop_m: float = field(init=False)
    corridor_minimum_width_m: float = field(init=False)
    camera_pitch_down_rad: float = field(init=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "camera_to_bumper_diagonal_m",
                           math.hypot(self.camera_height_m, self.camera_to_front_bumper_horizontal_m))
        object.__setattr__(self, "half_bumper_width_m", self.bumper_width_m / 2.0)
        object.__setattr__(self, "camera_forward_distance_at_stop_m",
                           self.camera_to_front_bumper_horizontal_m + self.stop_gap_from_bumper_m)
        object.__setattr__(self, "corridor_minimum_width_m", self.bumper_width_m + 2.0 * self.lateral_margin_m)
        object.__setattr__(self, "camera_pitch_down_rad", math.radians(self.camera_pitch_down_deg))

    def format_table(self) -> str:
        rows = [
            ("camera_model", self.camera_model),
            ("camera_height_m", f"{self.camera_height_m:.4f}"),
            ("camera_pitch_down_deg", f"{self.camera_pitch_down_deg:.2f}"),
            ("camera_to_front_bumper_horizontal_m", f"{self.camera_to_front_bumper_horizontal_m:.4f}"),
            ("bumper_width_m", f"{self.bumper_width_m:.4f}"),
            ("stop_gap_from_bumper_m", f"{self.stop_gap_from_bumper_m:.4f}"),
            ("lateral_margin_m", f"{self.lateral_margin_m:.4f}"),
            ("-- derived --", ""),
            ("camera_to_bumper_diagonal_m", f"{self.camera_to_bumper_diagonal_m:.4f}"),
            ("half_bumper_width_m", f"{self.half_bumper_width_m:.4f}"),
            ("camera_forward_distance_at_stop_m", f"{self.camera_forward_distance_at_stop_m:.4f}"),
            ("corridor_minimum_width_m", f"{self.corridor_minimum_width_m:.4f}"),
        ]
        width = max(len(name) for name, _ in rows)
        lines = [f"{name:<{width}} : {value}" for name, value in rows]
        return "\n".join(["Vehicle / camera geometry (source: " + (self.source_path or "defaults") + ")"] + lines)

    def print_table(self) -> None:
        print(self.format_table())


def load_geometry(path: Optional[str] = None) -> VehicleGeometry:
    import yaml

    resolved = path or find_config_path()
    with open(resolved, "r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    values = raw["vehicle_geometry"]
    return VehicleGeometry(
        camera_model=str(values["camera_model"]),
        camera_height_m=float(values["camera_height_m"]),
        camera_pitch_down_deg=float(values["camera_pitch_down_deg"]),
        camera_to_front_bumper_horizontal_m=float(values["camera_to_front_bumper_horizontal_m"]),
        bumper_width_m=float(values["bumper_width_m"]),
        stop_gap_from_bumper_m=float(values["stop_gap_from_bumper_m"]),
        lateral_margin_m=float(values["lateral_margin_m"]),
        source_path=resolved,
    )


GEOMETRY = load_geometry()


def main() -> None:
    GEOMETRY.print_table()


if __name__ == "__main__":
    main()
