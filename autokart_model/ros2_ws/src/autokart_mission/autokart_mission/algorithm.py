"""Lazy-import wrapper around the existing, canonical mission/braking FSM logic.

This module never vendors or reimplements the skidpad gate-crossing/winding-number progress state
machine or the braking stop-distance physics. It imports the existing, already-tested
``autokart_planner.mission`` module (``PlannerEngine``, ``SkidpadProgress``) from
``kart/ros2_ws/src/autokart_nodes/autokart_planner`` -- the live production workspace's planner
package -- exactly as a pure-Python dependency, via a path supplied at runtime (never hardcoded),
matching the pattern ``autokart_hp60c/backends.py`` uses for the HP60C SDK and
``autokart_mapping/algorithm.py`` uses for the geometry algorithms.

``autokart_planner.mission`` itself has two of its own transitive dependencies that must also be
importable before it is imported, both resolved the same lazy, parameterized way:

- ``autokart_perception.shared_core`` (``BrakeCapability``, ``PoseEstimate``) -- same canonical
  source as ``autokart_mapping``'s dependency (``autonomy_stack_phase1/phase_01_perception/src``).
- ``autokart_vehicle_geometry.geometry`` (the ``GEOMETRY`` singleton) -- this one is unusual: the
  module loads ``kart/ros2_ws/src/autokart_bringup/config/vehicle_geometry.yaml`` at *import time*
  (not lazily), via a source-tree search that walks upward from its own file location looking for a
  sibling ``autokart_bringup/config/vehicle_geometry.yaml``. That means the real, on-disk
  ``kart/ros2_ws/src`` directory structure must be used (not a relocated copy) unless the caller sets
  up an equivalent sibling layout elsewhere -- see DEPENDENCIES.md for the portability consequence of
  this.

This node does NOT modify, re-derive, or duplicate the substance of any of these three reused
modules. ``mission.py`` is a pure-Python import, with no ROS dependency of its own.
"""
from __future__ import annotations

import importlib
import sys
from pathlib import Path


class MissionAlgorithmError(RuntimeError):
    """Raised when the underlying mission/braking FSM or one of its transitive dependencies cannot
    be loaded."""


class MissionAlgorithm:
    """Holds the lazily-imported PlannerEngine/SkidpadProgress classes and the shared_core types
    this node's own ROS adapter needs (PoseEstimate, BrakeCapability)."""

    def __init__(self, algorithm_package_path: str, vehicle_geometry_package_path: str,
                 planner_package_path: str):
        for label, value in (("algorithm_package_path", algorithm_package_path),
                             ("vehicle_geometry_package_path", vehicle_geometry_package_path),
                             ("planner_package_path", planner_package_path)):
            if not value or not value.strip():
                # Each of the three paths must be explicitly configured. An empty string must never
                # be silently treated as a relative path that happens to resolve to "." (a directory)
                # -- see autokart_mapping/algorithm.py's own documented fix for the same hazard.
                raise MissionAlgorithmError(f"{label.upper()}_NOT_CONFIGURED")
        paths = {
            "algorithm_package_path": algorithm_package_path,
            "vehicle_geometry_package_path": vehicle_geometry_package_path,
            "planner_package_path": planner_package_path,
        }
        for label, value in paths.items():
            src = Path(value).expanduser()
            if not src.is_dir():
                raise MissionAlgorithmError(f"DEPENDENCY_PATH_NOT_FOUND:{label}:{src}")
            src_str = str(src)
            if src_str not in sys.path:
                sys.path.insert(0, src_str)
        try:
            shared_core = importlib.import_module("autokart_perception.shared_core")
        except Exception as exc:  # noqa: BLE001 - reported, not swallowed; see test_algorithm.py
            raise MissionAlgorithmError(
                f"ALGORITHM_IMPORT_FAILED:autokart_perception:{type(exc).__name__}:{exc}") from exc
        try:
            vehicle_geometry = importlib.import_module("autokart_vehicle_geometry.geometry")
        except Exception as exc:  # noqa: BLE001
            raise MissionAlgorithmError(
                f"ALGORITHM_IMPORT_FAILED:autokart_vehicle_geometry:{type(exc).__name__}:{exc}") from exc
        try:
            location = Path(planner_package_path).expanduser() / "autokart_planner" / "mission.py"
            spec = importlib.util.spec_from_file_location("_autokart_legacy_mission", location)
            mission_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mission_module)
        except Exception as exc:  # noqa: BLE001
            raise MissionAlgorithmError(
                f"ALGORITHM_IMPORT_FAILED:autokart_planner.mission:{type(exc).__name__}:{exc}") from exc
        self.PoseEstimate = shared_core.PoseEstimate
        self.BrakeCapability = shared_core.BrakeCapability
        self.GEOMETRY = vehicle_geometry.GEOMETRY
        self.PlannerEngine = mission_module.PlannerEngine
        self.SkidpadProgress = mission_module.SkidpadProgress
