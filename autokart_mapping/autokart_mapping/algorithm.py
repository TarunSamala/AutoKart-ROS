"""Lazy-import wrapper around the existing, canonical ``autokart_perception`` geometry/graph
algorithms.

This module never vendors or reimplements Delaunay triangulation, the Gabriel graph filter, gate
selection, centerline/curvature fitting, or the observed two-ring skidpad fit. It imports the
existing ``autokart_perception`` package (``application.CoreRuntime``, ``shared_core.AlgorithmCore``
via ``CoreRuntime().algorithms``, ``shared_core.fit_skidpad_topology``, ``shared_core.PoseEstimate``,
``types.Cone``) from a path supplied at runtime via the ``algorithm_package_path`` ROS parameter --
never hardcoded -- exactly the pattern ``autokart_hp60c/backends.py`` uses for the HP60C SDK.

Canonical source (per ../../docs/decisions/CANONICAL_COMPONENTS.md and AUDIT_PHASE0.md):
``autonomy_stack_phase1/phase_01_perception/src/autokart_perception/{application,shared_core,
geometry,gabriel,gates,boundaries,centerline,curvature,safety,types,sdk}.py``.
"""
from __future__ import annotations

import importlib
import sys
from pathlib import Path


class MappingAlgorithmError(RuntimeError):
    """Raised when the underlying autokart_perception geometry package cannot be loaded."""


class MappingAlgorithm:
    """Holds the lazily-imported CoreRuntime class/types needed by MappingEngine.

    Constructed once at node startup (not at module import time), so this package builds and can be
    unit-tested (its own ROS-adapter glue, not the reused algorithm) without the dependency present.
    """

    def __init__(self, algorithm_package_path: str):
        # An empty/blank path must fail closed explicitly, not be treated as a relative path and
        # silently resolved to the current working directory: Path("").expanduser().is_dir() is
        # True (it resolves to "."), which on a host where the real dependency happens to already
        # be importable some other way (e.g. a global editable pip install) would make this node
        # appear configured when the required ROS parameter was never actually set. See
        # reports/SENIOR_REVIEWER_REPORT.md.
        if not algorithm_package_path or not algorithm_package_path.strip():
            raise MappingAlgorithmError("ALGORITHM_PACKAGE_PATH_NOT_CONFIGURED")
        src = Path(algorithm_package_path).expanduser()
        if not src.is_dir():
            raise MappingAlgorithmError(f"DEPENDENCY_PATH_NOT_FOUND:{src}")
        src_str = str(src)
        if src_str not in sys.path:
            sys.path.insert(0, src_str)
        try:
            application = importlib.import_module("autokart_perception.application")
            shared_core = importlib.import_module("autokart_perception.shared_core")
            types_module = importlib.import_module("autokart_perception.types")
        except Exception as exc:  # noqa: BLE001 - deliberate: surfaced as a reported node fault,
            # exercised explicitly by test_algorithm.py's "rejects path without autokart_perception"
            # case, not swallowed.
            raise MappingAlgorithmError(f"ALGORITHM_IMPORT_FAILED:{type(exc).__name__}:{exc}") from exc
        self.CoreRuntime = application.CoreRuntime
        self.PoseEstimate = shared_core.PoseEstimate
        self.fit_skidpad_topology = shared_core.fit_skidpad_topology
        self.Cone = types_module.Cone

    def new_core_runtime(self):
        """A fresh ``CoreRuntime`` (and therefore a fresh ``AlgorithmCore``/``LocalConeMap``/
        ``WindingTracker``): used both at startup and whenever the engine must discard accumulated
        map/pose state (odometry reset, stale input, invalid frame) rather than silently keep
        tracking cones against a pose history that is no longer trustworthy."""
        return self.CoreRuntime()
