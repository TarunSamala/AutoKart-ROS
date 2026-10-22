"""Lazy-import adapter around the existing, already-reviewed VCU arming safety
supervisor (``kart/ros2_ws/src/autokart_vcu_safety/autokart_vcu_safety/core.py``).

This package does not reimplement the state machine, the startup-stage
evaluation, or the fault logic. It imports the real ``autokart_vcu_safety.core``
module from a path given at runtime by a ROS parameter (never hardcoded into
source), exactly the pattern ``autokart_hp60c.backends.HP60CBackend`` uses for
the HP60C SDK. If the dependency path is wrong or missing, this adapter fails
closed with an explicit, typed error -- it never falls back to a
reimplementation and never silently returns an "ARM allowed" answer.

Distinct, separate component this adapter does NOT wrap or replace:
``kart/ros2_ws/src/autokart_safety_bridge`` (``front_safety_envelope_monitor``)
is a diagnostic-only front-obstacle envelope classifier with
``vehicle_authority`` permanently OFF. It is not an arming authority and this
node does not subsume it -- see CONTRACT.md and the developer report for the
recommendation to eventually unify both under one "autokart_safety" node.
"""
from __future__ import annotations

import importlib
import sys
from pathlib import Path
from types import ModuleType
from typing import Optional


class SafetyCoreError(RuntimeError):
    """Raised for any adapter fault: missing dependency path, import failure,
    or an import that does not expose the expected core API."""


REQUIRED_ATTRS = ("SafetyLogic", "Inputs", "Config", "VcuView", "evaluate_startup")


def load_safety_core(package_path: str) -> ModuleType:
    """Resolve and import the real ``autokart_vcu_safety.core`` module.

    ``package_path`` must be the directory that *contains* the
    ``autokart_vcu_safety`` Python package (i.e. the parent of
    ``autokart_vcu_safety/__init__.py``), matching
    ``kart/ros2_ws/src/autokart_vcu_safety`` in the canonical workspace. Never
    hardcoded here: the caller (node.py) reads it from the
    ``vcu_safety_package_path`` ROS parameter, set per-deployment in a launch
    config file.
    """
    if not package_path:
        raise SafetyCoreError("DEPENDENCY_PATH_NOT_SET")
    src = Path(package_path).expanduser()
    if not src.is_dir():
        raise SafetyCoreError(f"DEPENDENCY_PATH_NOT_FOUND:{src}")
    src_str = str(src)
    # Always move src_str to the FRONT of sys.path, even if it is already
    # present further back. A plain "insert only if absent" is not enough: if
    # an earlier call (a prior misconfigured path, or -- as in this package's
    # own tests -- a stub fixture) already inserted some OTHER path at the
    # front, that other path keeps shadowing this one forever, because a
    # regular (non-namespace) package is resolved from the first matching
    # sys.path entry and every entry after it is ignored. Removing any
    # existing occurrence first guarantees src_str always wins.
    while src_str in sys.path:
        sys.path.remove(src_str)
    sys.path.insert(0, src_str)
    # Purge any previously cached autokart_vcu_safety module(s) first. Python's
    # import system caches by dotted name in sys.modules, not by source path:
    # without this, a second call to load_safety_core with a DIFFERENT
    # package_path (e.g. a prior failed/misconfigured path, or a stub used in
    # a test) would silently return the FIRST path's cached module forever,
    # regardless of the sys.path change above. That is exactly the kind of
    # silent-wrong-answer failure this adapter must never produce. Discovered
    # via this package's own test suite (test_adapter.py's stub-rejection
    # test ran before the real-core test in the same pytest process and
    # poisoned the cache); see test_reimport_after_a_bad_path_loads_the_new_real_module.
    for name in list(sys.modules):
        if name == "autokart_vcu_safety" or name.startswith("autokart_vcu_safety."):
            del sys.modules[name]
    try:
        core = importlib.import_module("autokart_vcu_safety.core")
    except Exception as exc:  # noqa: BLE001 - deliberate: any import failure must
        # surface as a typed, reported fault, never an uncaught exception that
        # kills node construction.
        raise SafetyCoreError(f"SDK_ADAPTER_IMPORT_FAILED:{type(exc).__name__}:{exc}") from exc
    missing = [name for name in REQUIRED_ATTRS if not hasattr(core, name)]
    if missing:
        raise SafetyCoreError(f"SDK_ADAPTER_API_MISMATCH:missing={','.join(missing)}")
    return core


class SyntheticInputsSource:
    """Deterministic, dependency-free upstream signal source.

    Mirrors ``autokart_hp60c.backends.SyntheticBackend``'s role: lets this
    package build, run and be tested on any machine with no live VCU bridge,
    odometry, perception, mapping, planner or controller node running. Starts
    every channel as "never seen" (fail-safe: nothing reads as fresh until a
    real sample explicitly arrives), exactly matching how the real node's
    freshness dictionary starts empty before any subscription callback fires.
    """

    def __init__(self):
        self.last_seen: dict[str, float] = {}

    def touch(self, key: str, now: float) -> None:
        self.last_seen[key] = now

    def age(self, key: str, now: float) -> Optional[float]:
        seen = self.last_seen.get(key)
        return None if seen is None else now - seen
