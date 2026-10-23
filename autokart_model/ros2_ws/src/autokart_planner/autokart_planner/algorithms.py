"""Curvature and stopping-distance algorithm backends for autokart_planner.

Same split as autokart_hp60c's SyntheticBackend/HP60CBackend:

- ``BuiltinAlgorithmBackend`` (default, zero external dependency): implements the literal
  formulas from the master architecture doc directly --
      kappa = (x'y'' - y'x'') / (x'^2 + y'^2)^1.5        (centered finite differences)
      d_required = v*t_total_latency + v^2/(2*a_brake_min) + d_margin
  This exists so the package builds/tests with nothing beyond ROS 2 Humble installed, matching
  this library's own portability bar (see autokart_hp60c's precedent and DEPENDENCIES.md). It is
  NOT a competing reimplementation meant to replace an already-qualified algorithm in production;
  ``test_algorithms.py`` cross-validates it against ``ReusedAlgorithmBackend`` whenever the real
  dependency is available in the dev/CI environment, and production/live deployment should select
  ``algorithm_backend: reused``.

- ``ReusedAlgorithmBackend`` (``algorithm_backend: reused``): lazily imports the existing,
  already-tested algorithms this node wraps rather than reimplements:
      autokart_perception.curvature.profile_centerline          (discrete three-point
          signed-circle curvature fit -- the actual algorithm already used by
          kart/ros2_ws's autokart_mapping node)
      autokart_perception.shared_core.BrakeCapability.stopping_distance (the exact
          v*t + v^2/(2a) + margin formula, already implemented and unit-tested elsewhere)
  via the ``algorithm_package_path`` ROS parameter -- never hardcoded, see DEPENDENCIES.md.
"""
from __future__ import annotations

import importlib
import sys
from dataclasses import dataclass
from math import hypot, isfinite
from pathlib import Path
from typing import Protocol


class AlgorithmBackendError(RuntimeError):
    """Raised for any backend fault: bad dependency path, import failure, ambiguous curvature,
    or an unqualified/invalid braking-capability input."""


class AlgorithmBackend(Protocol):
    def curvature_profile(self, points: list) -> list: ...

    def stopping_distance(self, speed_mps: float, *, a_brake_min_mps2: float,
                           t_total_latency_s: float, d_margin_m: float) -> float: ...


def _segment_lengths(points):
    return [hypot(bx - ax, by - ay) for (ax, ay), (bx, by) in zip(points, points[1:])]


@dataclass
class BuiltinAlgorithmBackend:
    min_segment_m: float = 0.02

    def curvature_profile(self, points: list) -> list:
        n = len(points)
        if n < 2:
            raise AlgorithmBackendError("CURVATURE_NEEDS_AT_LEAST_TWO_POINTS")
        if not all(isfinite(c) for p in points for c in p):
            raise AlgorithmBackendError("CURVATURE_NONFINITE_POINT")
        lengths = _segment_lengths(points)
        if any(length < self.min_segment_m for length in lengths):
            raise AlgorithmBackendError("CURVATURE_AMBIGUOUS_DUPLICATE_POINTS")
        if n == 2:
            # Exactly two points define a straight line: kappa=0 is exact here, not a fallback guess.
            return [0.0, 0.0]
        kappas = [0.0] * n
        for i in range(1, n - 1):
            xm1, ym1 = points[i - 1]
            x0, y0 = points[i]
            xp1, yp1 = points[i + 1]
            # Centered finite differences (unit parameter step) for x'(t), y'(t), x''(t), y''(t).
            xd = (xp1 - xm1) / 2.0
            yd = (yp1 - ym1) / 2.0
            xdd = xp1 - 2.0 * x0 + xm1
            ydd = yp1 - 2.0 * y0 + ym1
            denom = (xd * xd + yd * yd) ** 1.5
            kappas[i] = 0.0 if denom <= 1e-12 else (xd * ydd - yd * xdd) / denom
        # Endpoints have no centered neighbor on one side (first derivative there is undefined
        # by this method): reuse the nearest interior value -- conservative (never silently
        # reports 0 curvature at a path's ends when its interior is curved) rather than guessed.
        kappas[0] = kappas[1]
        kappas[-1] = kappas[-2]
        if not all(isfinite(k) for k in kappas):
            raise AlgorithmBackendError("CURVATURE_NONFINITE_RESULT")
        return kappas

    def stopping_distance(self, speed_mps: float, *, a_brake_min_mps2: float,
                           t_total_latency_s: float, d_margin_m: float) -> float:
        if not isfinite(speed_mps) or speed_mps < 0:
            raise AlgorithmBackendError("STOPPING_DISTANCE_INVALID_SPEED")
        if not isfinite(a_brake_min_mps2) or a_brake_min_mps2 <= 0:
            raise AlgorithmBackendError("STOPPING_DISTANCE_BRAKE_CAPABILITY_UNQUALIFIED")
        if not isfinite(t_total_latency_s) or t_total_latency_s < 0:
            raise AlgorithmBackendError("STOPPING_DISTANCE_INVALID_LATENCY")
        if not isfinite(d_margin_m) or d_margin_m < 0:
            raise AlgorithmBackendError("STOPPING_DISTANCE_INVALID_MARGIN")
        return speed_mps * t_total_latency_s + (speed_mps ** 2) / (2.0 * a_brake_min_mps2) + d_margin_m


class ReusedAlgorithmBackend:
    """Thin adapter over the existing, reused ``autokart_perception.curvature``/``shared_core``
    modules. The algorithm-package path is resolved at construction time, not import time, and is
    never hardcoded: it comes from the ``algorithm_package_path`` ROS parameter (see node.py)."""

    def __init__(self, algorithm_package_path: str):
        src = Path(algorithm_package_path).expanduser()
        if not src.is_dir():
            raise AlgorithmBackendError(f"DEPENDENCY_PATH_NOT_FOUND:{src}")
        src_str = str(src)
        # Always move the configured path to the FRONT of sys.path, rather than skipping the
        # insert when it's merely present somewhere else in sys.path. A plain "insert only if
        # absent" (this package's own first draft, and autokart_hp60c's identical pattern in
        # backends.py) is not enough: if some OTHER directory also named ".../autokart_perception"
        # already sits earlier in sys.path (e.g. a stray test fixture's path, or an unrelated
        # environment PYTHONPATH entry) when this path happens to already be present further down,
        # Python's package finder binds the top-level `autokart_perception` name to whichever
        # directory it meets FIRST while walking sys.path -- silently shadowing the dependency this
        # parameter was supposed to select. Found via a real test failure during development (two
        # ReusedAlgorithmBackend instances constructed with different paths in the same process;
        # see test_algorithms.py's `_clean_autokart_perception_imports` fixture and
        # reports/SENIOR_REVIEWER_REPORT.md), not merely theoretical.
        sys.path[:] = [p for p in sys.path if p != src_str]
        sys.path.insert(0, src_str)
        try:
            self._curvature_module = importlib.import_module("autokart_perception.curvature")
            self._shared_core_module = importlib.import_module("autokart_perception.shared_core")
        except Exception as exc:  # noqa: BLE001 - surfaced as a backend fault, not a crash
            raise AlgorithmBackendError(f"SDK_ADAPTER_IMPORT_FAILED:{type(exc).__name__}:{exc}") from exc

    def curvature_profile(self, points: list) -> list:
        samples, status = self._curvature_module.profile_centerline(points)
        if status != "PROVISIONAL_CURVATURE" or not samples:
            raise AlgorithmBackendError(f"CURVATURE_AMBIGUOUS:{status}")
        kappas = [0.0 if s.curvature_m_inv is None else float(s.curvature_m_inv) for s in samples]
        # profile_centerline() returns curvature_m_inv=None at the path's endpoints (undefined by
        # its three-point construction); reuse the nearest interior value, the same conservative
        # convention as BuiltinAlgorithmBackend, rather than silently treating an endpoint as flat.
        if len(kappas) > 2:
            if samples[0].curvature_m_inv is None:
                kappas[0] = kappas[1]
            if samples[-1].curvature_m_inv is None:
                kappas[-1] = kappas[-2]
        return kappas

    def stopping_distance(self, speed_mps: float, *, a_brake_min_mps2: float,
                           t_total_latency_s: float, d_margin_m: float) -> float:
        if not isfinite(speed_mps) or speed_mps < 0:
            raise AlgorithmBackendError("STOPPING_DISTANCE_INVALID_SPEED")
        cap = self._shared_core_module.BrakeCapability(
            minimum_deceleration_mps2=a_brake_min_mps2,
            maximum_delay_s=t_total_latency_s,
            provenance="MEASURED",
            margin_m=d_margin_m,
        )
        # source="LIVE" forces BrakeCapability.valid_for()'s strict branch (provenance must be
        # "MEASURED"), which we always pass explicitly above -- this backend never accepts an
        # unqualified brake capability regardless of which "source" this node itself was started
        # with (that is a separate, node-level provenance concept; see CONTRACT.md).
        if not cap.valid_for("LIVE"):
            raise AlgorithmBackendError("STOPPING_DISTANCE_BRAKE_CAPABILITY_UNQUALIFIED")
        return cap.stopping_distance(speed_mps)


def build_algorithm_backend(name: str, **kwargs) -> AlgorithmBackend:
    if name == "builtin":
        return BuiltinAlgorithmBackend()
    if name == "reused":
        return ReusedAlgorithmBackend(algorithm_package_path=kwargs["algorithm_package_path"])
    raise AlgorithmBackendError(f"UNKNOWN_ALGORITHM_BACKEND:{name}")
