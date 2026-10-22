"""Pure Pursuit + longitudinal-PID backends for autokart_controller.

Same split as autokart_hp60c's SyntheticBackend/HP60CBackend and autokart_planner's
BuiltinAlgorithmBackend/ReusedAlgorithmBackend:

- ``BuiltinPursuitBackend`` (default, zero external dependency): a direct, independently-coded
  implementation of the EXACT same formulas as the already-existing, already-tested
  ``kart_sim.autonomy.PurePursuitController``/``LongitudinalPidController``
  (`kart/kart_sim/autonomy.py`) -- lookahead point selection, rate-limited curvature-based
  steering, and the conditional-anti-windup longitudinal PID -- verified formula-for-formula
  against that source (see the per-line comments below) and cross-validated numerically in
  ``test_pursuit_backends.py``. It exists so this package builds/tests with nothing beyond ROS 2
  Humble installed, matching this library's own portability bar; production/live deployment
  should select ``pursuit_backend: reused``.

- ``ReusedPursuitBackend`` (``pursuit_backend: reused``): lazily imports and directly calls the
  real, unmodified ``kart_sim.autonomy.PurePursuitController.compute()`` and
  ``LongitudinalPidController.compute()`` -- the literal existing Pure Pursuit implementation this
  node is meant to wrap, per the mission statement ("do not re-derive Pure Pursuit math yourself,
  it already exists and is tested") -- via the ``kart_sim_package_path`` ROS parameter, never
  hardcoded (see DEPENDENCIES.md). Deliberately calls the two controllers directly rather than
  going through ``kart_sim.autonomy.ControllerManager.command()``: that convenience wrapper
  discards ``ControllerOutput``'s ``cross_track_error_m``/``heading_error_rad`` and
  ``LongitudinalOutput``'s ``error_mps`` when it packages a ``MotionCommand`` -- exactly the
  tracking diagnostics the mission statement asks this node to surface -- so this backend reuses
  the two constituent, already-tested classes directly instead of their lossy packaging.

ACTUAL steering formula reused/replicated (verified against kart/kart_sim/autonomy.py, NOT assumed
from the master architecture doc's own notation -- see CONTRACT.md for the discrepancies):

    lookahead = max(0.1, lookahead_m + speed_lookahead_gain_s * max(0, u_mps))   # no upper clamp
    (x_local, y_local) = target point in the vehicle body frame
    distance = max(0.1, hypot(x_local, y_local))
    requested_steering = atan2(2 * wheelbase_m * y_local, distance ** 2)
    steering = clamp(rate_limit(requested_steering), -maximum_steering_rad, maximum_steering_rad)

This is the curvature form of Pure Pursuit (`kappa = 2*y_local/distance**2`,
`delta = atan(wheelbase * kappa)`), algebraically equivalent to the textbook
`delta = atan(2*L*sin(alpha)/L_d)` when `distance == L_d` (which holds whenever a trajectory point
exists at exactly the lookahead distance; the actual code picks the first point at or beyond
`lookahead`, so `distance` is that point's true range, not necessarily the nominal `L_d`).
"""
from __future__ import annotations

import importlib
import sys
from dataclasses import dataclass, field
from math import atan2, cos, hypot, isfinite, pi, sin
from pathlib import Path
from typing import Protocol


class PursuitBackendError(RuntimeError):
    """Raised for any backend fault: bad dependency path, import failure, invalid
    wheelbase/timing, or an unqualified mechanical steering limit."""


def _require_under(module, root: Path, name: str) -> None:
    module_file = getattr(module, "__file__", None)
    if not module_file:
        raise PursuitBackendError(f"DEPENDENCY_PATH_MISMATCH:{name}:no __file__")
    resolved, root_resolved = Path(module_file).resolve(), root.resolve()
    if root_resolved not in resolved.parents:
        raise PursuitBackendError(
            f"DEPENDENCY_PATH_MISMATCH:{name} imported from {resolved}, not under configured {root_resolved}")


def _normalize_angle(angle_rad: float) -> float:
    return (angle_rad + pi) % (2.0 * pi) - pi


class PursuitBackend(Protocol):
    def compute(self, *, pose_x: float, pose_y: float, pose_yaw_rad: float, pose_speed_mps: float,
                path_points_with_heading_speed: list, dt_s: float, safety_speed_limit_mps: float,
                brake_stuck: bool = False) -> dict: ...


@dataclass
class BuiltinPursuitBackend:
    wheelbase_m: float
    lookahead_m: float = 1.5
    speed_lookahead_gain_s: float = 0.15
    maximum_steering_rad: float = 0.6
    maximum_steering_rate_radps: float = 3.0
    kp: float = 0.35
    ki: float = 0.12
    kd: float = 0.0
    integral_limit: float = 4.0
    _last_steering_rad: float = field(default=0.0, init=False, repr=False)
    _integral: float = field(default=0.0, init=False, repr=False)
    _previous_error: float = field(default=0.0, init=False, repr=False)

    def __post_init__(self) -> None:
        if not isfinite(self.wheelbase_m) or self.wheelbase_m <= 0.0:
            raise PursuitBackendError("WHEELBASE_UNQUALIFIED")
        if not isfinite(self.maximum_steering_rad) or self.maximum_steering_rad <= 0.0:
            raise PursuitBackendError("STEERING_LIMIT_UNQUALIFIED")

    def reset(self) -> None:
        """Forget rate-limiter memory and PID state. Called on every rejected cycle: the node has
        just published steering=0/speed=0, so the next valid command must start from that, not from
        pre-fault state (a review-found stale-state leak; see test_controller_core.py)."""
        self._last_steering_rad = 0.0
        self._integral = 0.0
        self._previous_error = 0.0

    def compute(self, *, pose_x: float, pose_y: float, pose_yaw_rad: float, pose_speed_mps: float,
                path_points_with_heading_speed: list, dt_s: float, safety_speed_limit_mps: float,
                brake_stuck: bool = False) -> dict:
        if dt_s <= 0.0 or not isfinite(dt_s):
            raise PursuitBackendError("CONTROLLER_TIMING_INVALID")
        points = path_points_with_heading_speed
        nearest = min(range(len(points)),
                     key=lambda i: (points[i][0] - pose_x) ** 2 + (points[i][1] - pose_y) ** 2)
        # Lower-clamped only (0.1 m floor), matching kart_sim.autonomy.PurePursuitController.compute
        # exactly -- it has NO upper (Lmax) clamp despite the master architecture doc's
        # clamp(L0+kv|v|, Lmin, Lmax) notation. See CONTRACT.md.
        lookahead = max(0.1, self.lookahead_m + self.speed_lookahead_gain_s * max(0.0, pose_speed_mps))
        target = points[-1]
        for point in points[nearest:]:
            if hypot(point[0] - pose_x, point[1] - pose_y) >= lookahead:
                target = point
                break
        dx, dy = target[0] - pose_x, target[1] - pose_y
        x_local = cos(pose_yaw_rad) * dx + sin(pose_yaw_rad) * dy
        y_local = -sin(pose_yaw_rad) * dx + cos(pose_yaw_rad) * dy
        distance = max(0.1, hypot(x_local, y_local))
        requested = atan2(2.0 * self.wheelbase_m * y_local, distance * distance)
        rate = self.maximum_steering_rate_radps * dt_s
        steering = max(self._last_steering_rad - rate, min(self._last_steering_rad + rate, requested))
        steering = max(-self.maximum_steering_rad, min(self.maximum_steering_rad, steering))
        self._last_steering_rad = steering
        heading_error = _normalize_angle(target[2] - pose_yaw_rad)
        target_speed_mps = target[3]

        target_clamped = max(0.0, min(target_speed_mps, safety_speed_limit_mps))
        error = target_clamped - pose_speed_mps
        proposed_integral = max(-self.integral_limit, min(self.integral_limit, self._integral + error * dt_s))
        derivative = (error - self._previous_error) / dt_s
        raw = self.kp * error + self.ki * proposed_integral + self.kd * derivative
        output = max(-1.0, min(1.0, raw))
        # Conditional integration: simple deterministic anti-windup, same convention as
        # kart_sim.autonomy.LongitudinalPidController.compute.
        if output == raw or (output > 0.0 and error < 0.0) or (output < 0.0 and error > 0.0):
            self._integral = proposed_integral
        self._previous_error = error
        drive = max(0.0, output)
        brake = max(0.0, -output)
        if brake_stuck:
            drive = 0.0

        return {
            "steering_rad": steering, "speed_mps": target_clamped, "brake": brake, "drive": drive,
            "cross_track_error_m": y_local, "heading_error_rad": heading_error,
            "speed_tracking_error_mps": error,
        }


class ReusedPursuitBackend:
    """Thin adapter directly calling the existing, reused
    ``kart_sim.autonomy.PurePursuitController``/``LongitudinalPidController``. The dependency path
    is resolved at construction time, not import time, and is never hardcoded: it comes from the
    ``kart_sim_package_path`` ROS parameter (see node.py)."""

    def __init__(self, kart_sim_package_path: str, *, wheelbase_m: float, lookahead_m: float,
                 speed_lookahead_gain_s: float, maximum_steering_rad: float,
                 maximum_steering_rate_radps: float):
        src = Path(kart_sim_package_path).expanduser()
        if not src.is_dir():
            raise PursuitBackendError(f"DEPENDENCY_PATH_NOT_FOUND:{src}")
        src_str = str(src)
        # Always move to the front -- see autokart_planner/algorithms.py::ReusedAlgorithmBackend
        # for why "insert only if absent" is not sufficient (a real bug found and fixed there).
        sys.path[:] = [p for p in sys.path if p != src_str]
        sys.path.insert(0, src_str)
        try:
            self._autonomy = importlib.import_module("kart_sim.autonomy")
            self._types = importlib.import_module("kart_sim.types")
        except Exception as exc:  # noqa: BLE001 - surfaced as a backend fault, not a crash
            raise PursuitBackendError(f"SDK_ADAPTER_IMPORT_FAILED:{type(exc).__name__}:{exc}") from exc
        # sys.path order alone does NOT guarantee the configured path was used: a meta-path finder
        # (e.g. an `__editable__` pip install, which this dev environment has for kart_sim via
        # ~/.local/.../__editable___auto_kart_simulator_0_1_0_finder.py) or an already-cached
        # sys.modules entry resolves `kart_sim` before sys.path is consulted. Found by this node's
        # own fault test (a path with no kart_sim/autonomy.py silently "succeeded" by importing the
        # real one from elsewhere). Verify, and fail closed on a mismatch.
        _require_under(self._autonomy, src, "kart_sim.autonomy")
        _require_under(self._types, src, "kart_sim.types")
        if not isfinite(maximum_steering_rad) or maximum_steering_rad <= 0.0:
            # kart_sim.PurePursuitController itself does not validate this; keep parity with the
            # builtin backend so neither silently accepts a zero/negative mechanical limit.
            raise PursuitBackendError("STEERING_LIMIT_UNQUALIFIED")
        self._pursuit_kwargs = dict(
            lookahead_m=lookahead_m, speed_lookahead_gain_s=speed_lookahead_gain_s,
            wheelbase_m=wheelbase_m, maximum_steering_rad=maximum_steering_rad,
            maximum_steering_rate_radps=maximum_steering_rate_radps)
        try:
            self._pursuit = self._autonomy.PurePursuitController(**self._pursuit_kwargs)
        except self._types.SimulationError as exc:
            raise PursuitBackendError(f"WHEELBASE_UNQUALIFIED:{exc}") from exc
        self._longitudinal = self._autonomy.LongitudinalPidController()

    def reset(self) -> None:
        """Same contract as BuiltinPursuitBackend.reset. Uses only public API of the reused classes:
        a fresh PurePursuitController (it has no public reset; its rate-limiter memory is a private
        field) and LongitudinalPidController.reset()."""
        self._pursuit = self._autonomy.PurePursuitController(**self._pursuit_kwargs)
        self._longitudinal.reset()

    def compute(self, *, pose_x: float, pose_y: float, pose_yaw_rad: float, pose_speed_mps: float,
                path_points_with_heading_speed: list, dt_s: float, safety_speed_limit_mps: float,
                brake_stuck: bool = False) -> dict:
        state = self._types.PlanarKartState(
            time_s=0.0, x_m=pose_x, y_m=pose_y, yaw_rad=pose_yaw_rad, u_mps=pose_speed_mps)
        points = []
        time_s = 0.0
        previous = None
        timing_speed = max(pose_speed_mps, 0.1)
        for x, y, heading, speed in path_points_with_heading_speed:
            if previous is not None:
                time_s += max(hypot(x - previous[0], y - previous[1]), 1e-3) / timing_speed
            points.append(self._autonomy.TrajectoryPoint(
                time_s=time_s, x_m=float(x), y_m=float(y), heading_rad=float(heading), speed_mps=float(speed)))
            previous = (x, y)
        trajectory = self._autonomy.Trajectory(
            header=self._autonomy.MessageHeader(stamp_s=0.0, frame_id="odom", sequence=0, source="autokart_controller"),
            points=tuple(points), valid_until_s=1e9)
        try:
            lateral = self._pursuit.compute(state, trajectory, dt_s)
            longitudinal = self._longitudinal.compute(
                lateral.target_speed_mps, pose_speed_mps, dt_s,
                safety_speed_limit_mps=safety_speed_limit_mps, brake_stuck=brake_stuck)
        except self._types.SimulationError as exc:
            raise PursuitBackendError(f"CONTROLLER_COMPUTE_ERROR:{exc}") from exc
        return {
            "steering_rad": float(lateral.steering_rad), "speed_mps": float(longitudinal.target_speed_mps),
            "brake": float(longitudinal.brake), "drive": float(longitudinal.drive),
            "cross_track_error_m": float(lateral.cross_track_error_m),
            "heading_error_rad": float(lateral.heading_error_rad),
            "speed_tracking_error_mps": float(longitudinal.error_mps),
        }


def build_pursuit_backend(name: str, **kwargs) -> PursuitBackend:
    if name == "builtin":
        return BuiltinPursuitBackend(
            wheelbase_m=kwargs["wheelbase_m"], lookahead_m=kwargs.get("lookahead_m", 1.5),
            speed_lookahead_gain_s=kwargs.get("speed_lookahead_gain_s", 0.15),
            maximum_steering_rad=kwargs.get("maximum_steering_rad", 0.6),
            maximum_steering_rate_radps=kwargs.get("maximum_steering_rate_radps", 3.0))
    if name == "reused":
        return ReusedPursuitBackend(
            kart_sim_package_path=kwargs["kart_sim_package_path"], wheelbase_m=kwargs["wheelbase_m"],
            lookahead_m=kwargs.get("lookahead_m", 1.5),
            speed_lookahead_gain_s=kwargs.get("speed_lookahead_gain_s", 0.15),
            maximum_steering_rad=kwargs.get("maximum_steering_rad", 0.6),
            maximum_steering_rate_radps=kwargs.get("maximum_steering_rate_radps", 3.0))
    raise PursuitBackendError(f"UNKNOWN_PURSUIT_BACKEND:{name}")
