"""Input sources and the reused-algorithm loader for autokart_odometry.

Two separate concerns live here, mirroring the autokart_hp60c split of
"talk to the data source" vs "the reused algorithm":

1. ``SyntheticSampleSource`` -- a dependency-free, deterministic generator of
   encoder/steering samples (``OdomSample``), so the node can be built, run and
   tested with no VCU and no encoders attached (true in this development
   environment). The other input source, ``actuator_feedback``, is a plain ROS
   subscription to node 04's ``/vehicle/actuator_feedback`` and lives in
   node.py, since it is ROS transport rather than a hardware backend.

2. ``load_wheel_odometry()`` -- lazily imports the existing, already-tested
   ``autokart_localization.odometry`` module (``WheelOdometry``,
   ``OdometryConfig``, ``STATUS_*``) from the ``algorithm_package_path`` ROS
   parameter, never hardcoded, same pattern as ``autokart_hp60c``'s
   ``HP60CBackend``. The bicycle-model integration is never reimplemented
   here. If it cannot be imported, the node fails closed (publishes no
   odometry and no TF at all) -- there is no "safe degenerate" odometry to fall
   back to, unlike the VCU bridge's zero-throttle fallback.

Note on field names: ``WheelOdometry`` exposes ``x_m``, ``y_m``, ``yaw_rad``,
``yaw_rate_radps``, ``speed_mps`` and ``status`` -- and nothing else. It does
NOT track an unsigned ``distance_travelled_total`` separately from signed
displacement (verified by reading odometry.py and grepping the localization,
vehicle-bridge and msgs packages). This node therefore does not claim or
publish such a field. See reports/DEVELOPER_REPORT.md.
"""
from __future__ import annotations

import importlib
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


class OdometryBackendError(RuntimeError):
    """Raised when the reused algorithm cannot be loaded or a source faults."""


@dataclass
class OdomSample:
    stamp_ns: int                      # sample-time domain, used ONLY for dt between samples
    speed_mps: Optional[float]         # signed, measured; None/NaN = unavailable
    axle_counts: Optional[int]         # cumulative raw counts (calibrated-counts mode); None = unavailable
    steering_rad: Optional[float]      # OBSERVED road-wheel angle; None/NaN = unavailable
    valid: bool                        # source's own validity flag
    source: str


class SyntheticSampleSource:
    """Deterministic samples: constant speed and steering, at ``rate_hz``.

    ``axle_counts`` is generated consistently with ``speed_mps`` using the
    SYNTHETIC ``counts_per_m`` passed in (a test value, never a kart
    calibration). Test hooks: ``steering_invalid_after`` (steering becomes
    None), ``speed_invalid_after`` (speed NaN, counts None, valid False),
    ``gap_after`` (one 0.5 s stamp jump), ``nonmonotonic_after`` (one stamp
    goes backwards). Each takes a 1-based sample index; 0 disables it.
    """

    def __init__(self, start_ns: int, rate_hz: float = 50.0, speed_mps: float = 1.0,
                 steering_rad: float = 0.0, counts_per_m: float = 0.0,
                 steering_invalid_after: int = 0, speed_invalid_after: int = 0,
                 gap_after: int = 0, nonmonotonic_after: int = 0):
        if rate_hz <= 0 or not math.isfinite(rate_hz):
            raise OdometryBackendError(f"INVALID_SYNTHETIC_RATE:{rate_hz}")
        self.step_ns = int(1e9 / rate_hz)
        self.speed_mps = speed_mps
        self.steering_rad = steering_rad
        self.counts_per_m = counts_per_m
        self.steering_invalid_after = steering_invalid_after
        self.speed_invalid_after = speed_invalid_after
        self.gap_after = gap_after
        self.nonmonotonic_after = nonmonotonic_after
        self._t_ns = start_ns
        self._index = 0
        self._distance_m = 0.0  # internal only, to keep synthetic counts consistent with speed

    def next_sample(self) -> OdomSample:
        self._index += 1
        i = self._index
        self._t_ns += self.step_ns
        if self.gap_after and i == self.gap_after:
            self._t_ns += 500_000_000
        stamp = self._t_ns
        if self.nonmonotonic_after and i == self.nonmonotonic_after:
            stamp = self._t_ns - 10 * self.step_ns

        dt = self.step_ns / 1e9
        self._distance_m += self.speed_mps * dt
        counts = int(round(self._distance_m * self.counts_per_m)) if self.counts_per_m > 0 else None

        speed_bad = bool(self.speed_invalid_after and i >= self.speed_invalid_after)
        steer_bad = bool(self.steering_invalid_after and i >= self.steering_invalid_after)
        return OdomSample(
            stamp_ns=stamp,
            speed_mps=float("nan") if speed_bad else self.speed_mps,
            axle_counts=None if speed_bad else counts,
            steering_rad=None if steer_bad else self.steering_rad,
            valid=not speed_bad,
            source="SYNTHETIC",
        )


def load_wheel_odometry(algorithm_package_path: str):
    """Return the reused ``autokart_localization.odometry`` module, or raise
    OdometryBackendError with an explicit reason. Never falls back to a local
    copy."""
    if not algorithm_package_path:
        raise OdometryBackendError("ALGORITHM_PATH_NOT_CONFIGURED")
    src = Path(algorithm_package_path).expanduser()
    if not src.is_dir():
        raise OdometryBackendError(f"DEPENDENCY_PATH_NOT_FOUND:{src}")
    src_str = str(src)
    if src_str not in sys.path:
        sys.path.insert(0, src_str)
    try:
        module = importlib.import_module("autokart_localization.odometry")
    except Exception as exc:  # noqa: BLE001 - exercised via test_backends' fault case
        raise OdometryBackendError(f"ALGORITHM_IMPORT_FAILED:{type(exc).__name__}:{exc}") from exc
    for name in ("WheelOdometry", "OdometryConfig", "STATUS_ENCODER_STEER_MODEL",
                 "STATUS_ENCODER_ONLY_NO_YAW", "STATUS_INVALID"):
        if not hasattr(module, name):
            raise OdometryBackendError(f"ALGORITHM_API_MISMATCH:missing {name}")
    return module
