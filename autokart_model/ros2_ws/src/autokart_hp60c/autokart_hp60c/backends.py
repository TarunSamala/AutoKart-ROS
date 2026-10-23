"""Camera acquisition backends for the autokart_hp60c node.

A backend is anything with ``start()``, ``read() -> Frame``, ``close()`` where
``Frame`` matches ``autokart_perception.types.Frame`` (rgb, depth, timestamps,
sequence ids, Intrinsics, depth_scale, aligned, source, drops, calibration dict).

``SyntheticBackend`` has no external dependency and is what makes this package
buildable and testable on any machine, including one with no HP60C hardware or
vendor SDK. ``HP60CBackend`` is a thin adapter around the existing, already
hardware-qualified ``autokart_perception.camera.HP60C`` ctypes wrapper; it is
imported lazily so the package never fails to build or load for lack of it.

This node does not reimplement HP60C SDK access. It reuses the existing
``autokart_perception.camera.HP60C`` class as the single source of truth for
talking to the native ``hp60c_bridge`` library, per DEPENDENCIES.md.
"""
from __future__ import annotations

import importlib
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Protocol


class CameraBackendError(RuntimeError):
    """Raised for any backend fault: connection, staleness, format, dependency."""


@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    def valid(self) -> bool:
        import math
        return all(math.isfinite(v) for v in (self.fx, self.fy, self.cx, self.cy)) and self.fx > 0 and self.fy > 0


@dataclass
class Frame:
    rgb: "object"            # numpy.ndarray[h,w,3] uint8
    depth: "object"          # numpy.ndarray[h,w] uint16
    timestamp_ns: int
    rgb_timestamp: int
    depth_timestamp: int
    rgb_sequence: int
    depth_sequence: int
    intrinsics: Intrinsics
    depth_scale: float
    aligned: bool
    source: str
    drops: int = 0
    received_frames: int = 0
    calibration: dict = field(default_factory=dict)


class CameraBackend(Protocol):
    def start(self) -> None: ...
    def read(self) -> Frame: ...
    def close(self) -> None: ...


class SyntheticBackend:
    """Deterministic fake frames. Used for CI, portability checks and unit
    tests on machines without HP60C hardware. Never claims LIVE provenance."""

    def __init__(self, width: int = 640, height: int = 480, period_s: float = 0.08):
        import numpy as np
        self._np = np
        self.width = width
        self.height = height
        self.period_s = period_s
        self._seq = 0
        self._started = False

    def start(self) -> None:
        self._started = True

    def read(self) -> Frame:
        if not self._started:
            raise CameraBackendError("BACKEND_NOT_STARTED")
        np = self._np
        self._seq += 1
        rgb = np.full((self.height, self.width, 3), fill_value=(self._seq * 7) % 256, dtype=np.uint8)
        depth = np.full((self.height, self.width), fill_value=1500, dtype=np.uint16)
        time.sleep(self.period_s)
        # Acquisition completes after the configured frame period.  Stamping
        # before the wait would report every synthetic frame as already old
        # and can consume most of the downstream freshness budget.
        now_ns = time.monotonic_ns()
        return Frame(
            rgb=rgb, depth=depth, timestamp_ns=now_ns,
            rgb_timestamp=now_ns // 1_000_000, depth_timestamp=now_ns // 1_000_000,
            rgb_sequence=self._seq, depth_sequence=self._seq,
            intrinsics=Intrinsics(fx=605.0, fy=605.0, cx=self.width / 2.0, cy=self.height / 2.0),
            depth_scale=0.001, aligned=True, source="SYNTHETIC", drops=0, received_frames=self._seq,
        )

    def close(self) -> None:
        self._started = False


class HP60CBackend:
    """Adapter over the existing, reused ``autokart_perception.camera.HP60C``.

    The algorithm-package path is resolved at construction time, not import
    time, and is never hardcoded: it comes from the ``algorithm_package_path``
    ROS parameter (see node.py), keeping this package portable to a workspace
    that places the dependency somewhere else. See DEPENDENCIES.md.
    """

    def __init__(self, algorithm_package_path: str, library: Optional[str], configuration: Optional[str],
                 max_age_ms: int = 250):
        if not algorithm_package_path.strip():
            raise CameraBackendError("ALGORITHM_PACKAGE_PATH_NOT_CONFIGURED")
        src = Path(algorithm_package_path).expanduser()
        if not src.is_dir():
            raise CameraBackendError(f"DEPENDENCY_PATH_NOT_FOUND:{src}")
        src_str = str(src)
        sys.path[:] = [x for x in sys.path if x != src_str]
        sys.path.insert(0, src_str)
        try:
            camera_module = importlib.import_module("autokart_perception.camera")
            if not Path(camera_module.__file__).resolve().is_relative_to(src.resolve()):
                raise ValueError("DEPENDENCY_ORIGIN_MISMATCH")
        except Exception as exc:  # pragma: no cover - exercised via test_backends fault case
            raise CameraBackendError(f"SDK_ADAPTER_IMPORT_FAILED:{type(exc).__name__}:{exc}") from exc
        self._HP60C = camera_module.HP60C
        self._CameraError = camera_module.CameraError
        self._impl = self._HP60C(
            library=Path(library) if library else None,
            max_age_ms=max_age_ms,
            configuration=Path(configuration) if configuration else None,
        )

    def start(self) -> None:
        try:
            self._impl.start()
        except self._CameraError as exc:
            raise CameraBackendError(str(exc)) from exc

    def read(self) -> Frame:
        try:
            f = self._impl.read()
        except self._CameraError as exc:
            raise CameraBackendError(str(exc)) from exc
        return Frame(
            rgb=f.rgb, depth=f.depth, timestamp_ns=f.timestamp_ns,
            rgb_timestamp=f.rgb_timestamp, depth_timestamp=f.depth_timestamp,
            rgb_sequence=f.rgb_sequence, depth_sequence=f.depth_sequence,
            intrinsics=Intrinsics(f.intrinsics.fx, f.intrinsics.fy, f.intrinsics.cx, f.intrinsics.cy),
            depth_scale=f.depth_scale, aligned=f.aligned, source=f.source,
            drops=f.drops, received_frames=f.received_frames, calibration=dict(getattr(f, "calibration", {})),
        )

    def close(self) -> None:
        self._impl.close()


def build_backend(name: str, **kwargs) -> CameraBackend:
    if name == "synthetic":
        return SyntheticBackend(**{k: v for k, v in kwargs.items() if k in ("width", "height", "period_s")})
    if name == "hp60c":
        return HP60CBackend(
            algorithm_package_path=kwargs["algorithm_package_path"],
            library=kwargs.get("library") or None,
            configuration=kwargs.get("configuration") or None,
            max_age_ms=kwargs.get("max_age_ms", 250),
        )
    raise CameraBackendError(f"UNKNOWN_BACKEND:{name}")
