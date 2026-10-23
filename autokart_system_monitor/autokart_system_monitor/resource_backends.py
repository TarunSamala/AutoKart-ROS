"""CPU/RAM sampling backends. Dependency-free by default: psutil is imported
lazily and the node never crashes, and never fails to build or launch, for
lack of it -- ``UnavailableResourceBackend`` reports UNAVAILABLE instead,
matching the project-wide "fail closed/report UNAVAILABLE, never fabricate"
convention used by every other node in this library.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


class ResourceBackendError(RuntimeError):
    """Raised only at construction time for an explicitly-requested backend
    that is not actually available (e.g. backend="psutil" but psutil is not
    installed)."""


@dataclass(frozen=True)
class ResourceSample:
    available: bool
    cpu_percent: float | None
    ram_percent: float | None
    reason: str


class ResourceBackend(Protocol):
    def sample(self) -> ResourceSample: ...


class UnavailableResourceBackend:
    """Dependency-free default. Never raises, never fabricates a number --
    every sample is explicitly unavailable with a reason."""

    def __init__(self, reason: str = "PSUTIL_NOT_INSTALLED"):
        self.reason = reason

    def sample(self) -> ResourceSample:
        return ResourceSample(available=False, cpu_percent=None, ram_percent=None, reason=self.reason)


class PsutilResourceBackend:
    """Thin wrapper around the real ``psutil`` library -- never reimplements
    CPU/RAM sampling. Imported lazily so this module (and the node that uses
    it) does not hard-require psutil at import time."""

    def __init__(self):
        try:
            import psutil
        except ImportError as exc:
            raise ResourceBackendError(f"PSUTIL_IMPORT_FAILED:{exc}") from exc
        self._psutil = psutil
        # Prime the non-blocking cpu_percent() call: psutil's documented
        # convention is that the FIRST call after import returns a
        # meaningless 0.0 (no time has elapsed to measure over) -- every
        # call after that compares against the previous call. Priming here
        # means the first real sample() from this backend is already
        # meaningful, not a guaranteed-wrong first reading.
        self._psutil.cpu_percent(interval=None)

    def sample(self) -> ResourceSample:
        try:
            cpu = float(self._psutil.cpu_percent(interval=None))
            ram = float(self._psutil.virtual_memory().percent)
        except Exception as exc:  # noqa: BLE001 - a sampling-time failure must
            # surface as UNAVAILABLE, not crash the node's periodic timer.
            return ResourceSample(available=False, cpu_percent=None, ram_percent=None,
                                   reason=f"PSUTIL_SAMPLE_FAILED:{type(exc).__name__}:{exc}")
        return ResourceSample(available=True, cpu_percent=cpu, ram_percent=ram, reason="OK")


def build_resource_backend(name: str = "psutil") -> ResourceBackend:
    """Never raises for the default ``name="psutil"`` case: if psutil is not
    importable, returns ``UnavailableResourceBackend`` rather than failing
    node construction, per this node's "reports, does not gate" contract --
    a monitoring node that refused to start because ITS OWN optional metrics
    library was missing would be a worse failure mode than just reporting
    that one metric as unavailable."""
    if name == "none":
        return UnavailableResourceBackend(reason="RESOURCE_BACKEND_DISABLED")
    if name == "psutil":
        try:
            return PsutilResourceBackend()
        except ResourceBackendError as exc:
            return UnavailableResourceBackend(reason=str(exc))
    raise ResourceBackendError(f"UNKNOWN_RESOURCE_BACKEND:{name}")
