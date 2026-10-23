"""Unit tests for the CPU/RAM resource backends. No ROS needed."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from autokart_system_monitor.resource_backends import (  # noqa: E402
    PsutilResourceBackend, ResourceBackendError, UnavailableResourceBackend,
    build_resource_backend,
)


def test_unavailable_backend_never_raises_and_reports_reason():
    backend = UnavailableResourceBackend(reason="TEST_REASON")
    sample = backend.sample()
    assert sample.available is False
    assert sample.cpu_percent is None and sample.ram_percent is None
    assert sample.reason == "TEST_REASON"


def test_build_resource_backend_none_is_always_unavailable():
    backend = build_resource_backend("none")
    assert isinstance(backend, UnavailableResourceBackend)
    assert backend.sample().available is False


def test_build_resource_backend_unknown_name_raises():
    with pytest.raises(ResourceBackendError):
        build_resource_backend("not-a-real-backend")


def test_build_resource_backend_psutil_falls_back_gracefully_if_missing(monkeypatch):
    """Simulates psutil not being installed: must return an unavailable
    backend, not raise and not crash the caller."""
    import builtins
    real_import = builtins.__import__

    def fake_import(name, *args, **kwargs):
        if name == "psutil":
            raise ImportError("simulated: psutil not installed")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fake_import)
    backend = build_resource_backend("psutil")
    assert isinstance(backend, UnavailableResourceBackend)
    sample = backend.sample()
    assert sample.available is False
    assert "PSUTIL_IMPORT_FAILED" in sample.reason


def test_psutil_backend_if_actually_installed_returns_plausible_values():
    pytest.importorskip("psutil")
    backend = PsutilResourceBackend()
    sample = backend.sample()
    assert sample.available is True
    assert sample.cpu_percent is not None and 0.0 <= sample.cpu_percent <= 100.0
    assert sample.ram_percent is not None and 0.0 <= sample.ram_percent <= 100.0
