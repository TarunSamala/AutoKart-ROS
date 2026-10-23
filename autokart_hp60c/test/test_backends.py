"""Unit tests for camera backends. No rclpy/ROS runtime required."""
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from autokart_hp60c.backends import (  # noqa: E402
    SyntheticBackend, HP60CBackend, CameraBackendError, build_backend,
)


def test_synthetic_backend_produces_valid_frames():
    backend = SyntheticBackend(period_s=0.0)
    backend.start()
    frame = backend.read()
    assert frame.source == "SYNTHETIC"
    assert frame.rgb.shape == (480, 640, 3)
    assert frame.depth.shape == (480, 640)
    assert frame.intrinsics.valid()
    backend.close()


def test_synthetic_backend_sequence_increments():
    backend = SyntheticBackend(period_s=0.0)
    backend.start()
    f1 = backend.read()
    f2 = backend.read()
    assert f2.rgb_sequence == f1.rgb_sequence + 1
    backend.close()


def test_synthetic_backend_requires_start():
    backend = SyntheticBackend(period_s=0.0)
    with pytest.raises(CameraBackendError):
        backend.read()


def test_hp60c_backend_missing_dependency_path_is_explicit_error(tmp_path):
    missing = tmp_path / "does_not_exist"
    with pytest.raises(CameraBackendError) as exc:
        HP60CBackend(algorithm_package_path=str(missing), library=None, configuration=None)
    assert "DEPENDENCY_PATH_NOT_FOUND" in str(exc.value)


def test_hp60c_backend_rejects_path_without_autokart_perception(tmp_path):
    # A directory that *does* contain an autokart_perception package (so the
    # real dependency elsewhere on sys.path cannot mask this case) but with no
    # camera.py submodule, so the SDK adapter import must fail explicitly.
    src = tmp_path / "src"
    pkg = src / "autokart_perception"
    pkg.mkdir(parents=True)
    (pkg / "__init__.py").write_text("")
    with pytest.raises(CameraBackendError) as exc:
        HP60CBackend(algorithm_package_path=str(src), library=None, configuration=None)
    assert "SDK_ADAPTER_IMPORT_FAILED" in str(exc.value)


def test_build_backend_unknown_name_raises():
    with pytest.raises(CameraBackendError):
        build_backend("not-a-real-backend")


def test_build_backend_synthetic():
    backend = build_backend("synthetic", period_s=0.0)
    assert isinstance(backend, SyntheticBackend)
