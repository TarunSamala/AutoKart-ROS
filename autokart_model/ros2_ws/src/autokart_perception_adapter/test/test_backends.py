"""Unit tests for perception backends. No rclpy/ROS runtime required."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from autokart_perception_adapter.backends import (  # noqa: E402
    SyntheticConeSource, AlgorithmBackend, PerceptionBackendError, build_backend,
)


@pytest.fixture(autouse=True)
def _isolate_autokart_perception_import_state():
    """Each AlgorithmBackend test points algorithm_package_path at a DIFFERENT
    directory. Two separate caches make that leak across tests if not reset:

    1. sys.modules caches 'autokart_perception' (and its submodules) by name
       after the first import -- a later test reusing the name would import
       the PREVIOUS test's module object instead of re-resolving.
    2. backends.AlgorithmBackend.__init__ permanently inserts its given path
       at sys.path[0] and never removes it (harmless in a real node process,
       which constructs one backend once, but across tests this means an
       EARLIER test's fake/empty tmp-dir package can still shadow a LATER
       test's real path, because it sits earlier in sys.path and Python
       resolves packages by sys.path order regardless of the sys.modules
       cache state).

    This was caught by running the real-import test (below) in the full file
    vs. in isolation: it passed alone and failed in the full run, both before
    this fixture existed.
    """
    saved_path = list(sys.path)
    stale = [name for name in sys.modules if name == "autokart_perception"
             or name.startswith("autokart_perception.")]
    for name in stale:
        del sys.modules[name]
    yield
    sys.path[:] = saved_path
    stale = [name for name in sys.modules if name == "autokart_perception"
             or name.startswith("autokart_perception.")]
    for name in stale:
        del sys.modules[name]


def test_synthetic_source_returns_fixed_qualified_layout():
    source = SyntheticConeSource()
    outcome = source.process(None, None, 500.0, 500.0, 320.0, 240.0, 0, "SYNTHETIC")
    assert outcome.valid is True
    assert outcome.source == "SYNTHETIC"
    assert len(outcome.cones) == 4
    labels = {c.label for c in outcome.cones}
    assert labels == {"blue", "yellow"}


def test_synthetic_source_is_deterministic():
    source = SyntheticConeSource()
    a = source.process(None, None, 1, 1, 1, 1, 0, "SYNTHETIC")
    b = source.process(None, None, 1, 1, 1, 1, 0, "SYNTHETIC")
    assert a.cones == b.cones


def test_algorithm_backend_missing_dependency_path_is_explicit_error(tmp_path):
    missing = tmp_path / "does_not_exist"
    with pytest.raises(PerceptionBackendError) as exc:
        AlgorithmBackend(algorithm_package_path=str(missing), model_path=None,
                          model_manifest=None, calibration_artifact=None)
    assert "DEPENDENCY_PATH_NOT_FOUND" in str(exc.value)


def test_algorithm_backend_rejects_path_without_autokart_perception(tmp_path):
    src = tmp_path / "src"
    pkg = src / "autokart_perception"
    pkg.mkdir(parents=True)
    (pkg / "__init__.py").write_text("")
    with pytest.raises(PerceptionBackendError) as exc:
        AlgorithmBackend(algorithm_package_path=str(src), model_path=None,
                          model_manifest=None, calibration_artifact=None)
    assert "SDK_ADAPTER_IMPORT_FAILED" in str(exc.value)


def test_build_backend_unknown_name_raises():
    with pytest.raises(PerceptionBackendError):
        build_backend("not-a-real-backend")


def test_build_backend_synthetic():
    assert isinstance(build_backend("synthetic"), SyntheticConeSource)


REAL_ALGO_PATH = __import__("os").environ.get("AUTOKART_PERCEPTION_SOURCE", str(Path(__file__).resolve().parents[3]/"autonomy_stack_phase1/phase_01_perception/src"))


@pytest.mark.skipif(not Path(REAL_ALGO_PATH).is_dir(), reason="algorithm package checkout not present")
def test_algorithm_backend_real_import_runs_synthetic_geometry():
    """Exercises the REAL autokart_perception package (no detector model
    given, so Pipeline runs in its own synthetic-geometry mode) to prove the
    lazy-import adapter actually works end to end against real code, not just
    its failure paths."""
    import numpy as np
    backend = AlgorithmBackend(algorithm_package_path=REAL_ALGO_PATH, model_path=None,
                                model_manifest=None, calibration_artifact=None)
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    depth = np.full((480, 640), 2000, dtype=np.uint16)
    outcome = backend.process(rgb, depth, 500.0, 500.0, 320.0, 240.0, 0, "SYNTHETIC")
    assert outcome.source == "SYNTHETIC"
    # Pipeline's own synthetic_cones() fixture; qualification depends on its
    # internal safety/topology checks, which this test does not second-guess.
    assert isinstance(outcome.cones, tuple)
