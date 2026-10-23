"""Unit tests for the lazy algorithm-import wrapper. No rclpy/ROS runtime required."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from autokart_mapping.algorithm import MappingAlgorithm, MappingAlgorithmError  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[3]
REAL_ALGORITHM_PATH = REPO_ROOT / "autonomy_stack_phase1" / "phase_01_perception" / "src"


def test_blank_path_fails_closed_instead_of_resolving_to_cwd():
    # Path("").expanduser().is_dir() is True (it resolves to "."), so without an explicit guard a
    # never-configured algorithm_package_path (the parameter's own documented default) would be
    # silently accepted as "." rather than rejected -- and would even happen to "work" on a host
    # where the real dependency is independently importable some other way (e.g. a global editable
    # pip install), masking the fact that the parameter was never actually set. This must fail
    # closed regardless of what else is importable on the host.
    with pytest.raises(MappingAlgorithmError) as exc:
        MappingAlgorithm("")
    assert "NOT_CONFIGURED" in str(exc.value)
    with pytest.raises(MappingAlgorithmError):
        MappingAlgorithm("   ")


def test_missing_dependency_path_is_explicit_error(tmp_path):
    missing = tmp_path / "does_not_exist"
    with pytest.raises(MappingAlgorithmError) as exc:
        MappingAlgorithm(str(missing))
    assert "DEPENDENCY_PATH_NOT_FOUND" in str(exc.value)


def test_rejects_path_without_autokart_perception(tmp_path):
    # A directory that *does* contain an autokart_perception package (so the real dependency
    # elsewhere on sys.path cannot mask this case) but with no application.py submodule, so the
    # import must fail explicitly rather than silently falling back to a different copy.
    src = tmp_path / "src"
    pkg = src / "autokart_perception"
    pkg.mkdir(parents=True)
    (pkg / "__init__.py").write_text("")
    with pytest.raises(MappingAlgorithmError) as exc:
        MappingAlgorithm(str(src))
    assert "ALGORITHM_IMPORT_FAILED" in str(exc.value)


@pytest.mark.skipif(not REAL_ALGORITHM_PATH.is_dir(), reason="autonomy_stack_phase1 checkout not present")
def test_loads_real_algorithm_package_and_exposes_expected_symbols():
    algorithm = MappingAlgorithm(str(REAL_ALGORITHM_PATH))
    assert algorithm.CoreRuntime is not None
    assert algorithm.PoseEstimate is not None
    assert callable(algorithm.fit_skidpad_topology)
    runtime = algorithm.new_core_runtime()
    assert hasattr(runtime, "algorithms")
