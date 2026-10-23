"""Unit tests for the lazy algorithm-import wrapper. No rclpy/ROS runtime required."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from autokart_mission.algorithm import MissionAlgorithm, MissionAlgorithmError  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[3]
ALGORITHM_PATH = REPO_ROOT / "autonomy_stack_phase1" / "phase_01_perception" / "src"
VEHICLE_GEOMETRY_PATH = REPO_ROOT / "kart" / "ros2_ws" / "src" / "autokart_vehicle_geometry"
PLANNER_PATH = REPO_ROOT / "kart" / "ros2_ws" / "src" / "autokart_nodes" / "autokart_planner"
ALL_PRESENT = ALGORITHM_PATH.is_dir() and VEHICLE_GEOMETRY_PATH.is_dir() and PLANNER_PATH.is_dir()


def test_blank_paths_fail_closed_instead_of_resolving_to_cwd():
    # See autokart_mapping/algorithm.py's identical, documented fix: Path("").expanduser().is_dir()
    # is True (it resolves to "."), so a never-configured parameter must be rejected explicitly.
    with pytest.raises(MissionAlgorithmError) as exc:
        MissionAlgorithm("", "/some/path", "/some/path")
    assert "NOT_CONFIGURED" in str(exc.value)
    with pytest.raises(MissionAlgorithmError) as exc:
        MissionAlgorithm("/some/path", "   ", "/some/path")
    assert "NOT_CONFIGURED" in str(exc.value)
    with pytest.raises(MissionAlgorithmError) as exc:
        MissionAlgorithm("/some/path", "/some/path", "")
    assert "NOT_CONFIGURED" in str(exc.value)


def test_missing_dependency_path_is_explicit_error(tmp_path):
    missing = tmp_path / "does_not_exist"
    with pytest.raises(MissionAlgorithmError) as exc:
        MissionAlgorithm(str(missing), str(tmp_path), str(tmp_path))
    assert "DEPENDENCY_PATH_NOT_FOUND" in str(exc.value)
    assert "algorithm_package_path" in str(exc.value)


@pytest.mark.skipif(not ALL_PRESENT, reason="full kart/autonomy_stack_phase1 checkout not present")
def test_loads_real_mission_module_and_exposes_expected_symbols():
    algorithm = MissionAlgorithm(str(ALGORITHM_PATH), str(VEHICLE_GEOMETRY_PATH), str(PLANNER_PATH))
    assert algorithm.PlannerEngine is not None
    assert algorithm.SkidpadProgress is not None
    assert algorithm.PoseEstimate is not None
    assert algorithm.BrakeCapability is not None
    assert algorithm.GEOMETRY.bumper_width_m > 0


@pytest.mark.skipif(not ALL_PRESENT, reason="full kart/autonomy_stack_phase1 checkout not present")
def test_rejects_when_vehicle_geometry_yaml_cannot_be_found(tmp_path):
    # vehicle_geometry_package_path must be the REAL kart/ros2_ws/src/autokart_vehicle_geometry
    # directory (not a relocated copy): autokart_vehicle_geometry.geometry's own source-tree search
    # depends on autokart_bringup being its sibling under the same parent. Copying just the
    # autokart_vehicle_geometry folder elsewhere breaks that and must fail closed, not silently
    # import with a missing/guessed geometry.
    import shutil
    copy = tmp_path / "autokart_vehicle_geometry_copy"
    shutil.copytree(VEHICLE_GEOMETRY_PATH, copy)
    with pytest.raises(MissionAlgorithmError) as exc:
        MissionAlgorithm(str(ALGORITHM_PATH), str(copy), str(PLANNER_PATH))
    assert "autokart_vehicle_geometry" in str(exc.value)
