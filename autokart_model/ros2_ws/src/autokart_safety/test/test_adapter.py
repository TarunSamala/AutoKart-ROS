"""Unit tests for the safety-core adapter. No rclpy/ROS runtime needed.

Exercises the lazy import against the REAL autokart_vcu_safety package
checked out at kart/ros2_ws/src/autokart_vcu_safety in this repository (not a
copy): these tests therefore also serve as a smoke test that the real core's
public API (SafetyLogic, Inputs, Config, VcuView, evaluate_startup) has not
drifted out from under this adapter.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from autokart_safety.adapter import SafetyCoreError, SyntheticInputsSource, load_safety_core  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[3]
REAL_VCU_SAFETY_SRC = REPO_ROOT / "kart" / "ros2_ws" / "src" / "autokart_vcu_safety"


def test_load_safety_core_empty_path_is_explicit_error():
    with pytest.raises(SafetyCoreError) as exc:
        load_safety_core("")
    assert "DEPENDENCY_PATH_NOT_SET" in str(exc.value)


def test_load_safety_core_missing_path_is_explicit_error(tmp_path):
    missing = tmp_path / "does_not_exist"
    with pytest.raises(SafetyCoreError) as exc:
        load_safety_core(str(missing))
    assert "DEPENDENCY_PATH_NOT_FOUND" in str(exc.value)


def test_load_safety_core_rejects_path_without_expected_api(tmp_path):
    # A directory that *does* contain an autokart_vcu_safety package (so sys.path
    # pollution from the real dependency elsewhere cannot mask this case) but
    # with a core module missing the expected API.
    src = tmp_path / "src"
    pkg = src / "autokart_vcu_safety"
    pkg.mkdir(parents=True)
    (pkg / "__init__.py").write_text("")
    (pkg / "core.py").write_text("SOMETHING_ELSE = 1\n")
    with pytest.raises(SafetyCoreError) as exc:
        load_safety_core(str(src))
    assert "SDK_ADAPTER_API_MISMATCH" in str(exc.value)


@pytest.mark.skipif(not REAL_VCU_SAFETY_SRC.is_dir(), reason="real kart/ros2_ws/src checkout not present")
def test_load_real_safety_core_exposes_expected_api():
    core = load_safety_core(str(REAL_VCU_SAFETY_SRC))
    assert hasattr(core, "SafetyLogic") and hasattr(core, "Inputs") and hasattr(core, "evaluate_startup")


@pytest.mark.skipif(not REAL_VCU_SAFETY_SRC.is_dir(), reason="real kart/ros2_ws/src checkout not present")
def test_real_core_defaults_to_disarmed_and_never_auto_arms():
    core = load_safety_core(str(REAL_VCU_SAFETY_SRC))
    logic = core.SafetyLogic()
    assert logic.state == "DISARMED"
    inputs = core.Inputs(ages={"vcu": None, "odom": None, "perception": None,
                                "mapping": None, "planner": None, "controller": None})
    out = logic.step(0.0, inputs, [])  # no requests at all -- pure tick
    assert out.state == "DISARMED" and out.arm_allowed is False


@pytest.mark.skipif(not REAL_VCU_SAFETY_SRC.is_dir(), reason="real kart/ros2_ws/src checkout not present")
def test_reimport_after_a_bad_path_loads_the_new_real_module(tmp_path):
    """Regression test for a real bug found while building this adapter: a
    prior call with a stub/misconfigured package_path must not leave a stale
    'autokart_vcu_safety' entry in sys.modules that masks a later, correct
    call with the real path. Reproduced pre-fix by calling the stub path
    first (poisoning sys.modules) and then the real path -- the pre-fix
    adapter returned the cached stub module (missing every required
    attribute) instead of the real core, even though sys.path had been
    updated correctly."""
    src = tmp_path / "src"
    pkg = src / "autokart_vcu_safety"
    pkg.mkdir(parents=True)
    (pkg / "__init__.py").write_text("")
    (pkg / "core.py").write_text("SOMETHING_ELSE = 1\n")
    with pytest.raises(SafetyCoreError):
        load_safety_core(str(src))

    core = load_safety_core(str(REAL_VCU_SAFETY_SRC))
    assert hasattr(core, "SafetyLogic"), (
        "got the stale stub module back instead of the real autokart_vcu_safety.core")


def test_synthetic_inputs_source_starts_with_nothing_fresh():
    source = SyntheticInputsSource()
    assert source.age("vcu", now=100.0) is None
    source.touch("vcu", now=100.0)
    assert source.age("vcu", now=100.3) == pytest.approx(0.3)
