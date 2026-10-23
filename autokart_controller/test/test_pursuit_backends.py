"""Unit tests for the Pure Pursuit + PID backends. No rclpy/ROS runtime needed for
BuiltinPursuitBackend; the ReusedPursuitBackend cross-validation tests are skipped (not failed)
when kart_sim isn't present at the expected dev-environment path."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from autokart_controller.pursuit_backends import (  # noqa: E402
    BuiltinPursuitBackend, ReusedPursuitBackend, PursuitBackendError, build_pursuit_backend,
)

KART_SIM_PACKAGE_PATH = "/home/mark1/Desktop/autonomous_kart/kart"


def _dependency_available() -> bool:
    return Path(KART_SIM_PACKAGE_PATH, "kart_sim", "autonomy.py").is_file()


requires_dependency = pytest.mark.skipif(
    not _dependency_available(), reason="kart_sim package not present in this environment")


@pytest.fixture(autouse=True)
def _clean_kart_sim_imports():
    import sys as _sys
    path_before = list(_sys.path)
    yield
    for name in list(_sys.modules):
        if name == "kart_sim" or name.startswith("kart_sim."):
            del _sys.modules[name]
    _sys.path[:] = path_before


def straight_path(n=11, step=1.0, speed=1.0):
    return [(float(i) * step, 0.0, 0.0, speed) for i in range(n)]


# ---------------------------------------------------------------------------
# BuiltinPursuitBackend
# ---------------------------------------------------------------------------

def test_builtin_zero_cross_track_error_on_path_produces_near_zero_steering():
    backend = BuiltinPursuitBackend(wheelbase_m=1.0)
    out = backend.compute(pose_x=0.0, pose_y=0.0, pose_yaw_rad=0.0, pose_speed_mps=1.0,
                          path_points_with_heading_speed=straight_path(), dt_s=0.1,
                          safety_speed_limit_mps=2.0)
    assert abs(out["steering_rad"]) < 1e-6
    assert out["cross_track_error_m"] == pytest.approx(0.0)
    assert out["speed_mps"] == pytest.approx(1.0)


def test_builtin_offset_pose_produces_nonzero_steering_toward_the_path():
    backend = BuiltinPursuitBackend(wheelbase_m=1.0, lookahead_m=1.5)
    out = backend.compute(pose_x=0.0, pose_y=-0.5, pose_yaw_rad=0.0, pose_speed_mps=1.0,
                          path_points_with_heading_speed=straight_path(), dt_s=0.1,
                          safety_speed_limit_mps=2.0)
    # Path is to the left (+y) of the vehicle: steering should turn left (positive).
    assert out["steering_rad"] > 0.0
    assert out["cross_track_error_m"] == pytest.approx(0.5, abs=1e-6)


def test_builtin_steering_is_clamped_to_the_configured_mechanical_limit():
    backend = BuiltinPursuitBackend(wheelbase_m=1.0, maximum_steering_rad=0.05,
                                    maximum_steering_rate_radps=100.0)
    out = backend.compute(pose_x=0.0, pose_y=-5.0, pose_yaw_rad=0.0, pose_speed_mps=0.0,
                          path_points_with_heading_speed=straight_path(), dt_s=0.1,
                          safety_speed_limit_mps=2.0)
    assert out["steering_rad"] == pytest.approx(0.05, abs=1e-9)


def test_builtin_steering_rate_is_limited_across_calls():
    backend = BuiltinPursuitBackend(wheelbase_m=1.0, maximum_steering_rad=1.0,
                                    maximum_steering_rate_radps=0.5)
    # First call establishes a baseline near zero steering (straight ahead, on-path).
    backend.compute(pose_x=0.0, pose_y=0.0, pose_yaw_rad=0.0, pose_speed_mps=0.0,
                    path_points_with_heading_speed=straight_path(), dt_s=0.1, safety_speed_limit_mps=2.0)
    # Second call requests a hard turn (big offset); the rate limit (0.5 rad/s * 0.1 s = 0.05 rad)
    # must cap how far steering can move in one step.
    out = backend.compute(pose_x=0.0, pose_y=-5.0, pose_yaw_rad=0.0, pose_speed_mps=0.0,
                          path_points_with_heading_speed=straight_path(), dt_s=0.1,
                          safety_speed_limit_mps=2.0)
    assert abs(out["steering_rad"]) <= 0.05 + 1e-9


def test_builtin_rejects_nonpositive_wheelbase():
    with pytest.raises(PursuitBackendError) as exc:
        BuiltinPursuitBackend(wheelbase_m=0.0)
    assert "WHEELBASE_UNQUALIFIED" in str(exc.value)


def test_builtin_rejects_nonpositive_steering_limit():
    with pytest.raises(PursuitBackendError) as exc:
        BuiltinPursuitBackend(wheelbase_m=1.0, maximum_steering_rad=0.0)
    assert "STEERING_LIMIT_UNQUALIFIED" in str(exc.value)


def test_builtin_rejects_nonpositive_dt():
    backend = BuiltinPursuitBackend(wheelbase_m=1.0)
    with pytest.raises(PursuitBackendError):
        backend.compute(pose_x=0.0, pose_y=0.0, pose_yaw_rad=0.0, pose_speed_mps=0.0,
                        path_points_with_heading_speed=straight_path(), dt_s=0.0,
                        safety_speed_limit_mps=2.0)


def test_builtin_speed_is_capped_by_safety_speed_limit():
    backend = BuiltinPursuitBackend(wheelbase_m=1.0)
    out = backend.compute(pose_x=0.0, pose_y=0.0, pose_yaw_rad=0.0, pose_speed_mps=0.0,
                          path_points_with_heading_speed=straight_path(speed=10.0), dt_s=0.1,
                          safety_speed_limit_mps=1.0)
    assert out["speed_mps"] <= 1.0


def test_builtin_brake_stuck_zeroes_drive():
    backend = BuiltinPursuitBackend(wheelbase_m=1.0)
    out = backend.compute(pose_x=0.0, pose_y=0.0, pose_yaw_rad=0.0, pose_speed_mps=0.0,
                          path_points_with_heading_speed=straight_path(speed=1.0), dt_s=0.1,
                          safety_speed_limit_mps=2.0, brake_stuck=True)
    assert out["drive"] == 0.0


def test_build_pursuit_backend_unknown_name_raises():
    with pytest.raises(PursuitBackendError):
        build_pursuit_backend("not-a-real-backend", wheelbase_m=1.0)


def test_build_pursuit_backend_builtin():
    backend = build_pursuit_backend("builtin", wheelbase_m=1.0)
    assert isinstance(backend, BuiltinPursuitBackend)


# ---------------------------------------------------------------------------
# ReusedPursuitBackend: dependency-path failure paths (always run, no dependency needed)
# ---------------------------------------------------------------------------

def test_reused_backend_missing_dependency_path_is_explicit_error(tmp_path):
    missing = tmp_path / "does_not_exist"
    with pytest.raises(PursuitBackendError) as exc:
        ReusedPursuitBackend(kart_sim_package_path=str(missing), wheelbase_m=1.0, lookahead_m=1.5,
                             speed_lookahead_gain_s=0.15, maximum_steering_rad=0.6,
                             maximum_steering_rate_radps=3.0)
    assert "DEPENDENCY_PATH_NOT_FOUND" in str(exc.value)


def test_reused_backend_rejects_path_without_kart_sim(tmp_path):
    src = tmp_path / "src"
    pkg = src / "kart_sim"
    pkg.mkdir(parents=True)
    (pkg / "__init__.py").write_text("")
    with pytest.raises(PursuitBackendError) as exc:
        ReusedPursuitBackend(kart_sim_package_path=str(src), wheelbase_m=1.0, lookahead_m=1.5,
                             speed_lookahead_gain_s=0.15, maximum_steering_rad=0.6,
                             maximum_steering_rate_radps=3.0)
    # Which of the two fires depends on the environment: on a clean machine the fake package wins
    # and autonomy.py is missing (IMPORT_FAILED); where an editable/meta-path install of the real
    # kart_sim exists (this dev machine), the real one is imported from elsewhere and the
    # path-provenance check must catch it (PATH_MISMATCH). Either way: never a silent success.
    assert ("SDK_ADAPTER_IMPORT_FAILED" in str(exc.value)
            or "DEPENDENCY_PATH_MISMATCH" in str(exc.value))


def test_reused_backend_rejects_module_resolved_outside_configured_path(tmp_path, monkeypatch):
    # Deterministic regardless of environment: pre-seed sys.modules with fake kart_sim modules
    # whose __file__ lies outside the configured path, simulating a meta-path finder / cached
    # import resolving the name elsewhere. Before the provenance check this silently "succeeded".
    import types as _types
    configured = tmp_path / "configured"
    configured.mkdir()
    elsewhere = tmp_path / "elsewhere" / "kart_sim"
    elsewhere.mkdir(parents=True)
    pkg = _types.ModuleType("kart_sim")
    pkg.__path__ = [str(elsewhere)]
    pkg.__file__ = str(elsewhere / "__init__.py")
    autonomy = _types.ModuleType("kart_sim.autonomy")
    autonomy.__file__ = str(elsewhere / "autonomy.py")
    types_mod = _types.ModuleType("kart_sim.types")
    types_mod.__file__ = str(elsewhere / "types.py")
    monkeypatch.setitem(sys.modules, "kart_sim", pkg)
    monkeypatch.setitem(sys.modules, "kart_sim.autonomy", autonomy)
    monkeypatch.setitem(sys.modules, "kart_sim.types", types_mod)
    with pytest.raises(PursuitBackendError) as exc:
        ReusedPursuitBackend(kart_sim_package_path=str(configured), wheelbase_m=1.0, lookahead_m=1.5,
                             speed_lookahead_gain_s=0.15, maximum_steering_rad=0.6,
                             maximum_steering_rate_radps=3.0)
    assert "DEPENDENCY_PATH_MISMATCH" in str(exc.value)


# ---------------------------------------------------------------------------
# Cross-validation: Builtin vs Reused, proving the zero-dependency default agrees with the real,
# already-tested kart_sim implementation (skipped if the dependency isn't present).
# ---------------------------------------------------------------------------

@requires_dependency
def test_reused_backend_builds_against_real_dependency():
    backend = build_pursuit_backend("reused", wheelbase_m=1.0, kart_sim_package_path=KART_SIM_PACKAGE_PATH)
    assert isinstance(backend, ReusedPursuitBackend)


@requires_dependency
@pytest.mark.parametrize("pose_y", [0.0, -0.5, 0.8, -2.0])
def test_builtin_and_reused_steering_agree_across_offsets(pose_y):
    builtin = build_pursuit_backend("builtin", wheelbase_m=1.0, lookahead_m=1.5,
                                    maximum_steering_rad=0.6, maximum_steering_rate_radps=3.0)
    reused = build_pursuit_backend("reused", wheelbase_m=1.0, lookahead_m=1.5,
                                   maximum_steering_rad=0.6, maximum_steering_rate_radps=3.0,
                                   kart_sim_package_path=KART_SIM_PACKAGE_PATH)
    path = straight_path(speed=1.0)
    b = builtin.compute(pose_x=0.0, pose_y=pose_y, pose_yaw_rad=0.0, pose_speed_mps=1.0,
                        path_points_with_heading_speed=path, dt_s=0.1, safety_speed_limit_mps=2.0)
    r = reused.compute(pose_x=0.0, pose_y=pose_y, pose_yaw_rad=0.0, pose_speed_mps=1.0,
                       path_points_with_heading_speed=path, dt_s=0.1, safety_speed_limit_mps=2.0)
    assert b["steering_rad"] == pytest.approx(r["steering_rad"], abs=1e-6)
    assert b["cross_track_error_m"] == pytest.approx(r["cross_track_error_m"], abs=1e-6)
    assert b["heading_error_rad"] == pytest.approx(r["heading_error_rad"], abs=1e-6)
    assert b["speed_mps"] == pytest.approx(r["speed_mps"], abs=1e-6)


@requires_dependency
def test_builtin_and_reused_agree_over_a_multi_step_sequence():
    # Exercises the stateful parts (steering rate limiter, PID integral) across several ticks,
    # not just a single-shot comparison.
    builtin = build_pursuit_backend("builtin", wheelbase_m=1.0, maximum_steering_rate_radps=1.0)
    reused = build_pursuit_backend("reused", wheelbase_m=1.0, maximum_steering_rate_radps=1.0,
                                   kart_sim_package_path=KART_SIM_PACKAGE_PATH)
    path = straight_path(speed=2.0)
    for step, pose_y in enumerate((-3.0, -2.0, -1.0, -0.2, 0.0, 0.0)):
        b = builtin.compute(pose_x=0.0, pose_y=pose_y, pose_yaw_rad=0.0, pose_speed_mps=0.5 + step * 0.1,
                            path_points_with_heading_speed=path, dt_s=0.05, safety_speed_limit_mps=2.0)
        r = reused.compute(pose_x=0.0, pose_y=pose_y, pose_yaw_rad=0.0, pose_speed_mps=0.5 + step * 0.1,
                           path_points_with_heading_speed=path, dt_s=0.05, safety_speed_limit_mps=2.0)
        assert b["steering_rad"] == pytest.approx(r["steering_rad"], abs=1e-6), f"step {step}"
        assert b["speed_mps"] == pytest.approx(r["speed_mps"], abs=1e-6), f"step {step}"
        assert b["brake"] == pytest.approx(r["brake"], abs=1e-6), f"step {step}"
