"""Unit tests for the curvature/stopping-distance algorithm backends. No rclpy/ROS runtime
needed for BuiltinAlgorithmBackend; the ReusedAlgorithmBackend cross-validation tests are
skipped (not failed) when the autokart_perception algorithm package isn't present at the
expected dev-environment path, so this file still runs green on a plain Humble install."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from autokart_planner.algorithms import (  # noqa: E402
    BuiltinAlgorithmBackend, ReusedAlgorithmBackend, AlgorithmBackendError, build_algorithm_backend,
)

# Matches this dev environment's autonomy_stack_phase1 checkout (see DEPENDENCIES.md). Tests that
# need it are skipped, not failed, when it's absent -- this package's default build/test path
# never requires it.
ALGORITHM_PACKAGE_PATH = __import__("os").environ.get("AUTOKART_PERCEPTION_SOURCE", str(Path(__file__).resolve().parents[3]/"autonomy_stack_phase1/phase_01_perception/src"))


def _dependency_available() -> bool:
    return Path(ALGORITHM_PACKAGE_PATH, "autokart_perception", "curvature.py").is_file()


requires_dependency = pytest.mark.skipif(
    not _dependency_available(), reason="autokart_perception algorithm package not present in this environment")


@pytest.fixture(autouse=True)
def _clean_autokart_perception_imports():
    """Several tests in this file deliberately import a FAKE `autokart_perception` package (to
    test the SDK_ADAPTER_IMPORT_FAILED path) and others import the REAL one from a different
    sys.path entry (the cross-validation tests). Python caches the top-level package object in
    sys.modules by name, so without this cleanup, whichever one imports first "wins" for the rest
    of the test process regardless of sys.path order -- a real cross-test import-cache hazard,
    not a hypothetical one (this was caught by a failing test run during development, see
    reports/SENIOR_REVIEWER_REPORT.md)."""
    import sys
    path_before = list(sys.path)
    yield
    for name in list(sys.modules):
        if name == "autokart_perception" or name.startswith("autokart_perception."):
            del sys.modules[name]
    sys.path[:] = path_before


def straight_line(n=5, step=1.0):
    return [(float(i) * step, 0.0) for i in range(n)]


def circle_arc(radius, n=9, span_deg=40.0):
    # x monotonically increasing (a "centerline" convention profile_centerline's own
    # CURVATURE_AMBIGUOUS guard enforces, see autokart_perception/curvature.py), tracing a circle
    # of radius `radius` starting at the origin heading in +x, curving toward +y.
    points = []
    for i in range(n):
        angle = math.radians(span_deg) * i / (n - 1)
        points.append((radius * math.sin(angle), radius * (1.0 - math.cos(angle))))
    return points


# ---------------------------------------------------------------------------
# BuiltinAlgorithmBackend
# ---------------------------------------------------------------------------

def test_builtin_straight_line_has_zero_curvature_everywhere():
    backend = BuiltinAlgorithmBackend()
    kappas = backend.curvature_profile(straight_line())
    assert all(abs(k) < 1e-9 for k in kappas)


def test_builtin_circle_arc_curvature_matches_one_over_radius():
    radius = 5.0
    backend = BuiltinAlgorithmBackend()
    points = circle_arc(radius, n=11)
    kappas = backend.curvature_profile(points)
    # Interior points only: endpoints intentionally reuse a neighbor (documented convention).
    for k in kappas[2:-2]:
        assert math.isclose(abs(k), 1.0 / radius, rel_tol=0.02)


def test_builtin_endpoints_reuse_nearest_interior_curvature_not_zero():
    radius = 3.0
    backend = BuiltinAlgorithmBackend()
    points = circle_arc(radius, n=9)
    kappas = backend.curvature_profile(points)
    assert kappas[0] == kappas[1]
    assert kappas[-1] == kappas[-2]
    assert abs(kappas[0]) > 1e-3  # not silently flattened to 0 at the path's end


def test_builtin_two_points_is_exactly_straight():
    backend = BuiltinAlgorithmBackend()
    assert backend.curvature_profile([(0.0, 0.0), (1.0, 0.0)]) == [0.0, 0.0]


def test_builtin_rejects_too_few_points():
    backend = BuiltinAlgorithmBackend()
    with pytest.raises(AlgorithmBackendError):
        backend.curvature_profile([(0.0, 0.0)])


def test_builtin_rejects_nonfinite_points():
    backend = BuiltinAlgorithmBackend()
    with pytest.raises(AlgorithmBackendError):
        backend.curvature_profile([(0.0, 0.0), (float("nan"), 1.0), (2.0, 2.0)])


def test_builtin_rejects_duplicate_points_too_close():
    backend = BuiltinAlgorithmBackend()
    with pytest.raises(AlgorithmBackendError):
        backend.curvature_profile([(0.0, 0.0), (0.0, 0.0), (1.0, 0.0)])


def test_builtin_stopping_distance_matches_formula():
    backend = BuiltinAlgorithmBackend()
    v, t, a, m = 3.0, 0.2, 2.0, 1.0
    expected = v * t + v ** 2 / (2 * a) + m
    assert backend.stopping_distance(v, a_brake_min_mps2=a, t_total_latency_s=t, d_margin_m=m) == \
        pytest.approx(expected)


def test_builtin_stopping_distance_rejects_unqualified_brake_capability():
    backend = BuiltinAlgorithmBackend()
    with pytest.raises(AlgorithmBackendError) as exc:
        backend.stopping_distance(1.0, a_brake_min_mps2=0.0, t_total_latency_s=0.1, d_margin_m=1.0)
    assert "BRAKE_CAPABILITY_UNQUALIFIED" in str(exc.value)


def test_builtin_stopping_distance_rejects_negative_speed():
    backend = BuiltinAlgorithmBackend()
    with pytest.raises(AlgorithmBackendError):
        backend.stopping_distance(-1.0, a_brake_min_mps2=2.0, t_total_latency_s=0.1, d_margin_m=1.0)


def test_build_algorithm_backend_unknown_name_raises():
    with pytest.raises(AlgorithmBackendError):
        build_algorithm_backend("not-a-real-backend")


def test_build_algorithm_backend_builtin():
    backend = build_algorithm_backend("builtin")
    assert isinstance(backend, BuiltinAlgorithmBackend)


# ---------------------------------------------------------------------------
# ReusedAlgorithmBackend: dependency-path failure paths (always run, no dependency needed)
# ---------------------------------------------------------------------------

def test_reused_backend_missing_dependency_path_is_explicit_error(tmp_path):
    missing = tmp_path / "does_not_exist"
    with pytest.raises(AlgorithmBackendError) as exc:
        ReusedAlgorithmBackend(algorithm_package_path=str(missing))
    assert "DEPENDENCY_PATH_NOT_FOUND" in str(exc.value)


def test_reused_backend_rejects_path_without_autokart_perception(tmp_path):
    src = tmp_path / "src"
    pkg = src / "autokart_perception"
    pkg.mkdir(parents=True)
    (pkg / "__init__.py").write_text("")
    with pytest.raises(AlgorithmBackendError) as exc:
        ReusedAlgorithmBackend(algorithm_package_path=str(src))
    assert "SDK_ADAPTER_IMPORT_FAILED" in str(exc.value)


# ---------------------------------------------------------------------------
# Cross-validation: Builtin vs Reused, proving the zero-dependency default agrees with the
# real, already-tested algorithm it stands in for (skipped if the dependency isn't present).
# ---------------------------------------------------------------------------

@requires_dependency
def test_reused_backend_builds_against_real_dependency():
    backend = build_algorithm_backend("reused", algorithm_package_path=ALGORITHM_PACKAGE_PATH)
    assert isinstance(backend, ReusedAlgorithmBackend)


@requires_dependency
def test_builtin_and_reused_curvature_agree_on_a_circular_arc():
    builtin = BuiltinAlgorithmBackend()
    reused = build_algorithm_backend("reused", algorithm_package_path=ALGORITHM_PACKAGE_PATH)
    radius = 8.0
    points = circle_arc(radius, n=15, span_deg=60.0)
    builtin_kappas = builtin.curvature_profile(points)
    reused_kappas = reused.curvature_profile(points)
    assert len(builtin_kappas) == len(reused_kappas)
    for b, r in zip(builtin_kappas[2:-2], reused_kappas[2:-2]):
        assert math.isclose(b, r, rel_tol=0.1), f"builtin={b} reused={r} disagree beyond tolerance"


@requires_dependency
def test_builtin_and_reused_stopping_distance_agree():
    builtin = BuiltinAlgorithmBackend()
    reused = build_algorithm_backend("reused", algorithm_package_path=ALGORITHM_PACKAGE_PATH)
    for v in (0.0, 0.5, 1.0, 2.5, 5.0):
        b = builtin.stopping_distance(v, a_brake_min_mps2=2.0, t_total_latency_s=0.2, d_margin_m=1.0)
        r = reused.stopping_distance(v, a_brake_min_mps2=2.0, t_total_latency_s=0.2, d_margin_m=1.0)
        assert b == pytest.approx(r, abs=1e-9), f"v={v}: builtin={b} reused={r}"
