"""Unit tests for the dashboard-gateway lazy-import adapter. No rclpy/aiohttp
runtime needed for most of these; the ones that import the real `gateway`
module exercise the REAL, already-tested dependency checked out at
kart/ros2_ws/src/autokart_dashboard_gateway (not a copy)."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from autokart_dashboard_gateway_node.adapter import (  # noqa: E402
    GatewayAdapterError, load_core, load_gateway,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
REAL_GATEWAY_SRC = REPO_ROOT / "kart" / "ros2_ws" / "src" / "autokart_dashboard_gateway"
REAL_PERCEPTION_SRC = REPO_ROOT / "autonomy_stack_phase1" / "phase_01_perception" / "src"


def test_load_gateway_empty_path_is_explicit_error():
    with pytest.raises(GatewayAdapterError) as exc:
        load_gateway("", "")
    assert "DEPENDENCY_PATH_NOT_SET" in str(exc.value)


def test_load_gateway_missing_path_is_explicit_error(tmp_path):
    missing = tmp_path / "does_not_exist"
    with pytest.raises(GatewayAdapterError) as exc:
        load_gateway(str(missing), str(tmp_path))
    assert "DEPENDENCY_PATH_NOT_FOUND" in str(exc.value)


@pytest.mark.skipif(not REAL_GATEWAY_SRC.is_dir(), reason="real gateway checkout not present")
def test_load_gateway_requires_perception_path_even_without_camera():
    """Real finding from building this adapter: gateway.py unconditionally
    imports camera_feed.py, which unconditionally imports
    autokart_perception.live_render -- so the perception dependency is
    required to import the gateway module at all, regardless of
    enable_camera_feed. Omitting perception_package_path must fail
    explicitly, not silently skip camera support."""
    with pytest.raises(GatewayAdapterError) as exc:
        load_gateway(str(REAL_GATEWAY_SRC), "")
    assert "PERCEPTION_DEPENDENCY_PATH_NOT_SET" in str(exc.value)


@pytest.mark.skipif(not REAL_GATEWAY_SRC.is_dir(), reason="real gateway checkout not present")
def test_load_gateway_wrong_perception_path_is_rejected_not_silently_substituted(tmp_path):
    """This machine has autokart_perception permanently importable via a
    site-wide editable install, independent of any path given here. Without
    the adapter's explicit post-import path check, a wrong
    perception_package_path would silently succeed by falling through to
    that global copy instead of failing -- exactly the kind of
    silently-wrong-dependency bug this adapter must not have."""
    with pytest.raises(GatewayAdapterError) as exc:
        load_gateway(str(REAL_GATEWAY_SRC), str(tmp_path))
    assert "PERCEPTION_DEPENDENCY_PATH_MISMATCH" in str(exc.value)


@pytest.mark.skipif(not (REAL_GATEWAY_SRC.is_dir() and REAL_PERCEPTION_SRC.is_dir()),
                     reason="real gateway + perception checkouts not present")
def test_load_real_gateway_exposes_expected_api():
    gateway = load_gateway(str(REAL_GATEWAY_SRC), str(REAL_PERCEPTION_SRC))
    assert hasattr(gateway, "make_ros_node")
    assert hasattr(gateway, "create_app")
    assert hasattr(gateway, "ALLOWED_SAFETY_REQUESTS")
    assert "set_throttle_v" not in gateway.ALLOWED_SAFETY_REQUESTS
    assert gateway.ALLOWED_SAFETY_REQUESTS == {
        "arm", "stop", "reset_fault", "confirm_manual_power_cut",
        "demo_drive_start", "demo_drive_stop",
    }


@pytest.mark.skipif(not REAL_GATEWAY_SRC.is_dir(), reason="real gateway checkout not present")
def test_load_core_has_no_perception_dependency():
    """core.py (MissionStore/valid_frame) has no camera_feed import at all --
    confirms this path genuinely needs no perception dependency, unlike
    load_gateway above."""
    core = load_core(str(REAL_GATEWAY_SRC))
    assert hasattr(core, "MissionStore") and hasattr(core, "valid_frame")
    store = core.MissionStore("SIMULATION", "test-run")
    frame = store.capture(0.0, {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "speed_mps": 0.0})
    assert core.valid_frame(frame)


def test_load_core_rejects_api_mismatch(tmp_path):
    pkg = tmp_path / "autokart_dashboard_gateway"
    pkg.mkdir(parents=True)
    (pkg / "__init__.py").write_text("")
    (pkg / "core.py").write_text("SOMETHING_ELSE = 1\n")
    with pytest.raises(GatewayAdapterError) as exc:
        load_core(str(tmp_path))
    assert "GATEWAY_API_MISMATCH" in str(exc.value)
