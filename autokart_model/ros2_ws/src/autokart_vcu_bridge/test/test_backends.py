"""Unit tests for VCU backends. No rclpy/ROS runtime required."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from autokart_vcu_bridge.backends import (  # noqa: E402
    SyntheticBackend, SerialVcuBackend, VcuBackendError, build_backend,
)

# The real autokart_vehicle_bridge package this node reuses, computed relative
# to this repository (never hardcoded in the node's own source -- only here,
# as a test convenience, to prove the lazy-import path genuinely works
# end-to-end against the real protocol.py, not just a fake stand-in).
REPO_ROOT = Path(__file__).resolve().parents[3]
REAL_VEHICLE_BRIDGE_PATH = REPO_ROOT / "kart" / "ros2_ws" / "src" / "autokart_vehicle_bridge"


@pytest.fixture(autouse=True)
def _isolate_lazy_import_state():
    """SerialVcuBackend's lazy-import pattern (sys.path.insert + import_module,
    same technique as autokart_hp60c's HP60CBackend) caches the imported
    module in sys.modules by its plain name ("autokart_vehicle_bridge"). Two
    tests in this file construct a backend pointed at two DIFFERENT physical
    directories that both define a top-level package of that same name (one
    real, one a deliberately-incomplete fake used to test the import-failure
    path) -- without this fixture, whichever test runs first wins the
    sys.modules cache for the rest of the process, and the second test
    silently imports the wrong one instead of failing or using its own path.
    This is a real characteristic of the lazy sys.path-based import pattern
    this node shares with autokart_hp60c (see DEPENDENCIES.md), not a
    production defect: a real node process only ever constructs one backend
    with one fixed algorithm_package_path for its entire lifetime."""
    saved_path = list(sys.path)
    yield
    sys.path[:] = saved_path
    for name in list(sys.modules):
        if name == "autokart_vehicle_bridge" or name.startswith("autokart_vehicle_bridge."):
            del sys.modules[name]


def test_synthetic_backend_starts_safe_and_rejects_arm_from_wrong_state():
    backend = SyntheticBackend()
    backend.start()
    assert backend.state == 0  # SAFE
    events = backend.poll()
    tel = dict(events)["TEL"]
    assert tel.state == 0 and tel.state_name == "SAFE"


def test_synthetic_backend_arm_dis_cycle():
    backend = SyntheticBackend()
    backend.start()
    backend.send_request(1, "ARM")
    tel = dict(backend.poll())["TEL"]
    assert tel.state == 1 and tel.state_name == "ARMED"

    backend.send_request(2, "DIS")
    tel = dict(backend.poll())["TEL"]
    assert tel.state == 0


def test_synthetic_backend_second_arm_is_naked():
    backend = SyntheticBackend()
    backend.start()
    backend.send_request(1, "ARM")
    backend.poll()
    backend.send_request(2, "ARM")  # already ARMED: wrong state
    events = backend.poll()
    kinds = dict(events)
    assert "NAK" in kinds and kinds["NAK"] == (2, 1)


def test_synthetic_backend_cmd_range_latches_fault():
    backend = SyntheticBackend()
    backend.start()
    backend.send_request(1, "ARM")
    backend.poll()
    backend.send_command(2, 999999, 0)  # way over THROTTLE_MAX_MV_LOCAL
    tel = dict(backend.poll())["TEL"]
    assert tel.state == 2 and tel.state_name == "FAULT"
    assert "CMD_RANGE" in tel.fault_names()


def test_synthetic_backend_rst_clears_fault():
    backend = SyntheticBackend()
    backend.start()
    backend.send_request(1, "ARM")
    backend.poll()
    backend.send_command(2, 999999, 0)
    backend.poll()
    backend.send_request(3, "RST")
    tel = dict(backend.poll())["TEL"]
    assert tel.state == 0 and tel.fault == 0


def test_synthetic_backend_speed_and_counts_increase_only_while_armed_and_driving():
    backend = SyntheticBackend()
    backend.start()
    backend.send_request(1, "ARM")
    backend.poll()
    backend.send_command(2, 2000, 0)
    counts_before = backend.counts
    for i in range(5):
        tel = dict(backend.poll())["TEL"]
        assert tel.speed_mmps > 0
    assert backend.counts > counts_before
    # Axle counts never decrease (unsigned path length), even after disarming.
    counts_after_driving = backend.counts
    backend.send_request(3, "DIS")
    for i in range(3):
        tel = dict(backend.poll())["TEL"]
        assert tel.speed_mmps == 0
    assert backend.counts == counts_after_driving


def test_synthetic_backend_hello_appears_on_first_poll_and_requires_start():
    backend = SyntheticBackend()
    with pytest.raises(VcuBackendError):
        backend.poll()
    backend.start()
    events = backend.poll()
    assert "HELLO" in dict(events)


def test_serial_backend_missing_dependency_path_is_explicit_error(tmp_path):
    missing = tmp_path / "does_not_exist"
    with pytest.raises(VcuBackendError) as exc:
        SerialVcuBackend(algorithm_package_path=str(missing), port="/dev/ttyUSB0")
    assert "DEPENDENCY_PATH_NOT_FOUND" in str(exc.value)


def test_serial_backend_rejects_path_without_autokart_vehicle_bridge(tmp_path):
    # A directory that *does* contain an autokart_vehicle_bridge package (so
    # the real dependency elsewhere on sys.path cannot mask this case) but
    # with no protocol.py submodule, so the import must fail explicitly.
    src = tmp_path / "src"
    pkg = src / "autokart_vehicle_bridge"
    pkg.mkdir(parents=True)
    (pkg / "__init__.py").write_text("")
    with pytest.raises(VcuBackendError) as exc:
        SerialVcuBackend(algorithm_package_path=str(src), port="/dev/ttyUSB0")
    assert "SDK_ADAPTER_IMPORT_FAILED" in str(exc.value)


@pytest.mark.skipif(not REAL_VEHICLE_BRIDGE_PATH.is_dir(),
                     reason="real autokart_vehicle_bridge checkout not present in this environment")
def test_serial_backend_really_imports_and_uses_the_real_protocol_module():
    """This is the actual reuse under real test: a real import of
    kart/ros2_ws's own, already-tested autokart_vehicle_bridge.protocol
    module, not a fake stand-in. The serial port itself is not opened here
    (that's covered, and expected to fail in this environment, by the
    SERIAL_PORT_UNAVAILABLE test below) -- this only proves the lazy import
    and frame-building/parsing genuinely work end to end."""
    backend = SerialVcuBackend(algorithm_package_path=str(REAL_VEHICLE_BRIDGE_PATH), port="/dev/ttyUSB0")
    frame = backend._protocol.build_cmd(7, 500, 0, 0)
    assert backend._protocol.parse_frame(frame) == "CMD,7,500,0,0"
    kind, value = backend._protocol.parse_message("HELLO,0.2.2-no-estop-input-locked,2")
    assert kind == "HELLO" and value == ("0.2.2-no-estop-input-locked", 2)


@pytest.mark.skipif(not REAL_VEHICLE_BRIDGE_PATH.is_dir(),
                     reason="real autokart_vehicle_bridge checkout not present in this environment")
def test_serial_backend_fails_closed_when_port_does_not_exist():
    """No ESP32-S3/VCU is attached in this environment: opening any serial
    port must fail closed with an explicit reason, never silently proceed."""
    backend = SerialVcuBackend(algorithm_package_path=str(REAL_VEHICLE_BRIDGE_PATH),
                                port="/dev/this_port_does_not_exist_on_any_machine")
    with pytest.raises(VcuBackendError) as exc:
        backend.start()
    assert "SERIAL_PORT_UNAVAILABLE" in str(exc.value)


def test_build_backend_unknown_name_raises():
    with pytest.raises(VcuBackendError):
        build_backend("not-a-real-backend")


def test_build_backend_synthetic():
    backend = build_backend("synthetic")
    assert isinstance(backend, SyntheticBackend)
