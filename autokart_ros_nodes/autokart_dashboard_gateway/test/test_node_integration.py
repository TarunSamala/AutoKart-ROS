"""Integration test: real rclpy node built from the REAL, reused
autokart_dashboard_gateway implementation via this package's adapter.

The single most safety-critical property this node must have is also the
one this file tests directly, not just by reading source: the gateway node
must have NO publisher capable of commanding vehicle motion -- only the two
bounded request-forwarding publishers, which other nodes (autokart_safety,
the vehicle bridge) independently gate. See reports/SENIOR_REVIEWER_REPORT.md
for the full audit this test backs up.
"""
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")

from autokart_dashboard_gateway_node.adapter import load_gateway  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[3]
REAL_GATEWAY_SRC = REPO_ROOT / "kart" / "ros2_ws" / "src" / "autokart_dashboard_gateway"
REAL_PERCEPTION_SRC = REPO_ROOT / "autonomy_stack_phase1" / "phase_01_perception" / "src"
pytestmark = pytest.mark.skipif(
    not (REAL_GATEWAY_SRC.is_dir() and REAL_PERCEPTION_SRC.is_dir()),
    reason="real gateway + perception checkouts not present")


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_node(ros_context):
    gateway = load_gateway(str(REAL_GATEWAY_SRC), str(REAL_PERCEPTION_SRC))
    store = gateway.MissionStore("SIMULATION", "test-run")
    node = gateway.make_ros_node(store)
    return gateway, node


def test_gateway_node_has_exactly_two_publishers_both_bounded_requests(ros_context):
    """Enumerate every publisher this node actually created (via rclpy's own
    graph introspection, not just reading attribute names) and assert the
    set is exactly {request topic, demo-drive topic}. A future change that
    accidentally adds a new publisher -- e.g. a direct throttle/brake/arm
    command topic -- would be caught by this test failing, not just by a
    human re-reading the diff."""
    gateway, node = _make_node(ros_context)
    try:
        topics_and_types = node.get_publisher_names_and_types_by_node(node.get_name(), node.get_namespace())
        # Exclude rclpy's own automatic per-node publishers (/rosout,
        # /parameter_events) -- every Node gets these; they are not an
        # actuation path.
        topic_names = {name for name, _ in topics_and_types} - {"/rosout", "/parameter_events"}
        assert topic_names == {"/autokart/safety/request", "/autokart/demo_drive/control"}, topic_names
    finally:
        node.destroy_node()


def test_send_request_rejects_everything_outside_the_whitelist(ros_context):
    gateway, node = _make_node(ros_context)
    try:
        assert node.send_request("arm") is True
        assert node.send_request("set_throttle_v") is False
        assert node.send_request("DROP TABLE") is False
        assert node.send_request("confirm_estop_tested") is False
    finally:
        node.destroy_node()


def test_demo_drive_reverse_is_refused_not_silently_dropped(ros_context):
    """Firmware has no reverse capability (see autokart_vehicle_bridge's
    demo_drive.py docstring) -- the gateway's own validator must say no
    rather than forwarding a request that would silently have no effect."""
    gateway, node = _make_node(ros_context)
    try:
        assert node.send_demo_drive_message("drive", direction="reverse", duration_s=2.0) is False
        assert node.send_demo_drive_message("drive", direction="forward", duration_s=2.0) is True
        assert node.send_demo_drive_message("drive", direction="forward", duration_s=999.0) is False
    finally:
        node.destroy_node()


def test_gateway_node_has_no_service_clients_either(ros_context):
    """A service client could also be an actuation path this gateway should
    never have. Confirmed empty via rclpy's own client registry, not just by
    reading source."""
    gateway, node = _make_node(ros_context)
    try:
        assert node.get_client_names_and_types_by_node(node.get_name(), node.get_namespace()) == []
    finally:
        node.destroy_node()
