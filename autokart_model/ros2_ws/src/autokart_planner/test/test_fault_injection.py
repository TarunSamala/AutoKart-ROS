"""ROS callback faults at canonical contract boundaries, with no hardware access."""
import json
import pytest
rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter
from autokart_msgs.msg import MissionDebug
from autokart_planner.node import PlannerNode


@pytest.fixture
def node():
    rclpy.init()
    instance = PlannerNode()
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def test_malformed_mission_drops_prior_mission(node):
    msg = MissionDebug()
    msg.payload_json = '[]'
    node.on_mission(msg)
    assert node.mission is None


def test_never_receiving_path_reports_invalid_repeatedly(node):
    captured = []
    node.publish = captured.append
    node.last_path_wall_s = 0.
    node.on_timer()
    node.on_timer()
    assert len(captured) == 2
    assert all(not p['valid'] for p in captured)


def test_missing_backend_reports_persistent_fault(node):
    captured = []
    node.publish = captured.append
    node.engine = None
    node.startup_error = 'DEPENDENCY_PATH_NOT_FOUND'
    node.on_timer()
    node.on_timer()
    assert len(captured) == 2
    assert all(p['reason_codes'] == ['DEPENDENCY_PATH_NOT_FOUND'] for p in captured)


def test_missing_mission_rejects_external_reference(node):
    captured = []
    node.publish = captured.append
    node._process([(0.,0.),(1.,0.)], node.get_clock().now().nanoseconds)
    assert captured[-1]['reason_codes'] == ['ODOMETRY_STATUS_UNQUALIFIED']
