"""Fault injection for permanently inhibited integration policy."""
import json
import math
from pathlib import Path
import pytest
rclpy = pytest.importorskip('rclpy')
from autokart_msgs.msg import MissionDebug, ActuatorFeedback
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from autokart_safety.node import AutokartSafetyNode

@pytest.fixture
def node():
    rclpy.init()
    result = AutokartSafetyNode()
    yield result
    result.destroy_node()
    rclpy.shutdown()

@pytest.mark.parametrize('payload', ['invalid', '[]', 'null', '{"source":"SYNTHETIC","valid":true}', '{"source":"REPLAY","valid":true}'])
def test_invalid_or_nonphysical_status_cannot_refresh_critical_gate(node, payload):
    node._touch('planner')
    message = MissionDebug(payload_json=payload)
    node._make_status_handler('planner')(message)
    assert node._age('planner') is None


def test_angular_velocity_does_not_become_translational_speed(node):
    node._on_feedback(ActuatorFeedback(motor_speed_radps=123.0, valid=True))
    assert node.speed_mps is None and not node.speed_valid
    msg = Odometry()
    msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
    msg.twist.twist.linear.x = float('nan')
    node._on_odometry(msg)
    assert node.speed_mps is None and not node.speed_valid


def test_request_flood_is_bounded_and_arm_excluded(node):
    for _ in range(200):
        node._on_request(String(data='stop'))
        node._on_request(String(data='arm'))
    assert len(node.pending_requests) == 32
    assert set(node.pending_requests) == {'stop'}


def test_old_status_redelivered_now_cannot_refresh_core_readiness(node):
    msg = MissionDebug()
    now = node.get_clock().now().nanoseconds
    stamp = now - 2_000_000_000
    msg.measurement_time.sec, msg.measurement_time.nanosec = divmod(stamp,1_000_000_000)
    msg.payload_json = json.dumps({'schema_version':1,'source':'LIVE_PHYSICAL','valid':True,
                                   'sample_time_ns':stamp,'expires_ns':now+1_000_000_000})
    node._make_status_handler('planner')(msg)
    assert node._age('planner') is None


def test_speed_requires_matching_physical_odometry_companion(node):
    msg = Odometry()
    msg.header.frame_id, msg.child_frame_id = 'odom','base_link'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.twist.twist.linear.x = 0.4
    stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
    node._on_odometry(msg)
    assert not node.speed_valid
    payload = dict(schema_version=1,source='SYNTHETIC',valid=True,
                   sample_time_ns=stamp,expires_ns=stamp+150_000_000)
    node._on_odometry_status(String(data=json.dumps(payload)))
    assert not node.speed_valid
    payload['source'] = 'LIVE_PHYSICAL'
    node._on_odometry_status(String(data=json.dumps(payload)))
    assert node.speed_valid and node.speed_mps == 0.4
