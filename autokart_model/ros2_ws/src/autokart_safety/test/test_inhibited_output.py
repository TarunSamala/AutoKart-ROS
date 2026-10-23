"""Integration output cannot authorize motion, even with verified flags set."""
import json
from pathlib import Path
import pytest
rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter
from std_msgs.msg import String
from autokart_safety.node import AutokartSafetyNode
from autokart_msgs.msg import MissionDebug


def test_arm_and_demo_requests_are_inhibited_and_stop_candidate_is_invalid():
    rclpy.init()
    core_path = Path(__file__).resolve().parents[3] / 'kart/ros2_ws/src/autokart_vcu_safety'
    node = AutokartSafetyNode(parameter_overrides=[
        Parameter('vcu_safety_package_path', value=str(core_path)),
        Parameter('calibration_verified', value=True),
        Parameter('vehicle_config_verified', value=True),
        Parameter('braking_verified', value=True),
    ])
    observer = rclpy.create_node('test_inhibited_candidate')
    received = []
    observer.create_subscription(MissionDebug,
        '/vehicle/authorized_command', lambda msg: received.append(json.loads(msg.payload_json)), 10)
    try:
        for request in ['arm', 'confirm_manual_power_cut', 'demo_drive_start']:
            node._on_request(String(data=request))
        assert not node.pending_requests
        for _ in range(30):
            node._tick()
            rclpy.spin_once(observer, timeout_sec=0.02)
        assert received
        assert all(not p['vehicle_authorized'] and not p['valid'] and p['output_inhibited']
                   and p['stop_requested'] and p['target_speed_mps'] == 0 for p in received)
    finally:
        observer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
