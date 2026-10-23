import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import pytest,rclpy
from rclpy.parameter import Parameter
from autokart_msgs.msg import MissionDebug
from autokart_mapping.node import MappingNode
ROOT=Path(__file__).resolve().parents[3]


def test_old_live_or_synthetic_pose_without_matching_source_never_qualifies():
    rclpy.init()
    node=MappingNode(parameter_overrides=[Parameter('algorithm_package_path',value=str(ROOT/'autonomy_stack_phase1/phase_01_perception/src'))])
    try:
        stamp=node.get_clock().now().nanoseconds
        pose=node.algorithm.PoseEstimate(0,0,0,0,stamp,0.05)
        node.poses[stamp]=pose
        assert node.pose_at_capture(stamp) is None
        node.odom_statuses[stamp]=dict(valid=True,continuity_lost=False,source='LIVE',expires_ns=stamp+250_000_000)
        assert node.pose_at_capture(stamp) is None
        node.odom_statuses[stamp]['source']='SYNTHETIC'
        assert node.pose_at_capture(stamp) is pose
        node.odom_statuses[stamp]['expires_ns']=stamp-1
        assert node.pose_at_capture(stamp) is None
    finally: node.destroy_node();rclpy.shutdown()


@pytest.mark.parametrize('payload',['[]','null','42'])
def test_scalar_companion_is_fault_not_exception(payload):
    rclpy.init();node=MappingNode()
    try: node.on_perception_status(MissionDebug(payload_json=payload))
    finally: node.destroy_node();rclpy.shutdown()
