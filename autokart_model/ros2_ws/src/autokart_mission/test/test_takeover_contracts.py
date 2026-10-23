import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import pytest,rclpy
from autokart_msgs.msg import MissionDebug
from autokart_mission.node import MissionNode


@pytest.mark.parametrize('payload',['[]','null','42'])
def test_scalar_mapping_status_is_rejected(payload):
    rclpy.init();node=MissionNode()
    try: node.on_mapping_status(MissionDebug(payload_json=payload))
    finally: node.destroy_node();rclpy.shutdown()
