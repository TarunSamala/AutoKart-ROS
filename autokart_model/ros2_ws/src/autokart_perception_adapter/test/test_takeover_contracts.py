import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import pytest
from autokart_perception_adapter.backends import AlgorithmBackend,PerceptionBackendError
from autokart_perception_adapter.node import image_to_rgb_array,image_to_depth_array,PerceptionAdapterNode
from sensor_msgs.msg import Image
from autokart_msgs.msg import MissionDebug
import rclpy


def test_padded_rgb_and_big_endian_depth():
    rgb=Image(height=1,width=1,step=4,encoding='rgb8',data=b'\x01\x02\x03\xff')
    assert image_to_rgb_array(rgb).tolist()==[[[1,2,3]]]
    depth=Image(height=1,width=2,step=6,is_bigendian=1,encoding='16UC1',data=b'\x01\x00\x02\x00\xff\xff')
    assert image_to_depth_array(depth).tolist()==[[256,512]]


def test_live_physical_missing_model_cannot_fall_back_to_synthetic():
    backend=AlgorithmBackend.__new__(AlgorithmBackend);backend._synthetic=True
    with pytest.raises(PerceptionBackendError,match='LIVE_MODEL_NOT_CONFIGURED'):
        backend.process(None,None,1.,1.,0.,0.,1,'LIVE_PHYSICAL')


@pytest.mark.parametrize('payload',['[]','null','42'])
def test_nondict_camera_metadata_is_reported_without_crash(payload):
    rclpy.init();node=PerceptionAdapterNode()
    try:
        msg=MissionDebug(payload_json=payload)
        node._on_camera_metadata(msg)
        assert node._last_fault=='CAMERA_METADATA_MALFORMED'
    finally:
        node.destroy_node();rclpy.shutdown()
