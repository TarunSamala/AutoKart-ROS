import json
import pytest
rclpy=pytest.importorskip("rclpy")
from autokart_vcu_bridge.node import VcuBridgeNode

def test_bridge_has_no_command_input_and_ticks_without_writes():
    rclpy.init()
    node=VcuBridgeNode(start_timer=False)
    try:
        node.backend.send_command=lambda *a: pytest.fail("unexpected wire command")
        node.backend.send_request=lambda *a: pytest.fail("unexpected wire request")
        node._tick()
        assert node.seq==1
        assert not list(node.subscriptions)
        assert node.source=="SYNTHETIC"
    finally: node.destroy_node(); rclpy.shutdown()
