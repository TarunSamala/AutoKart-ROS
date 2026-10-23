import json
from pathlib import Path
import pytest
from autokart_dashboard_gateway_node.readonly import PublishedState


def test_latest_state_preserves_provenance_without_motion_authority():
    state = PublishedState()
    state.update('/mission/state', {'source': 'SYNTHETIC', 'vehicle_authorized': True})
    state.update('/mission/state', {'source': 'REPLAY', 'valid': False})
    snapshot = state.snapshot()
    assert snapshot['vehicle_authorized'] is False
    assert snapshot['output_inhibited'] is True
    assert snapshot['topics']['/mission/state']['published'] == {'source':'REPLAY','valid':False}
    assert len(snapshot['topics']) == 1
    json.dumps(snapshot, allow_nan=False)


def test_readonly_ros_adapter_removes_all_application_publishers():
    rclpy = pytest.importorskip('rclpy')
    from autokart_dashboard_gateway_node.adapter import load_gateway
    from autokart_dashboard_gateway_node.readonly import make_readonly_node
    root = Path(__file__).resolve().parents[3]
    gateway = load_gateway(str(root/'kart/ros2_ws/src/autokart_dashboard_gateway'),
                           str(root/'autonomy_stack_phase1/phase_01_perception/src'))
    rclpy.init()
    node, state = make_readonly_node(gateway, gateway.MissionStore('SIMULATION','readonly-test'))
    try:
        assert not node.send_request('arm')
        assert not node.send_demo_drive_message('drive','forward',1)
        assert not [p for p in node.publishers if p.topic_name not in {'/rosout','/parameter_events'}]
        topics = {s.topic_name for s in node.subscriptions}
        assert {'/odom','/mission/state','/safety/status','/system/health'} <= topics
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_canonical_http_state_is_read_only_and_json_serializable():
    import asyncio
    from aiohttp import web
    from aiohttp.test_utils import TestClient, TestServer
    from autokart_dashboard_gateway_node.readonly import attach_state_endpoint
    async def scenario():
        state = PublishedState()
        state.update('/diagnostics', {'level': b'\x02', 'missing': float('nan')})
        app = web.Application()
        attach_state_endpoint(app, state)
        async with TestClient(TestServer(app)) as client:
            response = await client.get('/integration/api/state')
            assert response.status == 200
            body = await response.json()
            assert body['topics']['/diagnostics']['published'] == {'level':[2], 'missing':None}
            response = await client.post('/integration/api/state', json={'arm':True})
            assert response.status == 405
    asyncio.run(scenario())


def test_canonical_odometry_source_and_capture_stamp_are_required():
    from types import SimpleNamespace
    from autokart_dashboard_gateway_node.readonly import OdometryCompanion
    output = []
    gate = OdometryCompanion('REAL_KART', output.append, lambda: 1_100_000_000)
    def pose(stamp):
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=stamp)))
    def status(stamp, source):
        return SimpleNamespace(data=json.dumps({'schema_version':1,'valid':True,
                      'source':source,'sample_time_ns':1_000_000_000+stamp}))
    gate.on_pose(pose(0))
    gate.on_status(status(0,'SYNTHETIC'))
    assert not output
    gate.on_status(status(1,'LIVE_PHYSICAL'))
    gate.on_pose(pose(2))
    assert not output
    gate.on_pose(pose(1))
    assert len(output) == 1
