"""Camera/status contract tests: read-only, paired, source-qualified, fail closed."""

import asyncio
import json
from types import SimpleNamespace

import numpy as np
from aiohttp.test_utils import TestClient, TestServer

from autokart_dashboard_gateway.camera_feed import CameraFeed
from autokart_dashboard_gateway.core import MissionStore
from autokart_dashboard_gateway.gateway import create_app, safety_envelope_payload


def image(array, encoding):
    return SimpleNamespace(height=array.shape[0], width=array.shape[1],
                           step=array.strides[0], encoding=encoding,
                           is_bigendian=False, data=array.tobytes())


def test_camera_pair_detections_stale_and_disconnect(monkeypatch):
    clock = [10.0]
    monkeypatch.setattr("autokart_dashboard_gateway.camera_feed.time.monotonic", lambda: clock[0])
    feed = CameraFeed(max_age_s=1.0)
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    depth = np.ones((480, 640), dtype=np.uint16) * 123
    feed.update_image("rgb", 1_000_000_123, image(rgb, "bgr8"))
    assert feed.snapshot()["reason"] == "CAMERA_PAIR_MISSING"
    assert feed.image("1000000123", "rgb") is None
    feed.update_image("depth", 1_000_000_124, image(depth, "mono16"))
    assert not feed.snapshot()["available"]
    feed.update_image("depth", 1_000_000_123, image(depth, "mono16"))
    feed.update_status(1_000_000_123, {"source": "LIVE", "valid": False,
                                       "sample_time_ns": 1_000_000_123,
                                       "calibration_status": "UNVERIFIED",
                                       "detections": [{"label": "blue", "confidence": 0.9,
                                                       "box": [1, 2, 20, 30]}],
                                       "processing_ms": 12.5})
    snapshot = feed.snapshot()
    assert snapshot["available"] and snapshot["frame_id"] == "1000000123"
    assert snapshot["capture_time_ns"] == "1000000123"
    assert snapshot["source"] == "LIVE" and snapshot["valid"] is False
    assert snapshot["depth_qualified"] is False
    assert snapshot["depth_stats"]["raw_unit"] == "UNVERIFIED"
    assert snapshot["detections"][0]["box"] == [1, 2, 20, 30]
    assert feed.image(snapshot["frame_id"], "rgb").startswith(b"\xff\xd8")
    clock[0] = 10.5
    feed.update_image("rgb", 1_000_000_123, image(rgb, "bgr8"))
    feed.update_status(1_000_000_123, {"source": "LIVE", "valid": True,
                                       "sample_time_ns": 1_000_000_123})
    assert feed.snapshot()["age_ms"] == 500.0
    assert feed.snapshot()["valid"] is False
    clock[0] = 11.1
    assert feed.snapshot()["reason"] == "CAMERA_PAIR_STALE"
    assert feed.image("1000000123", "rgb") is None


def test_camera_rejects_mismatched_image_frames_and_status_stamps():
    feed = CameraFeed()
    rgb = image(np.zeros((480, 640, 3), dtype=np.uint8), "bgr8")
    depth = image(np.ones((480, 640), dtype=np.uint16), "mono16")
    rgb.header = SimpleNamespace(frame_id="camera_color")
    depth.header = SimpleNamespace(frame_id="camera_depth")
    feed.update_image("rgb", 123, rgb)
    feed.update_image("depth", 123, depth)
    assert feed.snapshot()["reason"] == "CAMERA_PAIR_FRAME_MISMATCH"
    depth.header.frame_id = "camera_color"
    feed.update_image("rgb", 124, rgb)
    feed.update_image("depth", 124, depth)
    feed.update_status(124, {"source": "LIVE", "sample_time_ns": 123})
    assert feed.snapshot()["source"] == "UNKNOWN"


def test_camera_latency_accepts_vendor_monotonic_capture_clock(monkeypatch):
    monkeypatch.setattr("autokart_dashboard_gateway.camera_feed.time.time_ns",
                        lambda: 1_800_000_000_000_000_000)
    monkeypatch.setattr("autokart_dashboard_gateway.camera_feed.time.monotonic_ns",
                        lambda: 1_300_000_000)
    feed = CameraFeed()
    feed.update_image("rgb", 1_000_000_000,
                      image(np.zeros((480, 640, 3), dtype=np.uint8), "bgr8"))
    feed.update_image("depth", 1_000_000_000,
                      image(np.ones((480, 640), dtype=np.uint16), "mono16"))
    assert feed.snapshot()["capture_to_gateway_ms"] == 300.0


def test_startup_fault_status_in_wall_clock_does_not_block_monotonic_camera_pair():
    feed = CameraFeed()
    feed.update_status(1_800_000_000_000_000_000,
                       {"source": "LIVE", "sample_time_ns": 1_800_000_000_000_000_000,
                        "valid": False, "reason_codes": ["CALIBRATION_UNAVAILABLE"]})
    stamp = 40_000_000_000_000
    feed.update_status(stamp, {"source": "LIVE", "sample_time_ns": stamp,
                               "valid": False, "reason_codes": ["LIVE_METRIC_CALIBRATION_UNVERIFIED"]})
    feed.update_image("rgb", stamp,
                      image(np.zeros((480, 640, 3), dtype=np.uint8), "bgr8"))
    feed.update_image("depth", stamp,
                      image(np.ones((480, 640), dtype=np.uint16), "mono16"))
    snapshot = feed.snapshot()
    assert snapshot["available"] and snapshot["source"] == "LIVE"
    assert snapshot["valid"] is False and snapshot["depth_qualified"] is False


def test_status_without_odometry_and_http_reconnect():
    async def scenario():
        store = MissionStore("SIMULATION")
        camera = CameraFeed()
        async with TestServer(create_app(store, camera=camera)) as server:
            async with TestClient(server) as client:
                response = await client.get("/mission/api/status")
                empty = await response.json()
                assert response.status == 200 and not empty["ros_connected"]
                assert empty["odometry"]["state"] == "MISSING"
                store.update_debug("perception", 1.0, {"source": "SIMULATED", "valid": False,
                                                        "reason_codes": ["CALIBRATION_UNVERIFIED"]})
                current = await (await client.get("/mission/api/status")).json()
                assert current["ros_connected"] and not current["odometry"]["current"]
                assert current["stages"]["perception"]["payload"]["valid"] is False
                assert current["vehicle_authorized"] is False
                assert (await client.get("/mission/api/camera")).status == 200
                assert (await client.get("/mission/api/camera/1/rgb")).status == 404
                assert (await client.post("/mission/api/status")).status == 405
                # A later GET after an interruption reads current store state.
                store.capture(1.0, {"x_m": 0, "y_m": 0, "yaw_rad": 0, "speed_mps": 0})
                reconnected = await (await client.get("/mission/api/status")).json()
                assert reconnected["odometry"]["state"] == "CURRENT"
    asyncio.run(scenario())


def test_health_endpoint_reports_liveness_without_actuating_or_requiring_a_fresh_topic():
    async def scenario():
        store = MissionStore("REAL_KART")
        async with TestServer(create_app(store)) as server:
            async with TestClient(server) as client:
                response = await client.get("/health")
                body = await response.json()
                assert response.status == 200
                assert body["status"] == "ok"
                assert body["mode"] == "LIVE" and body["source"] == "REAL_KART"
                assert body["vehicle_authority"] == "OFF"
                # No ROS topic has published anything yet; /health still answers (process liveness
                # only), unlike /mission/api/status's per-topic freshness fields.
                assert body["ros_connected"] is False
                assert body["uptime_s"] >= 0
                assert (await client.post("/health")).status == 405
    asyncio.run(scenario())


def test_health_endpoint_reports_replay_mode_with_no_ros_connected_field():
    async def scenario():
        store = MissionStore("SIMULATION")
        async with TestServer(create_app(store, replay=True)) as server:
            async with TestClient(server) as client:
                body = await (await client.get("/health")).json()
                assert body["mode"] == "REPLAY"
                assert body["ros_connected"] is None
    asyncio.run(scenario())


def test_readiness_endpoint_passes_through_the_saved_report_unmodified(tmp_path):
    async def scenario():
        report = {"gates": [{"name": "x", "status": "PASS"}], "ready": False, "reasons": ["y"]}
        path = tmp_path / "readiness_report_latest.json"
        path.write_text(json.dumps(report))
        store = MissionStore("REAL_KART")
        async with TestServer(create_app(store, readiness_path=path)) as server:
            async with TestClient(server) as client:
                body = await (await client.get("/mission/api/readiness")).json()
                assert body["available"] is True
                assert body["gates"] == report["gates"] and body["ready"] is False
                assert body["age_s"] >= 0
    asyncio.run(scenario())


def test_readiness_endpoint_reports_unavailable_not_fabricated_when_never_generated(tmp_path):
    async def scenario():
        store = MissionStore("REAL_KART")
        missing = tmp_path / "never_written.json"
        async with TestServer(create_app(store, readiness_path=missing)) as server:
            async with TestClient(server) as client:
                body = await (await client.get("/mission/api/readiness")).json()
                assert body["available"] is False
                assert body["reason"] == "READINESS_REPORT_NOT_GENERATED"
                assert "gates" not in body
    asyncio.run(scenario())


def test_safety_envelope_payload_parses_state_and_strips_calibration_suffix():
    message = SimpleNamespace(limiting_reason="CRITICAL:OBSTACLE_WITHIN_MINIMUM_CLEARANCE:CALIBRATION_UNVERIFIED",
                              valid=True, clear_distance_m=0.4, stopping_distance_m=2.8,
                              guaranteed_deceleration_mps2=2.0, data_source="LIVE")
    payload = safety_envelope_payload(message)
    assert payload == {"state": "CRITICAL", "reason": "OBSTACLE_WITHIN_MINIMUM_CLEARANCE:CALIBRATION_UNVERIFIED",
                       "valid": True, "calibration_verified": False, "clear_distance_m": 0.4,
                       "stopping_distance_m": 2.8, "guaranteed_deceleration_mps2": 2.0, "data_source": "LIVE"}


def test_safety_envelope_payload_converts_nan_distances_to_none_for_json():
    message = SimpleNamespace(limiting_reason="UNKNOWN:ODOMETRY_STALE_OR_MISSING",
                              valid=False, clear_distance_m=float("nan"), stopping_distance_m=float("nan"),
                              guaranteed_deceleration_mps2=2.0, data_source="SIMULATED")
    payload = safety_envelope_payload(message)
    assert payload["state"] == "UNKNOWN"
    assert payload["valid"] is False
    assert payload["clear_distance_m"] is None
    assert payload["stopping_distance_m"] is None


def test_safety_envelope_payload_never_defaults_missing_data_source_to_live():
    """RC-06 audit finding B: a message with no/garbage data_source must read UNVERIFIED, never LIVE."""
    missing = SimpleNamespace(limiting_reason="CLEAR:OBSTACLE_OUTSIDE_BRAKING_ENVELOPE", valid=True,
                              clear_distance_m=5.0, stopping_distance_m=2.8, guaranteed_deceleration_mps2=2.0)
    assert safety_envelope_payload(missing)["data_source"] == "UNVERIFIED"


def test_websocket_forwards_only_whitelisted_requests_and_never_without_a_sink():
    """The browser's only effect on the vehicle: a bounded request string, forwarded verbatim, never a command."""
    from autokart_dashboard_gateway.gateway import ALLOWED_SAFETY_REQUESTS

    async def scenario():
        store = MissionStore("SIMULATION")
        received = []

        def sink(action):
            if action not in ALLOWED_SAFETY_REQUESTS:
                return False
            received.append(action)
            return True

        async with TestServer(create_app(store, request_sink=sink)) as server:
            async with TestClient(server) as client:
                async with client.ws_connect("/mission/ws") as ws:
                    await ws.receive_json()  # initial manifest frame
                    await ws.send_json({"type": "request", "action": "arm"})
                    await ws.send_json({"type": "request", "action": "confirm_manual_power_cut"})
                    await ws.send_json({"type": "request", "action": "stop"})
                    await ws.send_json({"type": "request", "action": "confirm_estop_tested"})
                    await ws.send_json({"type": "request", "action": "set_throttle_v"})  # not in the whitelist
                    await ws.send_json({"type": "request", "action": "DROP TABLE"})
                    rejected = await ws.receive_json()
                    assert rejected == {"type": "request_rejected", "action": "confirm_estop_tested"}
                    rejected2 = await ws.receive_json()
                    assert rejected2["action"] == "set_throttle_v"
                    rejected3 = await ws.receive_json()
                    assert rejected3["action"] == "DROP TABLE"
        assert received == ["arm", "confirm_manual_power_cut", "stop"]
    asyncio.run(scenario())


def test_demo_drive_ping_calls_backend_ping_not_the_sink():
    async def scenario():
        store = MissionStore("REAL_KART")
        pings = []
        async with TestServer(create_app(store, demo_drive_sink=lambda *a, **k: True,
                                         demo_drive_ping=lambda: pings.append(1))) as server:
            async with TestClient(server) as client:
                async with client.ws_connect("/mission/ws") as ws:
                    await ws.receive_json()
                    await ws.send_json({"type": "demo_drive", "action": "ping"})
                    await ws.send_json({"type": "demo_drive", "action": "ping"})
                    await asyncio.sleep(0.05)
        assert pings == [1, 1]
    asyncio.run(scenario())


def test_demo_drive_drive_and_stop_forwarded_through_sink_with_params():
    async def scenario():
        store = MissionStore("REAL_KART")
        received = []
        async with TestServer(create_app(store, demo_drive_sink=lambda action, **kw: (
                received.append((action, kw)) or True))) as server:
            async with TestClient(server) as client:
                async with client.ws_connect("/mission/ws") as ws:
                    await ws.receive_json()
                    await ws.send_json({"type": "demo_drive", "action": "drive",
                                       "direction": "forward", "duration_s": 2.0})
                    await ws.send_json({"type": "demo_drive", "action": "stop"})
                    await asyncio.sleep(0.05)
        assert received[0] == ("drive", {"direction": "forward", "duration_s": 2.0})
        assert received[1] == ("stop", {"direction": "", "duration_s": 0.0})
    asyncio.run(scenario())


def test_demo_drive_rejected_action_is_surfaced_not_silently_dropped():
    async def scenario():
        store = MissionStore("REAL_KART")
        async with TestServer(create_app(store, demo_drive_sink=lambda *a, **k: False)) as server:
            async with TestClient(server) as client:
                async with client.ws_connect("/mission/ws") as ws:
                    await ws.receive_json()
                    await ws.send_json({"type": "demo_drive", "action": "drive",
                                       "direction": "reverse", "duration_s": 2.0})
                    rejected = await ws.receive_json()
                    assert rejected == {"type": "demo_drive_rejected", "action": "drive"}
    asyncio.run(scenario())


def test_validate_demo_drive_request_rejects_reverse_bad_duration_and_unknown_action():
    from autokart_dashboard_gateway.gateway import validate_demo_drive_request

    assert validate_demo_drive_request("stop", "", 0.0) is True
    assert validate_demo_drive_request("drive", "forward", 2.0) is True
    assert validate_demo_drive_request("drive", "reverse", 2.0) is False
    assert validate_demo_drive_request("drive", "forward", 0.0) is False
    assert validate_demo_drive_request("drive", "forward", 999.0) is False
    assert validate_demo_drive_request("set_throttle_v", "forward", 2.0) is False


def test_not_fitted_status_is_explicit_and_request_capability_is_truthful():
    async def scenario():
        store = MissionStore("REAL_KART")
        async with TestServer(create_app(store, request_sink=None)) as server:
            async with TestClient(server) as client:
                status = await (await client.get("/mission/api/status")).json()
                assert status["estop_input_status"] == "NOT FITTED - MANUAL POWER CUT ONLY"
                assert status["safety_requests_enabled"] is False
        async with TestServer(create_app(store, request_sink=lambda action: True)) as server:
            async with TestClient(server) as client:
                status = await (await client.get("/mission/api/status")).json()
                assert status["safety_requests_enabled"] is True
    asyncio.run(scenario())


def test_websocket_request_sink_return_value_gates_rejection_even_if_whitelisted():
    """send_request's own return value (e.g. autokart_vcu_safety.core whitelist) is still honoured here,
    not just the gateway's static ALLOWED_SAFETY_REQUESTS set."""
    async def scenario():
        store = MissionStore("SIMULATION")
        async with TestServer(create_app(store, request_sink=lambda action: False)) as server:
            async with TestClient(server) as client:
                async with client.ws_connect("/mission/ws") as ws:
                    await ws.receive_json()
                    await ws.send_json({"type": "request", "action": "arm"})
                    rejected = await ws.receive_json()
                    assert rejected == {"type": "request_rejected", "action": "arm"}
    asyncio.run(scenario())


def test_websocket_ignores_malformed_and_non_request_messages():
    async def scenario():
        store = MissionStore("SIMULATION")
        received = []
        async with TestServer(create_app(store, request_sink=received.append)) as server:
            async with TestClient(server) as client:
                async with client.ws_connect("/mission/ws") as ws:
                    await ws.receive_json()
                    await ws.send_str("not json")
                    await ws.send_json({"type": "not_a_request"})
                    await ws.send_json({"type": "request"})  # missing "action"
                    await ws.send_json("just a string")
                    await ws.send_json({"type": "request", "action": "arm"})
                    # Let the event loop process the queued messages before asserting.
                    for _ in range(5):
                        await asyncio.sleep(0)
        assert received == ["arm"]
    asyncio.run(scenario())


def test_replay_mode_never_forwards_requests_even_if_a_sink_is_given():
    async def scenario():
        store = MissionStore("SIMULATION")
        received = []
        async with TestServer(create_app(store, replay=True, request_sink=received.append)) as server:
            async with TestClient(server) as client:
                async with client.ws_connect("/mission/ws") as ws:
                    await ws.receive_json()
                    await ws.send_json({"type": "request", "action": "arm"})
                    for _ in range(5):
                        await asyncio.sleep(0)
        assert received == []
    asyncio.run(scenario())

    garbage = SimpleNamespace(limiting_reason="CLEAR:OBSTACLE_OUTSIDE_BRAKING_ENVELOPE", valid=True,
                              clear_distance_m=5.0, stopping_distance_m=2.8, guaranteed_deceleration_mps2=2.0,
                              data_source="TOTALLY_REAL_I_PROMISE")
    assert safety_envelope_payload(garbage)["data_source"] == "UNVERIFIED"
