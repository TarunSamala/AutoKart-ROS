import asyncio, json, threading, time
from types import SimpleNamespace as NS
import cv2, numpy as np
from gateway_core import GatewayCore, GatewayServer, jpeg_size, nz


def hdr(fid=""):
    return NS(stamp=NS(sec=100, nanosec=500000000), frame_id=fid)


def test_jpeg_size_and_nz():
    ok, buf = cv2.imencode(".jpg", np.zeros((120, 200, 3), np.uint8))
    assert jpeg_size(buf.tobytes()) == (200, 120)
    assert nz(float("nan")) is None and nz(1.5) == 1.5 and nz(float("inf")) is None


def test_image_and_detections_leave_together():
    out = []
    t = [0.0]
    c = GatewayCore(out.append, "LIVE", clock=lambda: t[0])
    ok, buf = cv2.imencode(".jpg", np.zeros((120, 200, 3), np.uint8))
    det = NS(header=hdr(), frame_id=7, cones=[NS(cls="blue", conf=0.9, x1=1, y1=2, x2=3, y2=4, dist_m=2.5, x=2.4, y=0.5, z=0.0, depth_ok=True),
                                               NS(cls="yellow", conf=0.8, x1=1, y1=2, x2=3, y2=4, dist_m=float("nan"), x=float("nan"), y=float("nan"), z=0.0, depth_ok=False)],
             inference_ms=20.0, depth_valid=True, calibration_verified=False)
    c.on_detections(det)
    assert out == []                                    # detections alone are never sent (no matching image yet)
    c.on_image(NS(header=hdr("7"), data=buf.tobytes()))
    assert [e["topic"] for e in out] == ["camera", "detections"]
    assert out[0]["frame_id"] == out[1]["frame_id"] == 7 and out[0]["data"]["width"] == 200
    d = out[1]["data"]["items"]
    assert d[1]["dist_m"] is None and d[0]["bbox"] == [1, 2, 3, 4]
    json.dumps(out, allow_nan=False)                    # strict JSON: no NaN reaches the browser
    assert out[0]["mode"] == "LIVE"


def test_map_invalid_when_plan_stale():
    out = []
    t = [0.0]
    c = GatewayCore(out.append, "REPLAY", clock=lambda: t[0])
    m = NS(header=hdr(), frame_id=3, valid=True, status="4_PAIRS", corridor_width_m=3.0,
           cones=[NS(cls="blue", x=1.0, y=1.5)], centerline=[NS(x=0.0, y=0.0), NS(x=5.0, y=0.0)])
    c.on_map(m)
    assert out[-1]["valid"] is False and out[-1]["mode"] == "REPLAY"      # no plan yet -> INVALID, not LIVE
    plan = NS(header=hdr(), frame_id=3, path=[NS(x=0.0, y=0.0), NS(x=5.0, y=0.0)], path_valid=True, stop_valid=True,
              stop_target=NS(x=4.0, y=0.0), nearest_cone_m=4.2, available_m=4.05, required_stop_m=1.0, decision="CRUISE",
              target_speed_mps=1.0)
    c.on_plan(plan)
    t[0] = 0.2
    c.on_map(m)
    assert out[-1]["topic"] == "map" and out[-1]["valid"] is True and out[-1]["data"]["stop_target"]["x"] == 4.0
    t[0] = 3.0                                                              # plan is now 2 s old
    c.on_map(m)
    assert out[-1]["valid"] is False


def test_telemetry_panels_and_safety():
    out = []
    t = [0.0]
    c = GatewayCore(out.append, "LIVE", clock=lambda: t[0])
    c.on_ctrl(NS(steer_deg_cmd=3.2, target_speed_mps=1.0, brake_pct=0.0, lookahead_valid=False, lookahead_x=0.0, lookahead_y=0.0,
                 wheelbase_m=1.0, lookahead_m=1.0, alpha_rad=0.1))
    c.on_telemetry(NS(header=hdr(), speed_mps=0.5, steer_deg=1.0, steer_valid=True, steer_pwm_ok=True, dac_v=1.2,
                      state_name="ARMED", en=True, brk=False))
    names = {e["topic"]: e for e in out}
    assert names["steering"]["data"]["target_deg"] == 3.2 and "OBSERVE" in names["steering"]["data"]["state"]
    assert abs(names["drive"]["data"]["actual_kmh"] - 1.8) < 1e-9 and "1.20 V" in names["drive"]["data"]["cmd"]
    assert names["brake"]["data"]["actual"] == 0.0
    st = NS(id=1, name="x", state="READY", detail="d")
    c.on_safety(NS(header=hdr(), state="DISARMED", vcu_heartbeat_ok=True, arm_allowed=False, stop_latched=False,
                   reasons=["a"], startup_ready=False, stages=[st]))
    topics = [e["topic"] for e in out]
    assert "safety" in topics and "startup" in topics


def test_throttle_cap_in_drive_panel_comes_from_config_not_a_literal():
    # The cap text must reflect whatever this gateway was configured with, not a baked-in "2.00 V".
    out = []
    c = GatewayCore(out.append, "LIVE", clock=lambda: 0.0, throttle_cap_v=0.7)
    c.on_telemetry(NS(header=hdr(), speed_mps=0.0, steer_deg=0.0, steer_valid=True, steer_pwm_ok=True, dac_v=0.5,
                      state_name="ARMED", en=True, brk=False))
    cmd = next(e for e in out if e["topic"] == "drive")["data"]["cmd"]
    assert "0.70 V" in cmd and "2.00 V" not in cmd


def test_websocket_roundtrip_and_request_whitelist():
    import websockets.sync.client as wsc
    got = []
    srv = GatewayServer("127.0.0.1", 18765, lambda a, m: got.append(a))
    srv.start()
    with wsc.connect("ws://127.0.0.1:18765") as ws:
        time.sleep(0.2)
        srv.emit({"topic": "odom", "stamp": 1.0, "mode": "LIVE", "valid": True, "source": "t", "data": {"x": 1.0}})
        assert json.loads(ws.recv(timeout=2))["data"]["x"] == 1.0
        ws.send(json.dumps({"type": "request", "action": "arm"}))
        ws.send(json.dumps({"type": "request", "action": "drive_forward_fast"}))   # not allowed
        ws.send("not json")
        ws.send(json.dumps({"type": "request", "action": "stop"}))
        time.sleep(0.3)
    assert got == ["arm", "stop"]
