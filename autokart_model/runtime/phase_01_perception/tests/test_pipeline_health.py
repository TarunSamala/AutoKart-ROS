"""Per-hop pipeline health: GET /api/v1/diagnostics/pipeline."""

import json
import threading
import time
import urllib.error
import urllib.request
from dataclasses import replace

import pytest

from autokart_perception import live_mission
from autokart_perception.camera import CameraError
from autokart_perception.live_mission import (HOP_STATES, PIPELINE_DIAGNOSTICS_PATH, LiveSession,
                                              build_server)
from autokart_perception.synthetic import Synthetic
from autokart_perception.types import Detection

HOP_NAMES = ["sdk_capture", "detector", "geometry", "http"]


def _reject_constant(name):
    raise AssertionError(f"non-JSON constant {name}")


def _get(port, path, token=None):
    request = urllib.request.Request(f"http://127.0.0.1:{port}{path}")
    if token:
        request.add_header("Authorization", "Bearer " + token)
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            body = response.read()
            code = response.status
    except urllib.error.HTTPError as error:
        body, code = error.read(), error.code
    return code, json.loads(body, parse_constant=_reject_constant)


class Running:
    """Session + HTTP server on an ephemeral port, torn down after the test."""

    def __init__(self, session, static_root, token=None):
        self.session = session
        self.server = build_server(session, host="127.0.0.1", port=0,
                                   static_root=static_root, token=token)
        self.port = self.server.server_address[1]
        self.thread = threading.Thread(target=self.server.serve_forever,
                                       kwargs={"poll_interval": .05}, daemon=True)

    def __enter__(self):
        self.session.start()
        self.thread.start()
        return self

    def __exit__(self, *_):
        self.server.shutdown()
        self.server.server_close()
        self.session.close()

    def pipeline(self, token=None):
        return _get(self.port, PIPELINE_DIAGNOSTICS_PATH, token)


def _wait(predicate, timeout=6.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(.05)
    raise AssertionError("condition not reached")


def _hops(report):
    return {hop["name"]: hop for hop in report["hops"]}


def _assert_shape(report):
    assert report["schema_version"] == 1
    assert set(report) == {"schema_version", "service", "hops", "overall", "vehicle_authorized"}
    assert set(report["service"]) == {"name", "mode", "source", "pid", "started_at_ns",
                                      "uptime_s", "camera_owner", "detector", "model_sha256"}
    assert report["service"]["name"] == "autokart_perception.live_mission"
    assert [hop["name"] for hop in report["hops"]] == HOP_NAMES
    for hop in report["hops"]:
        assert hop["state"] in HOP_STATES
    assert report["overall"] in HOP_STATES and report["overall"] != "NOT_APPLICABLE"
    assert report["vehicle_authorized"] is False
    json.dumps(report, allow_nan=False)


class FixedDetector:
    verified_checkpoint = True
    version = "fixed-test"
    backend = "test"
    latency_ms = 1.5
    model_sha256 = "sha-test"
    display_name = "fixed test"
    supported_classes = ("orange",)

    def detect(self, _image):
        return [Detection((300, 200, 340, 300), "orange", .9, 7)]


class FailingDetector(FixedDetector):
    def detect(self, _image):
        raise RuntimeError("ONNX_RUNTIME_FAULT")


def test_sim_pipeline_endpoint_shape_vocabulary_and_source(tmp_path):
    with Running(LiveSession(mode="sim"), tmp_path) as running:
        _wait(lambda: running.session.processed >= 2)
        assert _get(running.port, "/api/snapshot")[0] == 200
        frame_id = _get(running.port, "/api/snapshot")[1]["frame_id"]
        with urllib.request.urlopen(f"http://127.0.0.1:{running.port}/api/image/{frame_id}/rgb",
                                    timeout=5) as response:
            assert response.read()[:2] == b"\xff\xd8"
        assert _get(running.port, "/api/image/expired-1/rgb")[0] == 404
        code, report = running.pipeline()
    assert code == 200
    _assert_shape(report)
    hops = _hops(report)
    assert report["service"]["mode"] == "sim"
    assert report["service"]["source"] == "SIMULATED"
    assert report["service"]["camera_owner"] is False
    assert report["service"]["detector"] is None
    assert hops["sdk_capture"]["source"] == "SIMULATED"
    assert hops["sdk_capture"]["uses_vendor_sdk"] is False
    assert hops["sdk_capture"]["state"] == "OK"
    assert hops["sdk_capture"]["frames_received"] >= 2
    assert hops["sdk_capture"]["frame_age_ms"] >= 0
    assert hops["detector"]["state"] == "NOT_APPLICABLE"
    assert hops["detector"]["inference_count"] == 0
    assert hops["geometry"]["state"] == "OK"
    assert hops["geometry"]["calibration_applies"] is False
    assert hops["geometry"]["counts"]["qualified_observations"] == 6
    assert hops["http"]["snapshots_served"] == 2
    assert hops["http"]["image_requests"] == 2
    assert hops["http"]["image_not_found"] == 1
    assert hops["http"]["last_client_request_age_ms"] >= 0
    assert report["overall"] == "OK"
    assert "LIVE" not in json.dumps(report)


def test_live_without_camera_reports_sdk_error_keeps_server_and_retries(tmp_path):
    opened = []

    class DisconnectedCamera:
        def read(self):
            raise CameraError("CAMERA_DISCONNECTED")

        def close(self):
            pass

    def factory():
        opened.append(1)
        return DisconnectedCamera()

    session = LiveSession(mode="live", detector=FixedDetector(), source_factory=factory)
    with Running(session, tmp_path) as running:
        _wait(lambda: len(opened) >= 2)  # closed and reopened after the 1 s back-off
        code, report = running.pipeline()
        assert _get(running.port, "/health") == (200, {"server": "RUNNING", "source": "LIVE",
                                                       "vehicle_authorized": False})
        snapshot = _get(running.port, "/api/snapshot")[1]
        assert running.session.worker.is_alive()
    assert code == 200
    _assert_shape(report)
    hops = _hops(report)
    capture = hops["sdk_capture"]
    assert capture["state"] == "SDK_ERROR"
    assert capture["last_error"] == "CAMERA_DISCONNECTED"
    assert capture["reason"] == "CAMERA_DISCONNECTED"
    assert capture["error_counts"]["CAMERA_DISCONNECTED"] >= 2
    assert capture["source_opens"] >= 2
    assert capture["frames_received"] == 0
    assert capture["last_frame_stamp_ns"] is None
    assert capture["worker_alive"] is True
    assert hops["detector"]["state"] == "NO_FRAME"
    assert hops["geometry"]["state"] == "NO_FRAME"
    assert report["overall"] == "SDK_ERROR"
    assert report["service"]["source"] == "LIVE"
    assert report["service"]["camera_owner"] is False  # not an HP60C SDK session
    # Never substitute synthetic frames in live mode.
    assert snapshot["source"] == "LIVE" and snapshot["frame_id"] is None
    assert snapshot["status"] == "SENSOR_FAULT"
    assert "SIMULATED" not in json.dumps(report) and "SYNTHETIC" not in json.dumps(report)
    assert running.session.processed == 0


def test_live_real_hp60c_adapter_error_code_is_reported(tmp_path):
    """Real HP60C class: missing vendor configuration fails before any SDK call."""
    session = LiveSession(mode="live", detector=FixedDetector(),
                          camera_configuration=tmp_path / "missing_config.json")
    with Running(session, tmp_path) as running:
        report = _wait(lambda: (lambda r: r if _hops(r)["sdk_capture"]["last_error"] else None)(
            running.pipeline()[1]))
    capture = _hops(report)["sdk_capture"]
    assert capture["state"] == "SDK_ERROR"
    assert capture["last_error"] in {"SDK_NOT_PREPARED", "SDK_ADAPTER_NOT_BUILT"}
    assert capture["uses_vendor_sdk"] is True
    assert capture["frames_received"] == 0
    assert report["service"]["camera_owner"] is False


def test_transient_startup_code_is_starting_then_sdk_error(monkeypatch):
    session = LiveSession(mode="live", detector=FixedDetector())
    session.health.note_capture_error("FRAME_TIMEOUT", "CameraError:FRAME_TIMEOUT")
    capture = _hops(session.pipeline_health())["sdk_capture"]
    assert capture["state"] == "STARTING" and capture["last_error"] == "FRAME_TIMEOUT"
    monkeypatch.setattr(live_mission, "STARTUP_GRACE_S", 0.0)
    report = session.pipeline_health()
    assert _hops(report)["sdk_capture"]["state"] == "SDK_ERROR"
    assert report["overall"] == "SDK_ERROR"
    session.health.note_capture_error("STALE_FRAME", "CameraError:STALE_FRAME")
    assert _hops(session.pipeline_health())["sdk_capture"]["state"] == "STALE"


def test_live_frames_report_calibration_unverified_and_detector_metrics():
    session = LiveSession(mode="live", detector=FixedDetector())
    frame = replace(Synthetic().read(), source="LIVE")
    frame.depth[:] = 2000
    session.health.note_frame(frame)
    session.process_one(frame)
    report = session.pipeline_health()
    hops = _hops(report)
    assert hops["sdk_capture"]["state"] == "OK"
    assert hops["sdk_capture"]["source"] == "LIVE"
    assert hops["detector"]["state"] == "OK"
    assert hops["detector"]["inference_count"] == 1
    assert hops["detector"]["last_latency_ms"] == 1.5
    assert hops["detector"]["model_sha256"] == "sha-test"
    assert hops["geometry"]["state"] == "CALIBRATION_UNVERIFIED"
    assert hops["geometry"]["calibration_applies"] is True
    assert hops["geometry"]["counts"]["qualified_observations"] == 0
    assert "CALIBRATION_UNVERIFIED" in hops["geometry"]["reason_codes"]
    assert report["overall"] == "CALIBRATION_UNVERIFIED"
    assert report["service"]["detector"] == "fixed test"
    # Frame stops arriving: capture and downstream go STALE without old data.
    session.health.last_frame_stamp_ns -= 10_000_000_000
    session.health.last_processed_ns -= 10_000_000_000
    session.health.last_inference_ns -= 10_000_000_000
    hops = _hops(session.pipeline_health())
    assert hops["sdk_capture"]["state"] == "STALE"
    assert hops["detector"]["state"] == "STALE"
    assert hops["geometry"]["state"] == "STALE"


def test_inference_failure_degrades_detector_hop():
    session = LiveSession(mode="live", detector=FailingDetector())
    frame = replace(Synthetic().read(), source="LIVE")
    session.health.note_frame(frame)
    session.process_one(frame)
    detector = _hops(session.pipeline_health())["detector"]
    assert detector["state"] == "DEGRADED"
    assert detector["last_error"].startswith("INFERENCE_FAILURE")
    assert detector["inference_failures"] == 1 and detector["inference_count"] == 0


def test_replay_complete_reports_no_frame_with_replay_source(tmp_path):
    recording = tmp_path / "capture"
    recorder = LiveSession(mode="sim", record_path=recording, max_record_frames=1)
    recorder.process_one(Synthetic().read())
    from autokart_perception.recording import Replay
    session = LiveSession(mode="replay", source_factory=lambda: Replay(recording))
    session.start()
    _wait(lambda: not session.worker.is_alive())
    report = session.pipeline_health()
    session.close()
    hops = _hops(report)
    assert report["service"]["source"] == "REPLAY"
    assert report["service"]["camera_owner"] is False
    assert hops["sdk_capture"]["state"] == "NO_FRAME"
    assert hops["sdk_capture"]["worker_exit_reason"] == "REPLAY_COMPLETE"
    assert hops["sdk_capture"]["frames_received"] == 1
    assert hops["sdk_capture"]["frame_age_basis"] == "HOST_INGEST_MONOTONIC"
    assert hops["detector"]["state"] == "NOT_APPLICABLE"
    assert "SIMULATED" not in json.dumps(report["service"])


def test_pipeline_route_follows_api_token_rule(tmp_path):
    token = "t" * 32
    with Running(LiveSession(mode="sim"), tmp_path, token=token) as running:
        assert running.pipeline() == (401, {"error": "AUTH_REQUIRED"})
        assert _get(running.port, "/api/v1/unknown", token)[0] == 404
        code, report = running.pipeline(token)
    assert code == 200
    assert _hops(report)["http"]["unauthorized_requests"] == 1
    assert _hops(report)["http"]["diagnostics_served"] == 1


def test_non_finite_counters_are_serialised_as_null():
    session = LiveSession(mode="sim")
    session.health.last_inference_ms = float("nan")
    session.health.sdk_callback_fps = float("inf")
    report = session.pipeline_health()
    text = json.dumps(report, allow_nan=False)
    json.loads(text, parse_constant=_reject_constant)
    assert _hops(report)["detector"]["last_latency_ms"] is None
    assert _hops(report)["sdk_capture"]["sdk_callback_fps"] is None
