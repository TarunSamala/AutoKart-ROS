import json
import time
from dataclasses import replace

import numpy as np
import pytest

from autokart_perception import default_config
from autokart_perception.boundaries import estimate_boundaries
from autokart_perception.camera import CameraError
from autokart_perception.live_mission import (CONENET_CLASSES, CONENET_MODEL,
    CONENET_SHA256, CONENET_V2_MODEL, CONENET_V2_SHA256,
    YOLO26S_MODEL, YOLO26S_SHA256,
    LiveSession, PublishedFrame, SnapshotStore, live_detector)
from autokart_perception.live_render import depth_preview, project_base_to_pixel
from autokart_perception.mount import mount_from_config
from autokart_perception.recording import Replay
from autokart_perception.synthetic import Synthetic, synthetic_cones
from autokart_perception.types import Intrinsics


def test_live_motion_is_provisional_and_stale_frame_resets_reference():
    source = Synthetic()
    session = LiveSession(mode="live")
    first = source.read()
    first.source = "LIVE"
    initial = session.process_one(first).meta
    assert initial["camera_motion"]["status"] == "MOTION_REFERENCE_UNAVAILABLE"
    assert initial["camera_motion"]["frame"] == "CAMERA_OPTICAL"
    second = source.read()
    second.source = "LIVE"
    following = session.process_one(second).meta
    assert following["camera_motion"]["previous_timestamp_ns"] == first.timestamp_ns
    assert following["camera_motion"]["current_timestamp_ns"] == second.timestamp_ns
    assert following["camera_motion"]["qualified"] is False
    assert following["pose_status"] == "UNAVAILABLE"
    assert following["geometry"]["delaunay_edges"] == []
    assert following["vehicle_authorized"] is False

    stale = source.read()
    stale.source = "LIVE"
    stale.timestamp_ns -= 1_000_000_000
    assert session.process_one(stale).meta["camera_motion"] is None
    recovered = source.read()
    recovered.source = "LIVE"
    assert session.process_one(recovered).meta["camera_motion"]["status"] == "MOTION_REFERENCE_UNAVAILABLE"


def test_out_of_order_frame_cannot_seed_next_motion_reference():
    source = Synthetic()
    session = LiveSession(mode="live")
    statuses = []
    for sequence in (1, 2, 1, 4):
        frame = source.read()
        frame.source = "LIVE"
        frame.rgb_sequence = frame.depth_sequence = sequence
        statuses.append(session.process_one(frame).meta)
    assert statuses[2]["perception"]["status"] == "FRAME_OUT_OF_ORDER"
    assert statuses[2]["camera_motion"] is None
    assert statuses[3]["camera_motion"]["status"] == "MOTION_REFERENCE_UNAVAILABLE"
    assert statuses[3]["camera_motion"]["previous_timestamp_ns"] == 0
    assert statuses[3]["geometry"]["delaunay_edges"] == []


def test_live_nominal_projection_is_diagnostic_only_without_global_pose(tmp_path):
    from autokart_perception.types import Detection

    class FixedDetector:
        verified_checkpoint = True
        version = "fixed-test"
        backend = "test"
        latency_ms = 0.0
        model_sha256 = "test"
        display_name = "fixed test"
        supported_classes = ("orange",)
        def detect(self, _image):
            return [Detection((300, 200, 340, 300), "orange", .9, 7)]

    frame = Synthetic().read()
    frame.source = "LIVE"
    frame.depth[:] = 2000
    detector = FixedDetector()
    session = LiveSession(mode="live", detector=detector,
                          record_path=tmp_path / 'live-recording', max_record_frames=1)
    meta = session.process_one(frame).meta
    assert meta['pose_status']=='UNAVAILABLE'
    assert meta['base_frame']=='camera_ground_nominal'
    assert len(meta['candidate_observations'])==1
    assert meta['observations']==[]
    assert meta['geometry']['delaunay_edges']==[]
    assert meta['geometry']['gabriel_edges']==[]
    assert meta['geometry']['gates']==[]
    assert meta['geometry']['centerline']==[]
    assert meta['perception']['diagnostics']['cone_details'][0]['validation_reasons']==[
        'DEPTH_SCALE_UNVERIFIED', 'MOUNT_TO_BASE_LINK_UNVERIFIED']
    assert meta['vehicle_authorized'] is False
    replay=LiveSession(mode='replay').process_one(Replay(tmp_path / 'live-recording').read()).meta
    assert replay['base_frame']=='camera_ground_nominal'
    assert json.loads(json.dumps(replay['candidate_observations']))==json.loads(
        json.dumps(meta['candidate_observations']))
    assert replay['observations']==[]
    assert replay['geometry']['delaunay_edges']==[]
    assert replay['geometry']['gates']==[]
    assert replay['vehicle_authorized'] is False


def test_clipped_live_body_observation_is_visible_but_never_enters_geometry(tmp_path):
    from autokart_perception.types import Detection

    class ClippedDetector:
        verified_checkpoint=True; version='clipped-test'; backend='test'; latency_ms=0.0
        model_sha256='test'; display_name='clipped'; supported_classes=('orange',)
        def detect(self, _image):
            return [Detection((80, 210, 220, 480), 'orange', .9, 3)]

    frame=Synthetic().read(); frame.source='LIVE'; frame.depth[:]=800
    session=LiveSession(mode='live', detector=ClippedDetector(),
                        record_path=tmp_path / 'capture', max_record_frames=1)
    meta=session.process_one(frame).meta
    assert meta['candidate_observations']==[]
    assert len(meta['provisional_3d_observations'])==1
    body=meta['provisional_3d_observations'][0]
    assert body['position_kind']=='PROVISIONAL_3D_POSITION'
    assert body['ground_contact_status']=='UNAVAILABLE_CONE_FOOT_CLIPPED'
    assert body['uncertainty_m']>0
    assert meta['observations']==[]
    assert meta['geometry']['delaunay_edges']==[]
    assert meta['vehicle_authorized'] is False
    replay=LiveSession(mode='replay').process_one(Replay(tmp_path / 'capture').read()).meta
    assert json.loads(json.dumps(replay['provisional_3d_observations']))==json.loads(
        json.dumps(meta['provisional_3d_observations']))
    assert replay['geometry']['delaunay_edges']==[]


def test_live_zero_detection_and_missing_intrinsics_have_no_candidate_geometry():
    from autokart_perception.types import Detection

    class EmptyDetector:
        verified_checkpoint=True; version='empty'; backend='test'; latency_ms=0.0
        model_sha256='test'; display_name='empty'; supported_classes=('orange',)
        def detect(self, _image): return []

    frame=Synthetic().read();frame.source='LIVE'
    session=LiveSession(mode='live',detector=EmptyDetector())
    empty=session.process_one(frame).meta
    assert empty['candidate_observations']==[] and empty['geometry']['delaunay_edges']==[]
    frame=Synthetic().read();frame.source='LIVE';frame.intrinsics=Intrinsics(0,0,0,0)
    invalid=session.process_one(frame).meta
    assert invalid['candidate_observations']==[]
    assert invalid['vehicle_authorized'] is False


def test_raw16_depth_mask_and_measured_coverage_shape():
    depth = np.zeros((480, 640), dtype=np.uint16)
    depth.flat[:105_304] = 680
    image, stats = depth_preview(depth)
    assert stats["valid_pixels"] == 105_304
    assert stats["invalid_pixels"] == 201_896
    assert stats["valid_percent"] == pytest.approx(34.28, abs=.01)
    assert stats["median_raw"] == 680
    assert stats["raw_unit"] == "UNVERIFIED"
    assert np.all(image.reshape(-1, 3)[105_304:] == 0)
    assert np.any(image.reshape(-1, 3)[:105_304] != 0)
    with pytest.raises(ValueError, match="INVALID_RAW16_DEPTH"):
        depth_preview(depth.astype(">u2"))
    _, empty = depth_preview(np.zeros_like(depth))
    assert empty["valid_pixels"] == 0 and empty["median_raw"] is None


def test_project_base_point_uses_intrinsic_and_extrinsic_geometry():
    config = default_config()
    config["camera_mount"] = {**config["camera_mount"], "height_m": .2,
                              "pitch_down_deg": 0.0}
    matrix, _ = mount_from_config(config)
    intrinsics = Intrinsics(500, 500, 320, 240)
    assert project_base_to_pixel((2, 0, 0), matrix, intrinsics) == (320, 290)
    assert project_base_to_pixel((-2, 0, 0), matrix, intrinsics) is None
    assert project_base_to_pixel((float("nan"), 0, 0), matrix, intrinsics) is None


def test_single_snapshot_images_and_sdk_geometry_share_frame_id():
    session = LiveSession(mode="sim")
    published = session.process_one(Synthetic().read())
    current = session.store.current()
    assert published.frame_id == current["frame_id"]
    assert published.rgb_jpeg[:2] == b"\xff\xd8"
    assert published.depth_jpeg[:2] == b"\xff\xd8"
    assert session.store.image(current["frame_id"], "rgb") == published.rgb_jpeg
    assert session.store.image(current["frame_id"], "depth") == published.depth_jpeg
    assert current["source"] == "SIMULATED"
    assert current["rgb_depth_pair"] == "SIMULATED_PAIR"
    assert current["geometry"]["delaunay_edges"]
    assert current["geometry"]["gabriel_edges"]
    assert current["geometry"]["gates"]
    assert current["geometry"]["centerline"]
    assert current["planning"]["path_valid"] is True
    assert current["control_candidate"]["valid"] is True
    assert current["pose_status"] == "SIMULATED"
    assert current["vehicle_authorized"] is False
    json.dumps(current, allow_nan=False)


def test_fault_and_stale_clear_current_geometry():
    store = SnapshotStore("LIVE", maximum_age_ms=150)
    meta = {"frame_id": "live-1", "geometry": {"delaunay_edges": [[1, 2]]},
            "candidate_observations": [{"id": 1}],
            "ready": True, "vehicle_authorized": False}
    store.publish(PublishedFrame("live-1", meta, b"rgb", b"depth"))
    assert store.current()["frame_id"] == "live-1"
    store.fault("CAMERA_DISCONNECTED")
    fault = store.current()
    assert fault["frame_id"] is None and fault["geometry"]["delaunay_edges"] == []
    assert fault["candidate_observations"] == []
    assert fault["control_candidate"]["valid"] is False
    store.publish(PublishedFrame("live-2", {**meta, "frame_id": "live-2"}, b"rgb", b"depth"))
    store.last_good_ns = time.monotonic_ns() - 200_000_000
    stale = store.current()
    assert stale["status"] == "STALE" and stale["frame_id"] is None
    assert stale["planning"]["path_valid"] is False


def test_recorded_frame_replay_runs_same_geometry_implementations(tmp_path):
    recording = tmp_path / "capture"
    live = LiveSession(mode="sim", record_path=recording, max_record_frames=1)
    original = live.process_one(Synthetic().read()).meta
    replay_frame = Replay(recording).read()
    replay = LiveSession(mode="replay")
    reproduced = replay.process_one(replay_frame).meta
    assert reproduced["source"] == "REPLAY"
    assert reproduced["rgb_depth_pair"] == "RECORDED_PAIR"
    for key in ("delaunay_edges", "delaunay_triangles", "gabriel_edges",
                "boundaries", "gates", "centerline"):
        assert reproduced["geometry"][key] == original["geometry"][key]
    assert reproduced["vehicle_authorized"] is False


def test_source_mismatch_is_rejected():
    with pytest.raises(ValueError, match="SOURCE_MISMATCH"):
        LiveSession(mode="live").process_one(Synthetic().read())


@pytest.mark.skipif(not CONENET_MODEL.is_file(), reason="trained ConeNet v1 model unavailable")
def test_default_live_model_and_coneslayer_rollback_identity():
    conenet = live_detector("conenet-v1")
    assert tuple(conenet.classes) == CONENET_CLASSES
    assert conenet.supported_classes == CONENET_CLASSES
    assert conenet.model_sha256 == CONENET_SHA256
    assert conenet.stage == "experimental" and conenet.verified_checkpoint is False
    rows = np.zeros((5, 9), dtype=np.float32)
    for klass, row in enumerate(rows):
        row[:4] = (80 + 110 * klass, 320, 50, 80)
        row[4 + klass] = .9 - .1 * klass
    detections = conenet.postprocess(rows, (640, 480, 1.0, 0, 80))
    assert [item.label for item in detections] == list(CONENET_CLASSES)
    assert detections[0].box == (55, 200, 105, 280)
    assert detections[3].label == "unknown"
    assert conenet.postprocess(np.empty((0, 9), dtype=np.float32), (640, 480, 1.0, 0, 80)) == []
    assert conenet.postprocess(np.full((1, 9), np.nan, dtype=np.float32),
                               (640, 480, 1.0, 0, 80)) == []
    known = synthetic_cones()
    unknown = replace(known[0], id=9, label="unknown", xy=(10.0, 1.5))
    boundaries = estimate_boundaries([*known, unknown], default_config())
    assert boundaries.mode == "COLOR"
    assert 9 not in boundaries.left_ids + boundaries.right_ids
    rollback = live_detector("coneslayer")
    assert rollback.supported_classes == ("orange",)
    assert rollback.verified_checkpoint is True


@pytest.mark.skipif(not CONENET_V2_MODEL.is_file(), reason="exported ConeNet v2 model unavailable")
def test_conenet_v2_live_selection_executes_exported_model():
    detector = live_detector("conenet-v2")
    assert detector.display_name == "ConeNet v2"
    assert detector.model_sha256 == CONENET_V2_SHA256
    assert tuple(detector.classes) == CONENET_CLASSES
    assert detector.stage == "experimental" and detector.verified_checkpoint is False
    rows, geometry = detector.infer(np.zeros((480, 640, 3), dtype=np.uint8))
    assert rows.shape == (8400, 9)
    assert geometry == (640, 480, 1.0, 0, 80)


def test_yolo26s_live_selection_and_end_to_end_box_contract():
    detector = live_detector("yolo26s")
    assert detector.model_sha256 == YOLO26S_SHA256
    assert detector.display_name == "YOLO26s"
    assert detector.supported_classes == CONENET_CLASSES
    assert detector.stage == "experimental" and detector.verified_checkpoint is False
    rows = np.array([[10 + i * 110, 100, 60 + i * 110, 180,
                      .9 - i * .1, i] for i in range(5)], dtype=np.float32)
    detections = detector.postprocess(rows, (640, 480, 1.0, 0, 80))
    assert [item.label for item in detections] == list(CONENET_CLASSES)
    assert detections[0].box == (10, 20, 60, 100)
    assert detections[3].label == "unknown"
    assert detector.postprocess(np.zeros((0, 6), dtype=np.float32),
                                (640, 480, 1.0, 0, 80)) == []
    bad = np.array([[0, 0, 40, 50, .9, 7], [0, 0, 40, 50, .9, np.nan],
                    [50, 50, 10, 10, .9, 0]], dtype=np.float32)
    assert detector.postprocess(bad, (640, 480, 1.0, 0, 80)) == []
    rows, geometry = detector.infer(np.zeros((480, 640, 3), dtype=np.uint8))
    assert rows.shape == (300, 6)
    assert geometry == (640, 480, 1.0, 0, 80)
    assert detector.postprocess(rows, geometry) == []


def test_replay_cone_movement_and_removal_update_current_geometry():
    session = LiveSession(mode="replay")
    cones = synthetic_cones()
    base = Synthetic().read()

    def replay_frame(items, sequence):
        return replace(base, source="REPLAY", timestamp_ns=base.timestamp_ns + sequence,
                       rgb_sequence=sequence, depth_sequence=sequence,
                           calibration={"recorded_source": "SYNTHETIC",
                                        "recorded_cones": [vars(item) for item in items],
                                    "recorded_detections": [], "recorded_model_version": "fixture"})

    first = session.process_one(replay_frame(cones, 1)).meta
    moved = list(cones)
    moved[0] = replace(moved[0], xy=(2.0, 2.3))
    second = session.process_one(replay_frame(moved, 2)).meta
    assert first["geometry"]["delaunay_edges"]
    assert first["geometry"]["gabriel_edges"]
    assert second["observations"][0]["xy"] == (2.0, 2.3)
    assert second["geometry"] != first["geometry"]
    removed = session.process_one(replay_frame(moved[1:], 3)).meta
    assert all(item["id"] != 0 for item in removed["observations"])
    assert removed["geometry"] != second["geometry"]


def test_transient_camera_startup_keeps_one_owner_then_recovers():
    frame = replace(Synthetic().read(), source="LIVE")
    factories = []

    class StartingCamera:
        def __init__(self):
            self.reads = 0

        def read(self):
            self.reads += 1
            if self.reads == 1:
                raise CameraError("CALIBRATION_UNAVAILABLE")
            return frame

        def close(self):
            pass

    def source_factory():
        camera = StartingCamera()
        factories.append(camera)
        return camera

    session = LiveSession(mode="live", source_factory=source_factory, frame_limit=1)
    session.run()
    assert len(factories) == 1
    assert factories[0].reads == 2
    assert session.processed == 1
    assert session.store.frames[0].meta["vehicle_authorized"] is False


def test_disconnected_live_camera_reopens_without_reusing_old_image():
    factories = []

    class Camera:
        def __init__(self, disconnect):
            self.disconnect = disconnect
            self.closed = False

        def read(self):
            if self.disconnect:
                raise CameraError("CAMERA_DISCONNECTED")
            return replace(Synthetic().read(), source="LIVE")

        def close(self):
            self.closed = True

    def source_factory():
        camera = Camera(disconnect=not factories)
        factories.append(camera)
        return camera

    session = LiveSession(mode="live", source_factory=source_factory, frame_limit=1)
    session.run()
    assert len(factories) == 2
    assert all(camera.closed for camera in factories)
    assert session.processed == 1
    current = session.store.frames[-1]
    assert current.frame_id == current.meta["frame_id"]
    assert current.meta["camera_sequence"] == current.meta["depth_sequence"]
    assert current.meta["vehicle_authorized"] is False
