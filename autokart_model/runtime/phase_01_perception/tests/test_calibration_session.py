import hashlib
import json
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

from autokart_perception import calibration_session as cs
from autokart_perception.calibration_store import verify_artifact
from autokart_perception.mount import mount_from_config
from autokart_perception.pipeline import Pipeline
from autokart_perception.recording import Recorder
from autokart_perception.synthetic import Synthetic
from autokart_perception.types import Detection

CONFIG = yaml.safe_load((Path(__file__).parents[1] / 'config/perception.yaml').read_text())
HP60C_INTR = {'fx': 585.27, 'fy': 585.02, 'cx': 335.98, 'cy': 239.96}
MEASURED = {'x_forward_m': 0.0, 'y_left_m': 0.0, 'height_m': 0.30, 'pitch_down_deg': 4.0, 'roll_deg': 0.0, 'yaw_left_deg': 0.0}
TRUE = {'height_m': .32, 'pitch_down_deg': 6.0}
FACTOR = 1.05
DISTANCES = [1.0, 2.0, 3.0]


def test_plan_reports_half_metre_row_out_of_view_and_proposes_alternatives():
    report = cs.plan(HP60C_INTR, MEASURED, CONFIG)
    missing = {(c['distance_m'], c['region']) for c in report['out_of_view_cells']}
    assert {(0.5, r) for r in cs.REGIONS} <= missing
    assert all(p['status'] == 'PROPOSED_REQUIRES_APPROVAL' and p['distance_m'] > 0.5 for p in report['proposed_alternatives'])
    assert {(c['distance_m'], c['region']) for c in report['visible_cells']} >= {(d, r) for d in DISTANCES for r in cs.REGIONS}


def test_template_has_no_coordinates_and_is_refused_until_measured(tmp_path):
    path = tmp_path / 'stations.json'
    cs.template(cs.plan(HP60C_INTR, MEASURED, CONFIG), path, approved_distances=[0.7, 1.0, 2.0, 3.0])
    doc = json.loads(path.read_text())
    assert all(c['x_m'] is None for s in doc['stations'] for c in s['cones'])
    with pytest.raises(ValueError, match='CONE_NOT_MEASURED.*ALTERNATIVE_GRID_NOT_APPROVED|ALTERNATIVE_GRID_NOT_APPROVED|CONE_NOT_MEASURED'):
        cs.load_stations(path)


def test_association_rejects_missing_and_ambiguous_matches():
    a = Detection((100, 100, 140, 200), 'orange', .9, 0)
    b = Detection((112, 100, 152, 205), 'orange', .9, 1)
    assert cs.associate([a], (120, 200))[0] is a
    assert cs.associate([a], (400, 300))[1] == 'NO_DETECTION_NEAR_SURVEYED_CONE'
    assert cs.associate([a, b], (126, 202))[1] == 'AMBIGUOUS_ASSOCIATION'
    assert cs.associate([a], None)[1] == 'PREDICTION_OUT_OF_VIEW'


def _scene(directory, points, source):
    truth = {**CONFIG, 'camera_to_kart': None, 'camera_mount': {**CONFIG['camera_mount'], **TRUE}}
    mount, _ = mount_from_config(truth)
    frame = Synthetic().read(); intr = frame.intrinsics; frame.depth[:] = 0
    labels = []
    for x, y in points:
        optical = np.linalg.inv(mount) @ np.array([x, y, 0.0, 1.0]); z = optical[2]
        u = intr.fx * optical[0] / z + intr.cx; foot = intr.fy * optical[1] / z + intr.cy
        box = [int(u - 15), int(foot - 60), int(u + 15), int(foot + 1)]
        frame.depth[max(0, box[1]):box[3], box[0]:box[2]] = int(round(z / (frame.depth_scale * FACTOR)))
        labels.append({'box': box, 'label': 'orange', 'xy_m': [x, y]})
    if source == 'LIVE':
        frame.source = 'LIVE'; frame.timestamp_ns = time.monotonic_ns(); frame.aligned = True
    Recorder(directory).write(Pipeline(None, truth, True).process(frame) if source != 'LIVE' else Pipeline(None, truth).process(frame))
    return {'000001': labels}


def _session(tmp_path, source, validate_offset=0.0):
    stations = {'schema_version': 1, 'operator': 'tester', 'instrument': 'tape ±2 mm', 'base_link_definition': 'below optical centre',
                'mount_measurement': {**MEASURED, 'uncertainty': {'translation_m': .005, 'angle_deg': .5}},
                'approved_grid': {'distances_m': DISTANCES, 'approved_by': 'reviewer', 'reason': '0.5 m foot outside vertical FOV'},
                'stations': []}
    capture = tmp_path / 'capture'; capture.mkdir()
    ann = {'fit': {'schema_version': 1, 'sessions': {}}, 'validate': {'schema_version': 1, 'sessions': {}}}
    plan = {'fit': [(d, y) for d in (1.6, 2.0, 2.6, 3.2) for y in (-0.6, 0.0, 0.6)],
            'validate': [(d, y) for d in DISTANCES for y in (-0.3 * d, 0.0, 0.3 * d)]}
    for role, points in plan.items():
        sid = f'{role}-1'
        labels = _scene(capture / sid, points, source)
        if role == 'validate' and validate_offset:
            for item in labels['000001']:
                item['xy_m'][0] += validate_offset
        ann[role]['sessions'][sid] = labels
        stations['stations'].append({'id': sid, 'role': role, 'cones': [{'region': 'center', 'x_m': x, 'y_m': y, 'sigma_m': .01} for x, y in points]})
    for role in ann:
        (capture / f'{role}_annotations.json').write_text(json.dumps(ann[role]))
    sp = tmp_path / 'stations.json'; sp.write_text(json.dumps(stations))
    (capture / 'capture_manifest.json').write_text(json.dumps({'stations_sha256': hashlib.sha256(sp.read_bytes()).hexdigest(),
        'device_identity': {'intrinsics': HP60C_INTR, 'usb': 'TEST'}, 'rejections': {'CONE_FOOT_CLIPPED': 2}}))
    return capture, sp


def test_non_live_session_fits_but_never_passes(tmp_path):
    capture, stations = _session(tmp_path, 'SYNTHETIC')
    report = cs.evaluate_session(capture, stations, .05, artifact_path=tmp_path / 'art.json')
    assert abs(report['fitted']['depth_scale_factor'] - FACTOR) < .01
    assert report['status'] == 'NON_LIVE_PASS' and report['blocker'] == 'SURVEY_REQUIRES_LIVE_HP60C_RECORDINGS'
    assert report['held_out_validation']['surveyed_cones'] == 9 and report['fit_station_count'] == 1
    assert report['capture_rejections'] == {'CONE_FOOT_CLIPPED': 2}
    assert not (tmp_path / 'art.json').exists()


def test_bad_held_out_measurements_fail_even_when_fit_is_good(tmp_path):
    capture, stations = _session(tmp_path, 'LIVE', validate_offset=0.2)
    report = cs.evaluate_session(capture, stations, .05, artifact_path=tmp_path / 'art.json')
    assert report['fitted']['train_error_m']['max'] < .05
    assert report['status'] == 'FAIL' and not (tmp_path / 'art.json').exists()


def test_live_tagged_fixture_passes_and_writes_bound_artifact(tmp_path):
    capture, stations = _session(tmp_path, 'LIVE')
    art = tmp_path / 'art.json'
    report = cs.evaluate_session(capture, stations, .05, artifact_path=art)
    assert report['status'] == 'PASS', report['held_out_validation']
    doc = json.loads(art.read_text())
    assert doc['device_identity']['intrinsics'] == HP60C_INTR and doc['approved_grid']['approved_by'] == 'reviewer'
    candidate = cs.mount_config(CONFIG, {**MEASURED, 'height_m': doc['camera_mount']['height_m'], 'pitch_down_deg': doc['camera_mount']['pitch_down_deg']})
    mount, _ = mount_from_config(candidate)
    assert verify_artifact(art, mount)[0] == 'MOUNT_VERIFIED'


def test_unapproved_alternative_grid_is_not_verified(tmp_path):
    capture, stations = _session(tmp_path, 'LIVE')
    art = tmp_path / 'art.json'
    cs.evaluate_session(capture, stations, .05, artifact_path=art)
    doc = json.loads(art.read_text()); doc['approved_grid'] = None
    from autokart_perception.calibration_store import _digest
    doc['sha256'] = _digest(doc); art.write_text(json.dumps(doc))
    mount, _ = mount_from_config(cs.mount_config(CONFIG, {**MEASURED, **{k: doc['camera_mount'][k] for k in ('height_m', 'pitch_down_deg')}}))
    assert 'CALIBRATION_ALTERNATIVE_GRID_NOT_APPROVED' in verify_artifact(art, mount)[1]


def test_stations_file_edited_after_capture_is_refused(tmp_path):
    capture, stations = _session(tmp_path, 'SYNTHETIC')
    doc = json.loads(stations.read_text()); doc['stations'][1]['cones'][0]['x_m'] += .01
    stations.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match='STATIONS_FILE_CHANGED_AFTER_CAPTURE'):
        cs.evaluate_session(capture, stations, .05)


def test_live_frame_from_a_different_camera_is_not_verified(tmp_path):
    capture, stations = _session(tmp_path, 'LIVE')
    art = tmp_path / 'art.json'
    cs.evaluate_session(capture, stations, .05, artifact_path=art)
    doc = json.loads(art.read_text())
    cfg = {**cs.mount_config(CONFIG, {**MEASURED, **{k: doc['camera_mount'][k] for k in ('height_m', 'pitch_down_deg')}}),
           'calibration_artifact': str(art)}
    pipe = Pipeline(None, cfg)
    assert pipe.mount_status == 'MOUNT_VERIFIED'
    frame = Synthetic().read(); frame.source = 'LIVE'; frame.timestamp_ns = time.monotonic_ns(); frame.aligned = True
    from autokart_perception.types import Intrinsics
    frame.intrinsics = Intrinsics(**HP60C_INTR)
    assert pipe.process(frame).diagnostics['calibration_status'] == 'MOUNT_VERIFIED'
    other = Synthetic().read(); other.source = 'LIVE'; other.timestamp_ns = time.monotonic_ns(); other.aligned = True
    other.intrinsics = Intrinsics(**{**HP60C_INTR, 'fx': HP60C_INTR['fx'] + 5})
    d = pipe.process(other).diagnostics
    assert d['calibration_status'] == 'MOUNT_CONFIGURED_UNVERIFIED' and 'CALIBRATION_CAMERA_MISMATCH' in d['calibration_reasons']


def test_capture_requires_complete_cone_foot_with_margin():
    assert cs.foot_complete((300, 300, 340, 470))
    assert not cs.foot_complete((300, 300, 340, 479))
    assert not cs.foot_complete((300, 300, 340, 477))
    assert not cs.foot_complete((1, 300, 40, 400)) and not cs.foot_complete((600, 300, 639, 400))
