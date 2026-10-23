import json
import numpy as np
import pytest
from autokart_perception.calibration_store import build_artifact, mount_sha256, verify_artifact, write_artifact
from autokart_perception.mount import mount_from_config
from autokart_perception.pipeline import Pipeline
import yaml
from pathlib import Path

CONFIG = yaml.safe_load((Path(__file__).parents[1] / 'config/perception.yaml').read_text())
MOUNT, _ = mount_from_config(CONFIG)
GRID = [{'distance_m': d, 'region': r} for d in (.5, 1., 2., 3.) for r in ('left', 'center', 'right')]


def survey(**over):
    base = {'status': 'PASS', 'recorded_sources': {'LIVE': 12}, 'required_grid': True,
            'grid_coverage': GRID, 'tolerance_m': .1, 'surveyed_cones': 12}
    base.update(over)
    return base


def make(tmp_path, name='cal.json', mount=MOUNT, factor=1.02, **over):
    art = build_artifact(survey(**over), mount, CONFIG['camera_mount'], factor, 'tester',
                         {'median': .02, 'p95': .05, 'max': .06}, now=1000.0, validity_s=100)
    path = tmp_path / name
    write_artifact(art, path)
    return path


def test_valid_live_artifact_verifies_and_returns_factor(tmp_path):
    status, reasons, factor = verify_artifact(make(tmp_path), MOUNT, now=1050.0)
    assert (status, reasons, factor) == ('MOUNT_VERIFIED', [], 1.02)


@pytest.mark.parametrize('over,reason', [
    ({'recorded_sources': {'SYNTHETIC': 12}}, 'CALIBRATION_SURVEY_NOT_LIVE_PASS'),
    ({'recorded_sources': {'LIVE': 6, 'REPLAY': 6}}, 'CALIBRATION_SURVEY_NOT_LIVE_PASS'),
    ({'status': 'FAIL'}, 'CALIBRATION_SURVEY_NOT_LIVE_PASS'),
    ({'grid_coverage': GRID[:11]}, 'CALIBRATION_GRID_INCOMPLETE'),
    ({'required_grid': False}, 'CALIBRATION_GRID_INCOMPLETE'),
    ({'tolerance_m': .5}, 'CALIBRATION_TOLERANCE_INVALID')])
def test_insufficient_evidence_stays_unverified(tmp_path, over, reason):
    status, reasons, factor = verify_artifact(make(tmp_path, **over), MOUNT, now=1050.0)
    assert status == 'MOUNT_CONFIGURED_UNVERIFIED' and reason in reasons and factor == 1.0


def test_expired_mismatched_and_tampered_artifacts_rejected(tmp_path):
    path = make(tmp_path)
    assert 'CALIBRATION_EXPIRED' in verify_artifact(path, MOUNT, now=2000.0)[1]
    other = np.array(MOUNT); other[2, 3] += .01
    assert 'CALIBRATION_MOUNT_MISMATCH' in verify_artifact(path, other, now=1050.0)[1]
    doc = json.loads(path.read_text()); doc['depth_scale_factor'] = 1.5
    path.write_text(json.dumps(doc))
    assert 'CALIBRATION_ARTIFACT_HASH' in verify_artifact(path, MOUNT, now=1050.0)[1]
    assert verify_artifact(tmp_path / 'missing.json', MOUNT)[1] == ['CALIBRATION_ARTIFACT_UNREADABLE']


def test_pipeline_uses_artifact_only_when_verified(tmp_path):
    import time
    cfg = {**CONFIG}
    good = build_artifact(survey(), MOUNT, CONFIG['camera_mount'], 1.03, 't', {}, now=time.time())
    gp = tmp_path / 'good.json'; write_artifact(good, gp)
    pipe = Pipeline(None, {**cfg, 'calibration_artifact': str(gp)})
    assert pipe.mount_status == 'MOUNT_VERIFIED' and pipe.depth_scale_factor == 1.03
    bad = tmp_path / 'bad.json'; bad.write_text('{}')
    pipe = Pipeline(None, {**cfg, 'calibration_artifact': str(bad)})
    assert pipe.mount_status == 'MOUNT_CONFIGURED_UNVERIFIED' and pipe.depth_scale_factor == 1.0
    assert Pipeline(None, cfg).mount_status == 'MOUNT_CONFIGURED_UNVERIFIED'
