import math

import numpy as np
import pytest

from autokart_perception.calibration_groundcheck import analyse_frame, summarise

INTR = {'fx': 585.27, 'fy': 585.02, 'cx': 335.98, 'cy': 239.96}


def floor_depth(height, pitch_deg, roll_deg=0.0, scale=0.001, noise=0.0, seed=1):
    """Raw uint16 depth of an ideal flat floor seen by a camera at `height`, pitched down."""
    th, ro = math.radians(pitch_deg), math.radians(roll_deg)
    up = np.array([math.sin(ro) * math.cos(th), -math.cos(ro) * math.cos(th), -math.sin(th)])
    up /= np.linalg.norm(up)
    v, u = np.mgrid[0:480, 0:640]
    rays = np.stack([(u - INTR['cx']) / INTR['fx'], (v - INTR['cy']) / INTR['fy'], np.ones_like(u, float)], -1)
    denom = rays @ up
    with np.errstate(divide='ignore', invalid='ignore'):
        z = np.where(denom < -1e-6, -height / denom, np.nan)
    z = np.where((z > 0.3) & (z < 4.0), z, 0)
    z = z + np.random.default_rng(seed).normal(0, noise, z.shape) * (z > 0)
    return np.nan_to_num(z / scale).clip(0, 65535).astype(np.uint16)


def test_recovers_reported_kart_mount():
    f = analyse_frame(floor_depth(0.8382, 17.8), 0.001, INTR)
    assert f['status'] == 'PLANE_FOUND'
    assert f['height_m'] == pytest.approx(0.8382, abs=0.01)
    assert f['pitch_down_deg'] == pytest.approx(17.8, abs=0.3)
    assert abs(f['roll_deg']) < 0.3


def test_recovers_with_sensor_noise_and_roll():
    f = analyse_frame(floor_depth(0.84, 15.0, roll_deg=3.0, noise=0.004), 0.001, INTR)
    assert f['status'] == 'PLANE_FOUND'
    assert f['pitch_down_deg'] == pytest.approx(15.0, abs=1.0)
    assert f['roll_deg'] == pytest.approx(3.0, abs=1.0)
    assert f['height_m'] == pytest.approx(0.84, abs=0.03)


def test_pitch_independent_of_depth_scale_but_height_is_not():
    # Same physical floor, but the driver scale is wrong by 20%.
    raw = floor_depth(0.8382, 17.8)
    wrong = analyse_frame(raw, 0.0012, INTR)
    assert wrong['pitch_down_deg'] == pytest.approx(17.8, abs=0.3)
    assert wrong['height_m'] == pytest.approx(0.8382 * 1.2, abs=0.02)


def test_empty_depth_is_no_plane_not_a_number():
    f = analyse_frame(np.zeros((480, 640), np.uint16), 0.001, INTR)
    assert f['status'] == 'NO_GROUND_PLANE' and 'height_m' not in f


def test_cluttered_scene_without_dominant_plane_is_rejected():
    rng = np.random.default_rng(3)
    raw = (rng.uniform(500, 3500, (480, 640))).astype(np.uint16)
    f = analyse_frame(raw, 0.001, INTR)
    assert f['status'] == 'NO_GROUND_PLANE'


def test_summary_never_verifies_and_flags_inconsistency():
    frames = [analyse_frame(floor_depth(0.30, 4.0), 0.001, INTR, seed=i) for i in range(6)]
    rep = summarise(frames, {'height_m': 0.8382, 'pitch_down_deg': 17.8, 'roll_deg': 0.0})
    assert rep['qualified'] is False and rep['verification'] == 'NOT_VERIFIED_EVIDENCE_ONLY'
    assert rep['verdict'] == 'INCONSISTENT_WITH_REPORTED'
    assert not rep['pitch_consistent'] and not rep['height_consistent']


def test_summary_consistent_case_still_not_verified():
    frames = [analyse_frame(floor_depth(0.8382, 17.8, noise=0.003), 0.001, INTR, seed=i) for i in range(6)]
    rep = summarise(frames, {'height_m': 0.8382, 'pitch_down_deg': 17.8, 'roll_deg': 0.0})
    assert rep['verdict'] == 'CONSISTENT_WITH_REPORTED'
    assert rep['qualified'] is False


def test_too_few_planes_gives_no_ground_plane_verdict():
    frames = [analyse_frame(np.zeros((480, 640), np.uint16), 0.001, INTR) for _ in range(6)]
    rep = summarise(frames, {'height_m': 0.8382, 'pitch_down_deg': 17.8})
    assert rep['verdict'] == 'NO_GROUND_PLANE' and 'estimate' not in rep
