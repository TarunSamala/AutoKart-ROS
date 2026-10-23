"""Versioned, evidence-bound HP60C calibration artifact.

An artifact only grants ``MOUNT_VERIFIED`` when it embeds a passing survey
report made from LIVE recordings covering the full 0.5/1/2/3 m x left/centre/
right grid, matches the mount currently configured, and has not expired. The
hash detects accidental edits; it is not a signature against a malicious user.
"""
import copy
import hashlib
import json
import time
from pathlib import Path

import numpy as np

MAX_TOLERANCE_M = 0.15
STANDARD_GRID_M = (0.5, 1.0, 2.0, 3.0)
DEFAULT_VALIDITY_S = 30 * 24 * 3600


def mount_sha256(mount):
    return hashlib.sha256(np.asarray(mount, dtype='<f8').tobytes()).hexdigest()


def _digest(body):
    body = {k: v for k, v in body.items() if k != 'sha256'}
    return hashlib.sha256(json.dumps(body, sort_keys=True, allow_nan=False).encode()).hexdigest()


def build_artifact(survey, mount, camera_mount, depth_scale_factor, operator, held_out_error_m,
                   now=None, validity_s=DEFAULT_VALIDITY_S):
    now = time.time() if now is None else now
    body = {'schema_version': 1, 'status': 'BENCH_VERIFIED', 'operator': str(operator),
            'created_unix_s': now, 'expires_unix_s': now + validity_s,
            'camera_mount': copy.deepcopy(camera_mount), 'mount_sha256': mount_sha256(mount),
            'depth_scale_factor': float(depth_scale_factor),
            'held_out_error_m': copy.deepcopy(held_out_error_m), 'survey': survey}
    body['sha256'] = _digest(body)
    return body


def write_artifact(artifact, path):
    path = Path(path)
    if path.exists():
        raise FileExistsError(path)
    path.write_text(json.dumps(artifact, indent=2, sort_keys=True, allow_nan=False) + '\n')


def verify_artifact(path, mount, now=None):
    """Return (status, reasons, depth_scale_factor); status is MOUNT_VERIFIED only if all checks pass."""
    now = time.time() if now is None else now
    try:
        doc = json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return 'MOUNT_CONFIGURED_UNVERIFIED', ['CALIBRATION_ARTIFACT_UNREADABLE'], 1.0
    reasons = []
    if not isinstance(doc, dict) or doc.get('schema_version') != 1 or doc.get('status') != 'BENCH_VERIFIED':
        reasons.append('CALIBRATION_ARTIFACT_STATUS')
    else:
        survey = doc.get('survey') or {}
        factor = doc.get('depth_scale_factor')
        tolerance = survey.get('tolerance_m')
        if doc.get('sha256') != _digest(doc):
            reasons.append('CALIBRATION_ARTIFACT_HASH')
        if doc.get('mount_sha256') != mount_sha256(mount):
            reasons.append('CALIBRATION_MOUNT_MISMATCH')
        if not isinstance(doc.get('expires_unix_s'), (int, float)) or doc['expires_unix_s'] < now:
            reasons.append('CALIBRATION_EXPIRED')
        if survey.get('status') != 'PASS' or set(survey.get('recorded_sources', {})) != {'LIVE'}:
            reasons.append('CALIBRATION_SURVEY_NOT_LIVE_PASS')
        distances = survey.get('grid_distances_m') or list(STANDARD_GRID_M)
        if not survey.get('required_grid') or len(survey.get('grid_coverage', [])) != 3 * len(distances):
            reasons.append('CALIBRATION_GRID_INCOMPLETE')
        approval = doc.get('approved_grid') or {}
        if [float(d) for d in distances] != list(STANDARD_GRID_M) and not (
                approval.get('approved_by') and approval.get('reason') and
                [float(d) for d in approval.get('distances_m', [])] == [float(d) for d in distances]):
            reasons.append('CALIBRATION_ALTERNATIVE_GRID_NOT_APPROVED')
        if not isinstance(tolerance, (int, float)) or not 0 < tolerance <= MAX_TOLERANCE_M:
            reasons.append('CALIBRATION_TOLERANCE_INVALID')
        if not isinstance(factor, (int, float)) or not np.isfinite(factor) or not 0.5 <= factor <= 2.0:
            reasons.append('CALIBRATION_DEPTH_FACTOR_INVALID')
    if reasons:
        return 'MOUNT_CONFIGURED_UNVERIFIED', reasons, 1.0
    return 'MOUNT_VERIFIED', [], float(doc['depth_scale_factor'])


def artifact_summary(path):
    """Display-only facts from an artifact file; validity still comes from verify_artifact."""
    try:
        doc = json.loads(Path(path).read_text())
        survey = doc.get('survey') or {}
        return {'operator': doc.get('operator'), 'created_unix_s': doc.get('created_unix_s'),
                'expires_unix_s': doc.get('expires_unix_s'), 'held_out_error_m': doc.get('held_out_error_m'),
                'survey_error_m': survey.get('error_m'), 'tolerance_m': survey.get('tolerance_m'),
                'depth_scale_factor': doc.get('depth_scale_factor'), 'camera_mount': doc.get('camera_mount'),
                'device_identity': doc.get('device_identity'), 'grid_distances_m': survey.get('grid_distances_m'),
                'approved_grid': doc.get('approved_grid')}
    except (OSError, ValueError, AttributeError):
        return None
