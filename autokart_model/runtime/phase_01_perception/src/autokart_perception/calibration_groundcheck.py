"""Ground-plane self-check: compare the floor seen in depth with the tape-measured mount.

This is EVIDENCE, never qualification. It fits the dominant plane in the lower image
region of real depth frames and reports the camera height, pitch and roll that plane
implies, next to the operator's reported mount. It can never produce MOUNT_VERIFIED or
DEPTH_VERIFIED: verification still requires the surveyed-station session
(`calibration_session`) with held-out stations.

Limits stated up front:
  * Pitch and roll depend only on the plane normal, so they do not depend on the depth
    scale. Height scales linearly with the (unverified) depth scale, so a height mismatch
    cannot be split between a wrong depth scale and a wrong tape measurement.
  * It needs a flat floor filling the lower image. With no such plane it reports
    NO_GROUND_PLANE instead of a number.

Optical frame: x right, y down, z forward. Kart frame: X forward, Y left, Z up.
A camera pitched down by theta sees world-up as (0, -cos theta, -sin theta).
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

DEFAULT_TOLERANCE = {'height_m': 0.05, 'pitch_deg': 2.0, 'roll_deg': 2.0}
MIN_INLIER_FRACTION = 0.5
MIN_POINTS = 2000
INLIER_THRESHOLD_M = 0.02


def backproject(depth_raw, scale, intr, rows, min_m, max_m, stride=4):
    """Metric optical points from the image rows in [rows[0], rows[1]) on a pixel stride."""
    r0, r1 = rows
    sub = depth_raw[r0:r1:stride, ::stride].astype(np.float64) * scale
    v, u = np.mgrid[r0:r1:stride, 0:depth_raw.shape[1]:stride]
    valid = np.isfinite(sub) & (sub >= min_m) & (sub <= max_m)
    z = sub[valid]
    x = (u[valid] - intr['cx']) * z / intr['fx']
    y = (v[valid] - intr['cy']) * z / intr['fy']
    return np.column_stack([x, y, z])


def fit_plane_ransac(points, threshold=INLIER_THRESHOLD_M, iterations=300, seed=0):
    """Deterministic RANSAC then least-squares refit. Returns (normal, offset, inlier_mask)."""
    rng = np.random.default_rng(seed)
    n = len(points)
    best = None
    for _ in range(iterations):
        idx = rng.choice(n, 3, replace=False)
        a, b, c = points[idx]
        normal = np.cross(b - a, c - a)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        offset = -normal @ a
        inliers = np.abs(points @ normal + offset) <= threshold
        count = int(inliers.sum())
        if best is None or count > best[0]:
            best = (count, inliers)
    if best is None:
        return None, None, None
    inliers = best[1]
    pts = points[inliers]
    centroid = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - centroid, full_matrices=False)
    normal = vt[2]
    offset = -normal @ centroid
    inliers = np.abs(points @ normal + offset) <= threshold
    return normal, offset, inliers


def mount_from_plane(normal, offset):
    """Camera height, pitch-down and roll implied by a floor plane n.p + offset = 0."""
    n = np.asarray(normal, dtype=float)
    d = float(offset)
    if n[1] > 0:  # orient the normal to point up (negative optical y)
        n, d = -n, -d
    height = d
    pitch = math.degrees(math.atan2(-n[2], -n[1]))
    roll = math.degrees(math.atan2(n[0], -n[1]))
    return {'height_m': height, 'pitch_down_deg': pitch, 'roll_deg': roll}


def analyse_frame(depth_raw, scale, intr, min_m=0.3, max_m=4.0, lower_fraction=0.45, seed=0):
    h = depth_raw.shape[0]
    rows = (int(h * (1 - lower_fraction)), h)
    pts = backproject(depth_raw, scale, intr, rows, min_m, max_m)
    if len(pts) < MIN_POINTS:
        return {'status': 'NO_GROUND_PLANE', 'reason': 'INSUFFICIENT_VALID_DEPTH', 'points': int(len(pts))}
    normal, offset, inliers = fit_plane_ransac(pts, seed=seed)
    if normal is None:
        return {'status': 'NO_GROUND_PLANE', 'reason': 'DEGENERATE', 'points': int(len(pts))}
    fraction = float(inliers.mean())
    if fraction < MIN_INLIER_FRACTION:
        return {'status': 'NO_GROUND_PLANE', 'reason': 'NO_DOMINANT_PLANE',
                'points': int(len(pts)), 'inlier_fraction': fraction}
    estimate = mount_from_plane(normal, offset)
    if estimate['height_m'] <= 0:
        return {'status': 'NO_GROUND_PLANE', 'reason': 'PLANE_ABOVE_CAMERA', 'points': int(len(pts))}
    residual = np.abs(pts[inliers] @ normal + offset)
    return {'status': 'PLANE_FOUND', 'points': int(len(pts)), 'inlier_fraction': fraction,
            'rms_residual_m': float(np.sqrt(np.mean(residual ** 2))), **estimate}


def summarise(frames, reported, tolerance=None):
    tolerance = {**DEFAULT_TOLERANCE, **(tolerance or {})}
    found = [f for f in frames if f['status'] == 'PLANE_FOUND']
    report = {'schema_version': 1, 'qualified': False, 'verification': 'NOT_VERIFIED_EVIDENCE_ONLY',
              'frames_analysed': len(frames), 'frames_with_plane': len(found),
              'reported_mount': reported, 'tolerance': tolerance}
    if len(found) < max(3, len(frames) // 2):
        report.update(verdict='NO_GROUND_PLANE',
                      reasons=sorted({f.get('reason', 'UNKNOWN') for f in frames if f['status'] != 'PLANE_FOUND'}))
        return report
    est = {k: float(np.median([f[k] for f in found])) for k in ('height_m', 'pitch_down_deg', 'roll_deg')}
    spread = {k: float(np.std([f[k] for f in found])) for k in est}
    delta = {'height_m': est['height_m'] - reported['height_m'],
             'pitch_deg': est['pitch_down_deg'] - reported['pitch_down_deg'],
             'roll_deg': est['roll_deg'] - reported.get('roll_deg', 0.0)}
    pitch_ok = abs(delta['pitch_deg']) <= tolerance['pitch_deg']
    roll_ok = abs(delta['roll_deg']) <= tolerance['roll_deg']
    height_ok = abs(delta['height_m']) <= tolerance['height_m']
    report.update(
        estimate=est, frame_to_frame_std=spread, delta_vs_reported=delta,
        pitch_consistent=pitch_ok, roll_consistent=roll_ok, height_consistent=height_ok,
        implied_depth_scale_factor_if_height_correct=reported['height_m'] / est['height_m'],
        verdict='CONSISTENT_WITH_REPORTED' if (pitch_ok and roll_ok and height_ok) else 'INCONSISTENT_WITH_REPORTED',
        note=('Pitch/roll are independent of depth scale. A height mismatch cannot be attributed to depth '
              'scale versus the tape measurement without an independent reference.'))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description='Ground-plane self-check (evidence only, never verifies)')
    parser.add_argument('--mount', required=True, help='JSON with reported height_m,pitch_down_deg,roll_deg')
    parser.add_argument('--frames', type=int, default=20)
    parser.add_argument('--library')
    parser.add_argument('--camera-configuration')
    parser.add_argument('--out')
    args = parser.parse_args(argv)
    from .camera import CameraError, HP60C
    reported = json.loads(Path(args.mount).read_text())
    cam = HP60C(library=args.library, configuration=args.camera_configuration)
    frames, errors, deadline = [], {}, time.monotonic() + 30
    try:
        cam.start()
        while len(frames) < args.frames and time.monotonic() < deadline:
            try:
                f = cam.read()
            except CameraError as exc:
                errors[str(exc)] = errors.get(str(exc), 0) + 1
                time.sleep(0.1)
                continue
            intr = {'fx': f.intrinsics.fx, 'fy': f.intrinsics.fy, 'cx': f.intrinsics.cx, 'cy': f.intrinsics.cy}
            frames.append(analyse_frame(f.depth, f.depth_scale, intr, seed=len(frames)))
    finally:
        cam.close()
    report = summarise(frames, reported)
    report['source'] = 'LIVE_HP60C'
    report['camera_errors'] = errors
    text = json.dumps(report, indent=2)
    if args.out:
        Path(args.out).write_text(text + '\n')
    print(text)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
