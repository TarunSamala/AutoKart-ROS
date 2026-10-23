"""Guided HP60C survey: plan field of view, capture surveyed stations, fit, validate on held-out stations.

Surveyed coordinates come only from the operator's stations file; this tool never
invents positions. Fit stations and validation stations are different physical
placements. PASS is decided only on validation stations, with the unchanged
survey validator and the operator-chosen tolerance.

  plan      intrinsics + measured mount -> which grid cells show a cone foot
  template  write a stations file with every value left null for measurement
  capture   guided live capture with rejection checks -> recordings + annotations
  evaluate  fit on fit stations, validate on held-out stations, report, artifact on PASS
"""
import argparse
import hashlib
import json
import math
import os
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
import yaml

from .calibration import GRID_DISTANCES_M, validate_survey
from .calibration_fit import collect, fit, scaled_replays
from .calibration_store import build_artifact, write_artifact
from .depth import DepthError, sample_depth_diagnostic, validate_frame
from .mount import mount_from_config

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / 'config/perception.yaml'
REGIONS = ('left', 'center', 'right')
CONE_HEIGHT_M = 0.30
FOOT_MARGIN_PX = 4
FRAMES_PER_STATION = 5
MOUNT_KEYS = ('x_forward_m', 'y_left_m', 'height_m', 'pitch_down_deg', 'roll_deg', 'yaw_left_deg')


def mount_config(config, measured):
    return {**config, 'camera_to_kart': None,
            'camera_mount': {'height_reference': 'optical_center', **{k: float(measured[k]) for k in MOUNT_KEYS}}}


def project(mount, intr, xyz):
    optical = np.linalg.inv(mount) @ np.array([*xyz, 1.0])
    if optical[2] <= 0.05:
        return None
    return (intr['fx'] * optical[0] / optical[2] + intr['cx'], intr['fy'] * optical[1] / optical[2] + intr['cy'])


def visible(mount, intr, x, y, w=640, h=480):
    foot, top = project(mount, intr, (x, y, 0.0)), project(mount, intr, (x, y, CONE_HEIGHT_M))
    return (foot is not None and top is not None and FOOT_MARGIN_PX <= foot[0] < w - FOOT_MARGIN_PX
            and top[1] >= 0 and foot[1] < h - FOOT_MARGIN_PX), foot


def foot_complete(box, w=640, h=480):
    """Whole cone base inside the image with margin: no clipped bottom or side."""
    x1, _, x2, y2 = box
    return y2 <= h - FOOT_MARGIN_PX and x1 >= FOOT_MARGIN_PX and x2 <= w - FOOT_MARGIN_PX


def region_of(u):
    return 'left' if u < 640 / 3 else 'right' if u >= 1280 / 3 else 'center'


def lateral_for(mount, intr, distance, region):
    """Ground Y whose foot falls in the middle of that image third at this distance."""
    target_u = {'left': 640 / 6, 'center': 320, 'right': 640 * 5 / 6}[region]
    for y in np.linspace(2.5, -2.5, 501):
        ok, foot = visible(mount, intr, distance, y)
        if ok and abs(foot[0] - target_u) < 8:
            return round(float(y), 2)
    return None


def plan(intr, measured, config, distances=GRID_DISTANCES_M):
    mount, status = mount_from_config(mount_config(config, measured))
    if status != 'MOUNT_CONFIGURED_UNVERIFIED':
        raise ValueError(status)
    cells, missing = [], []
    for distance in distances:
        for region in REGIONS:
            y = lateral_for(mount, intr, distance, region)
            (cells if y is not None else missing).append({'distance_m': distance, 'region': region, 'y_left_m': y})
    proposals = []
    for cell in missing:
        for d in np.arange(cell['distance_m'], 4.01, 0.05):
            y = lateral_for(mount, intr, round(float(d), 2), cell['region'])
            if y is not None:
                proposals.append({'replaces': cell, 'distance_m': round(float(d), 2), 'y_left_m': y,
                                  'status': 'PROPOSED_REQUIRES_APPROVAL'})
                break
        else:
            proposals.append({'replaces': cell, 'status': 'NO_MEASURABLE_ALTERNATIVE_WITHIN_4M'})
    return {'intrinsics': intr, 'measured_mount': measured, 'visible_cells': cells,
            'out_of_view_cells': missing, 'proposed_alternatives': proposals,
            'note': 'Visibility uses the measured (unverified) mount; confirm on the live preview.'}


def template(plan_report, path, approved_distances=None):
    distances = approved_distances or [c['distance_m'] for c in plan_report['visible_cells']]
    stations = []
    for role in ('fit', 'validate'):
        for index, distance in enumerate(sorted(set(distances))):
            stations.append({'id': f'{role}-{index + 1}', 'role': role, 'instructions':
                             f'Place one cone in each image third near {distance} m forward; measure every cone foot.',
                             'cones': [{'region': r, 'x_m': None, 'y_m': None, 'sigma_m': None, 'label': 'orange'}
                                       for r in REGIONS]})
    doc = {'schema_version': 1, 'operator': None, 'instrument': None, 'base_link_definition': None,
           'mount_measurement': {**{k: None for k in MOUNT_KEYS},
                                 'uncertainty': {'translation_m': None, 'angle_deg': None}},
           'approved_grid': ({'distances_m': sorted(set(distances)), 'approved_by': None, 'reason': None}
                             if approved_distances else None),
           'stations': stations}
    path = Path(path)
    if path.exists():
        raise FileExistsError(path)
    path.write_text(json.dumps(doc, indent=2) + '\n')
    return doc


def load_stations(path):
    doc = json.loads(Path(path).read_text())
    problems = []
    if doc.get('schema_version') != 1:
        problems.append('SCHEMA_VERSION')
    for key in ('operator', 'instrument', 'base_link_definition'):
        if not doc.get(key):
            problems.append(f'MISSING_{key.upper()}')
    mount = doc.get('mount_measurement') or {}
    for key in MOUNT_KEYS:
        if not isinstance(mount.get(key), (int, float)) or not math.isfinite(mount[key]):
            problems.append(f'MOUNT_NOT_MEASURED:{key}')
    unc = mount.get('uncertainty') or {}
    if not all(isinstance(unc.get(k), (int, float)) and unc[k] > 0 for k in ('translation_m', 'angle_deg')):
        problems.append('MOUNT_UNCERTAINTY_NOT_RECORDED')
    grid = doc.get('approved_grid')
    if grid is not None and not (grid.get('approved_by') and grid.get('reason') and grid.get('distances_m')):
        problems.append('ALTERNATIVE_GRID_NOT_APPROVED')
    roles = Counter()
    for station in doc.get('stations', []):
        roles[station.get('role')] += 1
        for cone in station.get('cones', []):
            values = [cone.get(k) for k in ('x_m', 'y_m', 'sigma_m')]
            if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values) or cone['sigma_m'] <= 0:
                problems.append(f'CONE_NOT_MEASURED:{station.get("id")}:{cone.get("region")}')
    if not roles['fit'] or not roles['validate']:
        problems.append('NEED_FIT_AND_VALIDATE_STATIONS')
    if problems:
        raise ValueError('STATIONS_INCOMPLETE: ' + ', '.join(problems))
    return doc


def device_identity(camera, frame, configuration):
    usb = {}
    for dev in Path('/sys/bus/usb/devices').glob('*'):
        try:
            if (dev / 'idVendor').read_text().strip() == '3482' and (dev / 'idProduct').read_text().strip() == '6723':
                usb = {k: (dev / k).read_text().strip() for k in ('idVendor', 'idProduct', 'manufacturer', 'product', 'serial', 'bcdDevice')
                       if (dev / k).exists()}
        except OSError:
            continue
    i = frame.intrinsics
    return {'usb': usb or 'NOT_FOUND', 'firmware': usb.get('bcdDevice', 'UNAVAILABLE_FROM_SDK') if usb else 'UNAVAILABLE',
            'sdk_configuration': str(configuration),
            'sdk_configuration_sha256': hashlib.sha256(Path(configuration).read_bytes()).hexdigest(),
            'resolution': list(frame.rgb.shape[1::-1]), 'intrinsics': {'fx': i.fx, 'fy': i.fy, 'cx': i.cx, 'cy': i.cy},
            'aligned': bool(frame.aligned), 'raw_depth_unit': frame.depth_scale,
            'sdk_parameters': list(frame.calibration.get('sdk_parameters', []))}


def check_frame(frame, config):
    try:
        validate_frame(frame, config['max_frame_age_ms'])
    except DepthError as exc:
        return str(exc)
    return None


def associate(detections, predicted, gate_px=60):
    """Nearest detection by box bottom-centre; reject if absent or ambiguous."""
    if predicted is None:
        return None, 'PREDICTION_OUT_OF_VIEW'
    scored = sorted((math.hypot((d.box[0] + d.box[2]) / 2 - predicted[0], d.box[3] - predicted[1]), d) for d in detections)
    if not scored or scored[0][0] > gate_px:
        return None, 'NO_DETECTION_NEAR_SURVEYED_CONE'
    if len(scored) > 1 and scored[1][0] < 1.5 * scored[0][0]:
        return None, 'AMBIGUOUS_ASSOCIATION'
    return scored[0][1], None


def capture(stations_path, out_dir, library, configuration, model, manifest, config_path=DEFAULT_CONFIG,
            preview=True, wait=input):
    from .camera import CameraError, HP60C
    from .detector import Yolo26OnnxDetector
    from .pipeline import Pipeline
    from .recording import Recorder
    doc = load_stations(stations_path)
    config = yaml.safe_load(Path(config_path).read_text())
    cfg = mount_config(config, doc['mount_measurement'])
    mount, _ = mount_from_config(cfg)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=False)
    camera = HP60C(library=Path(library), configuration=Path(configuration), max_age_ms=config['max_frame_age_ms'])
    detector = Yolo26OnnxDetector(Path(model), manifest_path=Path(manifest))
    pipeline = Pipeline(detector, cfg)
    show = preview and bool(os.environ.get('DISPLAY'))
    annotations = {'fit': {'schema_version': 1, 'sessions': {}}, 'validate': {'schema_version': 1, 'sessions': {}}}
    rejections, identity = Counter(), None
    try:
        for station in doc['stations']:
            print(f"\n== Station {station['id']} ({station['role']}) ==\n{station.get('instructions', '')}")
            for cone in station['cones']:
                print(f"  cone {cone['region']}: x={cone['x_m']} m, y={cone['y_m']} m, sigma={cone['sigma_m']} m")
            wait('Place cones exactly as measured, clear the scene, press Enter to capture... ')
            session = f"{station['id']}"
            recorder = Recorder(out / session)
            labels = annotations[station['role']]['sessions'].setdefault(session, {})
            good, attempts = 0, 0
            while good < FRAMES_PER_STATION and attempts < FRAMES_PER_STATION * 20:
                attempts += 1
                try:
                    frame = camera.read()
                except CameraError as exc:
                    rejections[f'CAMERA:{exc}'] += 1
                    continue
                identity = identity or device_identity(camera, frame, configuration)
                if identity['intrinsics'] != device_identity(camera, frame, configuration)['intrinsics']:
                    rejections['DEVICE_CONFIGURATION_CHANGED'] += 1
                    continue
                problem = check_frame(frame, config)
                if problem:
                    rejections[problem] += 1
                    continue
                detections = detector.detect(frame.rgb)
                intr = {'fx': frame.intrinsics.fx, 'fy': frame.intrinsics.fy, 'cx': frame.intrinsics.cx, 'cy': frame.intrinsics.cy}
                items, frame_ok = [], True
                for cone in station['cones']:
                    detection, reason = associate(detections, project(mount, intr, (cone['x_m'], cone['y_m'], 0.0)))
                    if detection is None:
                        rejections[reason] += 1; frame_ok = False; break
                    if not foot_complete(detection.box, frame.rgb.shape[1], frame.rgb.shape[0]):
                        rejections['CONE_FOOT_CLIPPED'] += 1; frame_ok = False; break
                    sample, status = sample_depth_diagnostic(frame.depth, detection.box, frame.depth_scale, config['depth_min_m'],
                                                             config['depth_max_m'], config['min_depth_pixels'], config['max_depth_mad_m'])
                    if sample is None:
                        rejections[f'INVALID_DEPTH:{status}'] += 1; frame_ok = False; break
                    items.append({'box': list(detection.box), 'label': cone.get('label', 'orange'),
                                  'xy_m': [cone['x_m'], cone['y_m']], 'sigma_m': cone['sigma_m'],
                                  'detected_label': detection.label, 'detected_confidence': detection.confidence})
                if show:
                    view = frame.rgb.copy()
                    for d in detections:
                        cv2.rectangle(view, d.box[:2], d.box[2:], (0, 255, 0) if frame_ok else (0, 0, 255), 2)
                    cv2.imshow('HP60C calibration (RGB | depth)', np.hstack([view, cv2.applyColorMap(
                        cv2.convertScaleAbs(frame.depth, alpha=255 / 4000), cv2.COLORMAP_TURBO)]))
                    cv2.waitKey(1)
                if not frame_ok:
                    continue
                recorder.write(pipeline.process(frame))
                labels[f'{recorder.index:06d}'] = items
                good += 1
            print(f'  captured {good}/{FRAMES_PER_STATION} accepted frames after {attempts} attempts')
            if good < FRAMES_PER_STATION:
                rejections[f'STATION_INCOMPLETE:{station["id"]}'] += 1
    finally:
        camera.close()
        if show:
            cv2.destroyAllWindows()
    for role, body in annotations.items():
        (out / f'{role}_annotations.json').write_text(json.dumps(body, indent=2) + '\n')
    manifest_doc = {'schema_version': 1, 'stations_file': str(stations_path),
                    'stations_sha256': hashlib.sha256(Path(stations_path).read_bytes()).hexdigest(),
                    'device_identity': identity, 'rejections': dict(rejections), 'captured_unix_s': time.time()}
    (out / 'capture_manifest.json').write_text(json.dumps(manifest_doc, indent=2) + '\n')
    return manifest_doc


def _error_by_distance(samples, fitted, config):
    from .calibration_fit import evaluate
    rows = evaluate([fitted['depth_scale_factor'], fitted['height_m'], fitted['pitch_down_deg']], samples, config)
    groups = {}
    for sample, row in zip(samples, rows):
        groups.setdefault(round(float(np.hypot(*sample['xy'])), 1), []).append(row)
    out = []
    for distance, items in sorted(groups.items()):
        arr = np.asarray(items)
        out.append({'range_m': distance, 'count': len(items), 'x_error_m_mean': float(arr[:, 0].mean()),
                    'y_error_m_mean': float(arr[:, 1].mean()), 'position_error_m_max': float(np.hypot(arr[:, 0], arr[:, 1]).max())})
    return out


def evaluate_session(capture_dir, stations_path, tolerance_m, config_path=DEFAULT_CONFIG, artifact_path=None):
    capture_dir = Path(capture_dir)
    doc = load_stations(stations_path)
    manifest = json.loads((capture_dir / 'capture_manifest.json').read_text())
    if manifest['stations_sha256'] != hashlib.sha256(Path(stations_path).read_bytes()).hexdigest():
        raise ValueError('STATIONS_FILE_CHANGED_AFTER_CAPTURE')
    config = mount_config(yaml.safe_load(Path(config_path).read_text()), doc['mount_measurement'])
    roles = {s['id']: s['role'] for s in doc['stations']}
    recs = {role: [str(capture_dir / sid) for sid, r in roles.items() if r == role and (capture_dir / sid).is_dir()]
            for role in ('fit', 'validate')}
    fit_samples = collect(recs['fit'], capture_dir / 'fit_annotations.json', config)
    fitted = fit(fit_samples, config, held_out=False)
    fitted_mount = {**config['camera_mount'], 'height_m': fitted['height_m'], 'pitch_down_deg': fitted['pitch_down_deg']}
    candidate = {**config, 'camera_mount': fitted_mount}
    distances = tuple(doc['approved_grid']['distances_m']) if doc.get('approved_grid') else GRID_DISTANCES_M
    with scaled_replays(fitted['depth_scale_factor']):
        validation = validate_survey(recs['validate'], capture_dir / 'validate_annotations.json', candidate,
                                     tolerance_m, require_grid=True, grid_distances=distances)
    validate_samples = collect(recs['validate'], capture_dir / 'validate_annotations.json', config)
    report = {'schema_version': 1, 'status': validation['status'], 'blocker': validation.get('blocker'),
              'tolerance_m': tolerance_m, 'grid_distances_m': list(distances),
              'approved_grid': doc.get('approved_grid'), 'device_identity': manifest['device_identity'],
              'measured_mount': doc['mount_measurement'], 'fitted': fitted,
              'held_out_validation': validation,
              'held_out_error_by_range': _error_by_distance(validate_samples, fitted, config) if validate_samples else [],
              'fit_station_count': len(recs['fit']), 'validation_station_count': len(recs['validate']),
              'capture_rejections': manifest['rejections'],
              'independence': 'PASS/FAIL uses only validation stations; fit stations are never re-scored.'}
    report_path = capture_dir / 'calibration_report.json'
    report_path.write_text(json.dumps(report, indent=2, default=float) + '\n')
    if report['status'] == 'PASS' and artifact_path:
        mount, _ = mount_from_config(candidate)
        artifact = build_artifact(validation, mount, fitted_mount, fitted['depth_scale_factor'], doc['operator'],
                                  validation.get('error_m'))
        write_artifact(artifact_with_binding(artifact, report), artifact_path)
    return report


def artifact_with_binding(artifact, report):
    from .calibration_store import _digest
    bound = {**artifact, 'device_identity': report['device_identity'], 'measured_mount': report['measured_mount'],
             'fitted_parameters': {k: report['fitted'][k] for k in ('depth_scale_factor', 'height_m', 'pitch_down_deg',
                                                                     'parameter_std')},
             'grid_distances_m': report['grid_distances_m'], 'approved_grid': report['approved_grid']}
    bound['sha256'] = _digest(bound)
    return bound


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='command', required=True)
    for name in ('plan', 'template'):
        s = sub.add_parser(name)
        s.add_argument('--intrinsics', nargs=4, type=float, metavar=('FX', 'FY', 'CX', 'CY'), required=True)
        s.add_argument('--mount', required=True, help='JSON with measured x_forward_m,y_left_m,height_m,pitch_down_deg,roll_deg,yaw_left_deg')
        s.add_argument('--config', default=str(DEFAULT_CONFIG))
        if name == 'template':
            s.add_argument('--output', required=True)
            s.add_argument('--approved-distances', nargs='+', type=float, help='Alternative grid; stations file must name approver')
    c = sub.add_parser('capture')
    c.add_argument('--stations', required=True); c.add_argument('--output', required=True)
    c.add_argument('--camera-library', required=True); c.add_argument('--camera-configuration', required=True)
    c.add_argument('--model', required=True); c.add_argument('--manifest', required=True)
    c.add_argument('--config', default=str(DEFAULT_CONFIG)); c.add_argument('--no-preview', action='store_true')
    e = sub.add_parser('evaluate')
    e.add_argument('--capture', required=True); e.add_argument('--stations', required=True)
    e.add_argument('--tolerance-m', type=float, required=True, help='Chosen before testing, <= 0.15')
    e.add_argument('--artifact', help='Written only on PASS; existing file refused')
    e.add_argument('--config', default=str(DEFAULT_CONFIG))
    args = p.parse_args(argv)
    if args.command in ('plan', 'template'):
        intr = dict(zip(('fx', 'fy', 'cx', 'cy'), args.intrinsics))
        report = plan(intr, json.loads(Path(args.mount).read_text()), yaml.safe_load(Path(args.config).read_text()))
        if args.command == 'template':
            template(report, args.output, args.approved_distances)
        print(json.dumps(report, indent=2))
        return 0
    if args.command == 'capture':
        print(json.dumps(capture(args.stations, args.output, args.camera_library, args.camera_configuration,
                                 args.model, args.manifest, args.config, not args.no_preview), indent=2))
        return 0
    report = evaluate_session(args.capture, args.stations, args.tolerance_m, args.config, args.artifact)
    print(json.dumps({k: report[k] for k in ('status', 'blocker', 'fitted', 'held_out_error_by_range')}, indent=2, default=float))
    return 0 if report['status'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
