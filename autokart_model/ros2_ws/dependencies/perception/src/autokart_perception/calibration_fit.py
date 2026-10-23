"""Fit depth scale factor, camera height and pitch from surveyed LIVE cones, then emit a verified artifact.

Surveyed X/Y of the camera origin, roll and yaw stay as measured; only
depth_scale_factor, height_m and pitch_down_deg are estimated. The CLI fits on
one set of placements and judges PASS only on a separate validation set with
the unchanged ``validate_survey``; fit cones are never re-scored.
"""
import argparse
import json
from dataclasses import replace
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import least_squares

from .calibration import validate_survey
from .calibration_store import build_artifact, write_artifact
from .depth import DepthError, localize_diagnostic, validate_frame
from .evaluation import load_annotations, recording_files
from .mount import mount_from_config
from .recording import Replay
from .types import Detection

PENALTY_M = 1.0


def collect(recordings, annotations, config):
    sessions = recording_files(recordings)
    labeled = load_annotations(annotations, sessions)
    samples = []
    for path in map(Path, recordings):
        replay = Replay(path)
        for meta_path in replay.files:
            frame = replay.read()
            key = (path.name, meta_path.stem)
            if key not in labeled:
                continue
            for index, item in enumerate(labeled[key][1]):
                if item['xy_m'] is not None:
                    samples.append({'frame': frame, 'box': item['box'], 'label': item['label'],
                                    'index': index, 'xy': np.asarray(item['xy_m'], float),
                                    'session': path.name, 'stem': meta_path.stem})
    return samples


def evaluate(params, samples, config):
    """Per sample: [ground dx, ground dy, measured - expected optical range] or penalties on failure."""
    factor, height, pitch = params
    cfg = {**config, 'camera_to_kart': None, 'max_ground_contact_offset_m': 10.0,
           'camera_mount': {**config['camera_mount'], 'height_m': height, 'pitch_down_deg': pitch}}
    mount, status = mount_from_config(cfg)
    rows = []
    for sample in samples:
        row = [PENALTY_M] * 3
        if status == 'MOUNT_CONFIGURED_UNVERIFIED':
            frame = replace(sample['frame'], depth_scale=sample['frame'].depth_scale * factor)
            try:
                validate_frame(frame, config['max_frame_age_ms'])
                _, detail = localize_diagnostic(frame, Detection(sample['box'], sample['label'], 1.0, sample['index']), mount, cfg)
                if 'ground_candidate_xy_m' in detail:
                    expected = (np.linalg.inv(mount) @ np.array([*sample['xy'], 0.0, 1.0]))[2]
                    row = [*(np.asarray(detail['ground_candidate_xy_m']) - sample['xy']),
                           detail['sampled_depth_m'] - expected]
            except DepthError:
                pass
        rows.append(row)
    return np.asarray(rows)


def _residuals(params, samples, config):
    return evaluate(params, samples, config).ravel()


def fit(samples, config, held_out=True):
    """Least-squares fit. held_out=True keeps odd cones out of the fit for a quick split check;
    the guided session passes held_out=False because its validation uses separate placements."""
    if len(samples) < 6:
        raise ValueError('NEED_AT_LEAST_6_SURVEYED_CONES')
    train, held = (samples[0::2], samples[1::2]) if held_out else (samples, [])
    start = [1.0, float(config['camera_mount']['height_m']), float(config['camera_mount']['pitch_down_deg'])]
    result = least_squares(_residuals, start, args=(train, config), x_scale=[0.05, 0.05, 2.0],
                           bounds=([0.7, 0.05, -10.0], [1.4, 1.0, 45.0]), diff_step=1e-3)
    factor, height, pitch = (float(v) for v in result.x)
    dof = max(1, result.fun.size - 3)
    try:
        covariance = np.linalg.inv(result.jac.T @ result.jac) * float(result.fun @ result.fun) / dof
        std = [float(np.sqrt(max(0.0, v))) for v in np.diag(covariance)]
    except np.linalg.LinAlgError:
        std = [None, None, None]

    def errors(group):
        rows = evaluate(result.x, group, config)
        return np.hypot(rows[:, 0], rows[:, 1])
    stats = lambda e: {'median': float(np.median(e)), 'p95': float(np.percentile(e, 95)), 'max': float(e.max())} if len(e) else None
    return {'depth_scale_factor': factor, 'height_m': height, 'pitch_down_deg': pitch,
            'parameter_std': dict(zip(('depth_scale_factor', 'height_m', 'pitch_down_deg'), std)),
            'train_error_m': stats(errors(train)), 'held_out_error_m': stats(errors(held)) if held else None,
            'train_count': len(train), 'held_out_count': len(held), 'converged': bool(result.success)}


class scaled_replays:
    """Apply the fitted depth factor to replayed frames while the unchanged survey validator runs."""

    def __init__(self, factor):
        self.factor = factor

    def __enter__(self):
        import autokart_perception.calibration as calibration
        self.module, self.original, factor = calibration, calibration.Replay, self.factor

        class Scaled(self.original):
            def read(self):
                frame = super().read()
                return None if frame is None else replace(frame, depth_scale=frame.depth_scale * factor)
        calibration.Replay = Scaled
        return self

    def __exit__(self, *exc):
        self.module.Replay = self.original


def main(argv=None):
    p = argparse.ArgumentParser(description='Fit HP60C calibration; PASS is judged only on separate validation placements')
    p.add_argument('--fit-recording', action='append', required=True)
    p.add_argument('--fit-annotations', required=True)
    p.add_argument('--validate-recording', action='append', required=True)
    p.add_argument('--validate-annotations', required=True)
    p.add_argument('--config', default=str(Path(__file__).resolve().parents[2] / 'config/perception.yaml'))
    p.add_argument('--tolerance-m', type=float, required=True, help='Chosen before testing; <= 0.15')
    p.add_argument('--operator', required=True)
    p.add_argument('--output', required=True, help='New calibration artifact JSON; existing file refused')
    args = p.parse_args(argv)
    if set(args.fit_recording) & set(args.validate_recording):
        raise SystemExit('fit and validation recordings must be different placements')
    config = yaml.safe_load(Path(args.config).read_text())
    fitted = fit(collect(args.fit_recording, args.fit_annotations, config), config, held_out=False)
    mount_params = {**config['camera_mount'], 'height_m': fitted['height_m'], 'pitch_down_deg': fitted['pitch_down_deg']}
    candidate = {**config, 'camera_to_kart': None, 'camera_mount': mount_params}
    mount, _ = mount_from_config(candidate)
    with scaled_replays(fitted['depth_scale_factor']):
        survey = validate_survey(args.validate_recording, args.validate_annotations, candidate, args.tolerance_m, require_grid=True)
    print(json.dumps({'fit': fitted, 'held_out_status': survey['status'], 'blocker': survey.get('blocker')}, indent=2))
    if survey['status'] != 'PASS':
        return 2
    write_artifact(build_artifact(survey, mount, mount_params, fitted['depth_scale_factor'], args.operator,
                                  survey.get('error_m')), args.output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
