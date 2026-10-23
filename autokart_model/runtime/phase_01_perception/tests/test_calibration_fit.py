import json
from pathlib import Path
import numpy as np
import yaml
from autokart_perception.calibration_fit import collect, fit
from autokart_perception.mount import mount_from_config
from autokart_perception.pipeline import Pipeline
from autokart_perception.recording import Recorder
from autokart_perception.synthetic import Synthetic

CONFIG = yaml.safe_load((Path(__file__).parents[1] / 'config/perception.yaml').read_text())
TRUE = {'height_m': .32, 'pitch_down_deg': 6.0}
TRUE_FACTOR = 1.05


def scene(tmp_path, points):
    truth = {**CONFIG, 'camera_to_kart': None, 'camera_mount': {**CONFIG['camera_mount'], **TRUE}}
    mount, _ = mount_from_config(truth)
    inverse = np.linalg.inv(mount)
    frame = Synthetic().read()
    intr = frame.intrinsics
    frame.depth[:] = 0
    labels = []
    for x, y in points:
        optical = inverse @ np.array([x, y, 0.0, 1.0])
        z = optical[2]
        u = intr.fx * optical[0] / z + intr.cx
        foot = intr.fy * optical[1] / z + intr.cy
        box = [int(u - 15), int(foot - 60), int(u + 15), int(foot + 1)]
        raw = int(round(z / (frame.depth_scale * TRUE_FACTOR)))
        frame.depth[max(0, box[1]):box[3], box[0]:box[2]] = raw
        labels.append({'box': box, 'label': 'orange', 'xy_m': [x, y]})
    recording = tmp_path / 'session'
    Recorder(recording).write(Pipeline(None, truth, True).process(frame))
    annotations = tmp_path / 'labels.json'
    annotations.write_text(json.dumps({'schema_version': 1, 'sessions': {'session': {'000001': labels}}}))
    return recording, annotations


def test_fit_recovers_scale_height_and_pitch_on_held_out_cones(tmp_path):
    points = [(d, y) for d in (1.6, 2.0, 2.6, 3.2) for y in (-0.6, 0.0, 0.6)]
    recording, annotations = scene(tmp_path, points)
    samples = collect([recording], annotations, CONFIG)
    assert len(samples) == 12
    result = fit(samples, CONFIG)
    assert abs(result['depth_scale_factor'] - TRUE_FACTOR) < .01
    assert abs(result['height_m'] - TRUE['height_m']) < .01
    assert abs(result['pitch_down_deg'] - TRUE['pitch_down_deg']) < .5
    assert result['held_out_error_m']['max'] < .05
    assert result['held_out_count'] == 6


def test_too_few_surveyed_cones_refused(tmp_path):
    recording, annotations = scene(tmp_path, [(1.5, 0.0), (2.0, 0.3)])
    try:
        fit(collect([recording], annotations, CONFIG), CONFIG)
    except ValueError as exc:
        assert 'NEED_AT_LEAST_6' in str(exc)
    else:
        raise AssertionError('expected refusal')
