from pathlib import Path
import yaml
from autokart_perception.pipeline import Pipeline
from autokart_perception.synthetic import Synthetic
from autokart_perception.types import Detection

CFG = yaml.safe_load((Path(__file__).parents[1] / 'config/perception.yaml').read_text())


class Fixed:
    verified_checkpoint = True; version = 'fixed'; backend = 'test'; confidence = .5; iou = .5; latency_ms = 1.0

    def __init__(self, boxes):
        self.boxes = boxes

    def detect(self, rgb):
        return [Detection(box, 'orange', .9, i) for i, box in enumerate(self.boxes)]


def live(frame):
    import time
    frame.source = 'LIVE'; frame.timestamp_ns = time.monotonic_ns(); frame.aligned = True
    return frame


def test_frame_with_only_clipped_cone_feet_is_invalid_but_does_not_latch():
    pipe = Pipeline(Fixed([(300, 380, 340, 480)]), CFG)
    result = pipe.process(live(Synthetic().read()))
    assert result.status == 'METRIC_LOCALIZATION_FAILED' and result.safety.state == 'SENSOR_FAULT'
    assert not result.cones and not result.centerline
    assert pipe.latched_fault_reason is None
    assert 'MANUAL_RESTART_REQUIRED' not in result.safety.reasons


def test_genuine_sensor_integrity_fault_still_latches_until_operator_clears():
    pipe = Pipeline(None, CFG, True)
    pipe.note_sensor_fault('CAMERA_DISCONNECTED')
    assert 'MANUAL_RESTART_REQUIRED' in pipe.process(Synthetic().read()).safety.reasons
    assert pipe.clear_sensor_fault() == 'CAMERA_DISCONNECTED'
    assert pipe.latched_fault_reason is None
    assert pipe.process(Synthetic().read()).centerline
