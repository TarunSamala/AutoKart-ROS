import json
from pathlib import Path
from kart_sim.scenario import run_scenario
from kart_sim.safety_metrics import derive_safety_metrics

ROOT = Path(__file__).parents[2]


def test_metrics_capture_detection_action_torque_brake_stop_distance_and_invariants(tmp_path):
    output = run_scenario(ROOT / "configs/scenarios/vcu_controlled_stop_nominal.yaml", tmp_path)
    rows = [json.loads(line) for line in (output / "telemetry.jsonl").read_text().splitlines()]
    metrics = derive_safety_metrics(rows)
    assert metrics.fault_detection_s is not None
    assert metrics.safety_action_s is not None
    assert metrics.torque_zero_s is not None
    assert metrics.brake_response_s is not None
    assert metrics.stopped_s is not None
    assert metrics.distance_after_detection_m is not None and metrics.distance_after_detection_m >= 0
    assert metrics.invariant_violations == ()
