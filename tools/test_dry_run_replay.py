import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [
    str(ROOT / "autonomy_stack_phase1" / "phase_01_perception" / "src"),
    str(ROOT / "kart" / "ros2_ws" / "src" / "autokart_vehicle_geometry"),
    str(ROOT / "kart" / "ros2_ws" / "src" / "autokart_nodes" / "autokart_planner"),
    str(ROOT / "kart" / "ros2_ws" / "src" / "autokart_vehicle_bridge"),
]

import dry_run_replay as dr


class FakeIntrinsics:
    def __init__(self):
        self.fx = self.fy = 585.0
        self.cx, self.cy = 320.0, 240.0

    def valid(self):
        return True


class FakePerceptionResult:
    def __init__(self, cones):
        self.cones = cones
        self.detections = []
        self.diagnostics = {}


class FakeSDKResult:
    def __init__(self, cones):
        self.perception_result = FakePerceptionResult(cones)
        self.processing_ms = 1.0


class FakeCone:
    def __init__(self, x, y):
        self.xy = (x, y)


def test_qualified_cones_extracts_finite_xy_only():
    good = FakeCone(2.0, 0.1)
    bad = FakeCone(float("nan"), 0.0)
    result = FakeSDKResult([good, bad])
    out = dr.qualified_cones(result, cone_radius_m=0.12)
    assert out == [{"x_m": 2.0, "y_m": 0.1, "radius_m": 0.12}]


def test_qualified_cones_empty_when_no_cones_attribute():
    class Empty:
        perception_result = type("P", (), {})()

    out = dr.qualified_cones(Empty(), cone_radius_m=0.1)
    assert out == []


def make_synthetic_replay_dir(tmp_path, n_frames=5, cone_every=None):
    """Build a minimal Replay-format directory without touching the camera or recording.Recorder
    (which requires a real Frame/result pipeline run); write the schema directly instead."""
    import numpy as np

    for index in range(1, n_frames + 1):
        name = f"{index:06d}"
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        depth = np.full((480, 640), 2000, dtype=np.uint16)
        np.savez_compressed(tmp_path / f"{name}.npz", rgb=rgb, depth=depth)
        meta = {
            "schema_version": 2, "timestamp_ns": 1_000_000_000 + index * 50_000_000,
            "rgb_timestamp": index, "depth_timestamp": index, "rgb_sequence": index,
            "depth_sequence": index, "intrinsics": {"fx": 585.0, "fy": 585.0, "cx": 320.0, "cy": 240.0},
            "depth_scale": 0.001, "aligned": True, "source": "REPLAY", "drops": 0,
            "calibration": {}, "model_version": "test", "model_sha256": None,
            "calibration_sha256": None,
            "array_sha256": {
                "rgb": __import__("hashlib").sha256(rgb.tobytes()).hexdigest(),
                "depth": __import__("hashlib").sha256(depth.tobytes()).hexdigest(),
            },
            "diagnostics": {}, "detections": [], "cones": [], "received_frames": index,
            "capture_elapsed_ns": 0,
        }
        (tmp_path / f"{name}.json").write_text(json.dumps(meta))


def test_dry_run_replay_runs_end_to_end_on_synthetic_frames(tmp_path, monkeypatch):
    make_synthetic_replay_dir(tmp_path, n_frames=6)

    # No real ONNX model required for this structural check: force an empty detector and stub
    # qualified_cones so the replay loop is exercised without depending on model weights on disk.
    monkeypatch.setattr(dr, "qualified_cones", lambda result, cone_radius_m: [])
    result = dr.run(str(tmp_path), v_sim_mps=1.0, a_cfg_mps2=0.5, t_latency_s=0.2, margin_m=1.0)
    assert result["frames_replayed"] == 6
    assert result["brake_decision_event"] is None  # no obstacles were ever reported
    assert len(result["events"]) == 6
    assert all(math.isfinite(e["x_sim_m"]) for e in result["events"])


def test_dry_run_replay_reports_brake_when_an_obstacle_is_close(tmp_path, monkeypatch):
    make_synthetic_replay_dir(tmp_path, n_frames=6)
    # Report a fixed close obstacle on every frame -- must trigger STOP_CANDIDATE quickly.
    monkeypatch.setattr(dr, "qualified_cones",
                        lambda result, cone_radius_m: [{"x_m": 1.3, "y_m": 0.0, "radius_m": cone_radius_m}])
    result = dr.run(str(tmp_path), v_sim_mps=1.0, a_cfg_mps2=3.0, t_latency_s=0.0, margin_m=0.1)
    assert result["brake_decision_event"] is not None
    assert result["brake_decision_event"]["decision"] == "STOP_CANDIDATE"
    assert result["brake_decision_event"]["commanded_v"] == 0.0


def test_missing_replay_dir_raises_file_not_found():
    with pytest.raises(FileNotFoundError):
        dr.run("/nonexistent/path/for/sure", 1.0, 0.5, 0.2, 1.0)
