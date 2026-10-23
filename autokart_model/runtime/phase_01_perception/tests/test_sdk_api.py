import json
from pathlib import Path

import pytest

from autokart_perception import (Cone, ConfigurationError, PerceptionGeometrySDK,
                                 PoseEstimate, process_cones)
from autokart_perception.pipeline import Pipeline
from autokart_perception.recording import Recorder, Replay
from autokart_perception.synthetic import Synthetic


def corridor():
    return [Cone(index, "blue" if index % 2 == 0 else "yellow", .95,
                 (x, 1.5 if index % 2 == 0 else -1.5), .05,
                 (x * x + 1.5 ** 2) ** .5, (0, 0, 1, 1))
            for x, start in ((2, 0), (5, 2), (8, 4))
            for index in (start, start + 1)]


def test_public_geometry_api_has_typed_stages_and_json_contract():
    now = 1_000_000_000
    result = process_cones(corridor(), PoseEstimate(0, 0, 0, 0, now, .05), timestamp_ns=now)
    assert result.geometry.delaunay_edges and result.geometry.gabriel_edges
    assert result.geometry.boundaries.left_ids and result.geometry.boundaries.right_ids
    assert len(result.geometry.gates) == 3
    assert result.geometry.centerline_m == ((2.0, 0.0), (5.0, 0.0), (8.0, 0.0))
    assert result.planning.valid and result.processing_ms >= 0
    assert result.configuration_sha256 and result.algorithm_version
    assert not result.vehicle_authorized
    assert json.loads(json.dumps(result.to_dict()))["geometry"]["gates"]


def test_public_api_config_errors_are_structured():
    with pytest.raises(ConfigurationError) as error:
        PerceptionGeometrySDK({"min_track_width_m": -1})
    assert error.value.code == "CONFIG_INVALID"


def test_public_frame_api_replays_existing_perception_pipeline(tmp_path):
    sdk = PerceptionGeometrySDK()
    synthetic = Pipeline(None, sdk.config, synthetic=True).process(Synthetic().read())
    path = tmp_path / "capture"
    Recorder(path).write(synthetic)
    frame = Replay(path).read()
    result = sdk.process_frame(frame)
    assert result.perception["status"] == "RECORDED_OBSERVATIONS"
    assert result.perception["cone_count"] == 6
    assert result.geometry.delaunay_edges and result.geometry.gabriel_edges
    assert result.geometry.gates and result.geometry.centerline_m
    assert not result.planning.valid
    assert "INVALID_POSE" in result.reasons
    assert not result.control_candidate.valid
    assert not result.vehicle_authorized
