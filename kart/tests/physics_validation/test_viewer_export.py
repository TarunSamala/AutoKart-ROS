import json
from pathlib import Path

from kart_sim.my_track_runner import default_run_config, run_my_track_scenario
from kart_sim.viewer_export import export_replay


def test_export_replay_writes_a_loadable_json(tmp_path: Path) -> None:
    config = default_run_config("shared", seed=1)
    config.max_duration_s = 5.0
    result = run_my_track_scenario(config)
    kart = config.kart_config.kart

    out_path = export_replay(
        result, config.layout, profile="shared", seed=1,
        front_extent_m=kart.front_extent_m, rear_overhang_m=kart.rear_overhang_m, half_width_m=kart.width_m / 2,
        camera_hfov_deg=config.camera_config.hfov_deg, camera_range_m=config.camera_config.zmax_m,
        path=tmp_path / "replay.json",
    )
    payload = json.loads(out_path.read_text())
    assert payload["scenario"] == "my_track"
    assert len(payload["cones"]) == 120
    assert len(payload["frames"]) == len(result.telemetry_rows)
    assert "front_extent_m" in payload["kart_geometry"]
