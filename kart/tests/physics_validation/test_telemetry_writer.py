import csv
import json
from pathlib import Path

from kart_sim.my_track_runner import default_run_config, run_my_track_scenario
from kart_sim.telemetry_writer import write_my_track_run


def test_write_my_track_run_produces_csvs_and_summary(tmp_path: Path) -> None:
    config = default_run_config("shared", seed=1)
    config.max_duration_s = 5.0
    result = run_my_track_scenario(config)

    run_dir = write_my_track_run(result, results_root=tmp_path, profile="shared", seed=1, timestamp="20260101T000000Z")
    assert run_dir == tmp_path / "my_track" / "20260101T000000Z_shared_1"

    with (run_dir / "telemetry_10hz.csv").open() as stream:
        rows_10hz = list(csv.DictReader(stream))
    assert len(rows_10hz) == len(result.telemetry_rows)
    assert "steer_rate_dps" in rows_10hz[0] and "clearance_m" in rows_10hz[0] and "dac_V" in rows_10hz[0]

    with (run_dir / "telemetry_1hz.csv").open() as stream:
        rows_1hz = list(csv.DictReader(stream))
    assert 1 <= len(rows_1hz) <= len(rows_10hz)

    summary = json.loads((run_dir / "summary.json").read_text())
    assert summary["metrics"]["scenario"] == "my_track"
    assert isinstance(summary["events"], list)
