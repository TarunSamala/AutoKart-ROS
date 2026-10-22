import csv
import io

import pytest

from autokart_vehicle_bridge.run_log import FIELDS, RunCsvWriter, final_bumper_gap


def test_each_row_has_requested_columns_and_gap_uses_last_real_sighting():
    file = io.StringIO()
    writer = RunCsvWriter(file)
    writer.write({"time": 1, "cone_forward_distance": 3.0, "bumper_to_cone": 1.9,
                  "speed": 0.5, "dac_mv": 0, "decision": "STOP_CANDIDATE", "state": "SAFE",
                  "axle_counts": 100, "dead_reckoned": False})
    writer.write({"time": 2, "cone_forward_distance": 2.0, "bumper_to_cone": 0.9,
                  "speed": 0.1, "dac_mv": 0, "decision": "STOP_CANDIDATE", "state": "SAFE",
                  "axle_counts": 150, "dead_reckoned": True})
    writer.write({"time": 3, "speed": 0, "dac_mv": 0, "decision": "VCU_TEL",
                  "state": "SAFE", "axle_counts": 200})
    rows = list(csv.DictReader(io.StringIO(file.getvalue())))
    assert tuple(rows[0]) == FIELDS
    assert len(rows) == 3
    assert final_bumper_gap(rows, 100.0) == pytest.approx(0.9)
    reversed_rows = [dict(row, axle_counts=str(300 - int(row["axle_counts"]))) for row in rows]
    assert final_bumper_gap(reversed_rows, 100.0, -1) == pytest.approx(0.9)


def test_gap_refuses_uncalibrated_counts_or_missing_sighting():
    with pytest.raises(ValueError, match="calibrated"):
        final_bumper_gap([], 0)
    with pytest.raises(ValueError, match="real cone sighting"):
        final_bumper_gap([{"axle_counts": "10"}], 100)
    rows = [{"bumper_to_cone": "1.0", "axle_counts": "-1", "dac_mv": "0",
             "dead_reckoned": "False"}]
    with pytest.raises(ValueError, match="no axle encoder"):
        final_bumper_gap(rows, 100)
