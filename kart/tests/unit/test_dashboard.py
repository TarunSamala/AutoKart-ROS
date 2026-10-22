from __future__ import annotations

import json

import pytest

from kart_sim.dashboard import DashboardExporter, TelemetryRecord, TelemetryStore, compare_by_track_distance
from kart_sim.types import SimulationError


def record(t: float, speed: float, *, fault: bool = False) -> TelemetryRecord:
    return TelemetryRecord(t, "sim", {"state": {"time_s": t, "u_mps": speed}, "vcu": {"state": "EMERGENCY_STOP" if fault else "AUTONOMOUS", "faults": ("MOTOR-003",) if fault else ()}})


def test_bounded_decimation_fault_selection_and_connection_loss():
    store = TelemetryStore(3)
    for index in range(5): store.append(record(index, index, fault=index == 3))
    assert len(store.records) == 3 and len(store.decimate(2)) == 2
    assert len(store.fault_events()) == 1
    store.connection_lost(); assert not store.connection_alive
    with pytest.raises(SimulationError): store.decimate(0)


def test_jsonl_comparison_and_exports(tmp_path):
    run = tmp_path / "telemetry.jsonl"
    run.write_text("\n".join(json.dumps({"state": {"time_s": t, "u_mps": speed}, "track_s_m": t}) for t, speed in ((0, 1), (1, 2))) + "\n")
    store = TelemetryStore.from_jsonl(run)
    comparison = compare_by_track_distance(store.records, [record(0, .5), record(1, 1.0)])
    assert comparison[-1]["speed_delta_mps"] == 1.0
    html = DashboardExporter.export_html(store, tmp_path / "report.html")
    assert "VCU state" in html.read_text()
    assert DashboardExporter.export_csv(store, tmp_path / "report.csv").exists()
    assert DashboardExporter.export_json(store, tmp_path / "report.json").exists()
