import json

import readiness_report as rr


def test_gate_camera_rate_not_run_with_no_file(tmp_path, monkeypatch):
    monkeypatch.setattr(rr, "RESULTS", tmp_path)
    gate = rr.gate_camera_rate()
    assert gate["status"] == "NOT_RUN" and "camera_rate_check.py" in gate["command"]


def test_gate_camera_rate_pass_above_9hz(tmp_path, monkeypatch):
    monkeypatch.setattr(rr, "RESULTS", tmp_path)
    (tmp_path / "camera_rate_20260101T000000Z.json").write_text(json.dumps({"paired_rgb_depth_hz": 9.5}))
    gate = rr.gate_camera_rate()
    assert gate["status"] == "PASS" and gate["value"] == 9.5


def test_gate_camera_rate_fail_below_9hz(tmp_path, monkeypatch):
    monkeypatch.setattr(rr, "RESULTS", tmp_path)
    (tmp_path / "camera_rate_20260101T000000Z.json").write_text(json.dumps({"paired_rgb_depth_hz": 2.6}))
    gate = rr.gate_camera_rate()
    assert gate["status"] == "FAIL"


def test_gate_depth_error_all_within_budget(tmp_path, monkeypatch):
    monkeypatch.setattr(rr, "RESULTS", tmp_path)
    payload = {"results": [{"target_m": 1.2, "absolute_error_cm": 2.0, "budget_m": 0.05},
                           {"target_m": 2.0, "absolute_error_cm": 3.0, "budget_m": 0.10}]}
    (tmp_path / "depth_check_20260101T000000Z.json").write_text(json.dumps(payload))
    gate = rr.gate_depth_error()
    assert gate["status"] == "PASS"
    assert all(row["within_budget"] for row in gate["stations"])


def test_gate_depth_error_fails_when_one_station_over_budget(tmp_path, monkeypatch):
    monkeypatch.setattr(rr, "RESULTS", tmp_path)
    payload = {"results": [{"target_m": 1.2, "absolute_error_cm": 8.0, "budget_m": 0.05}]}
    (tmp_path / "depth_check_20260101T000000Z.json").write_text(json.dumps(payload))
    gate = rr.gate_depth_error()
    assert gate["status"] == "FAIL"


def test_gate_braking_not_run_when_incomplete(tmp_path, monkeypatch):
    monkeypatch.setattr(rr, "RESULTS", tmp_path)
    (tmp_path / "braking_20260101T000000Z.json").write_text(json.dumps({"completed": False}))
    gate = rr.gate_braking()
    assert gate["status"] == "NOT_RUN"


def test_gate_braking_pass_when_completed(tmp_path, monkeypatch):
    monkeypatch.setattr(rr, "RESULTS", tmp_path)
    payload = {"completed": True, "summary": {"v_run_mps": 1.1, "a_verified_mps2_suggestion": 0.6,
                                              "worst_case_mean_deceleration_mps2": 0.75}}
    (tmp_path / "braking_20260101T000000Z.json").write_text(json.dumps(payload))
    gate = rr.gate_braking()
    assert gate["status"] == "PASS" and gate["v_run_mps"] == 1.1


def test_gate_runway_not_run_without_measured_inputs():
    gate = rr.gate_runway({"v_run_mps": None}, {"suggested_t_latency_s": None})
    assert gate["status"] == "NOT_RUN"


def test_gate_runway_computes_when_inputs_present():
    braking = {"v_run_mps": 1.0, "a_verified_mps2_suggestion": 0.6}
    latency = {"suggested_t_latency_s": 0.3}
    gate = rr.gate_runway(braking, latency)
    assert gate["status"] in ("PASS", "FAIL")
    assert len(gate["rows"]) == 5


def test_build_report_not_ready_when_nothing_measured(tmp_path, monkeypatch):
    monkeypatch.setattr(rr, "RESULTS", tmp_path)
    report = rr.build_report()
    assert report["ready"] is False
    assert len(report["reasons"]) > 0


def test_gate_flags_reads_real_vcu_safety_config():
    """Not synthetic: confirms the real config's three flags are read correctly (all false pre-commissioning)."""
    gate = rr.gate_flags()
    assert gate["calibration_verified"] is False
    assert gate["vehicle_config_verified"] is False
    assert gate["braking_verified"] is False
    assert gate["status"] == "NOT_RUN"
