import json
import os

from calibration_candidate import evaluate


def test_not_run_when_no_depth_check_result(tmp_path):
    result = evaluate(tmp_path)
    assert result["status"] == "NOT_RUN"
    assert "depth_check.py --fit-pitch" in result["command"]


def test_not_run_when_result_has_no_fit(tmp_path):
    (tmp_path / "depth_check_20260101T000000Z.json").write_text(json.dumps({"results": []}))
    result = evaluate(tmp_path)
    assert result["status"] == "NOT_RUN"


def test_fail_when_fit_unreliable(tmp_path):
    payload = {"pitch_fit": {"recommendation": "NO_CONFIG_CHANGE_FIT_UNRELIABLE"}}
    (tmp_path / "depth_check_20260101T000000Z.json").write_text(json.dumps(payload))
    result = evaluate(tmp_path)
    assert result["status"] == "FAIL"


def test_pass_with_candidate_lines_when_fit_is_provisional(tmp_path):
    payload = {"pitch_fit": {
        "recommendation": "PROVISIONAL_FIT_REVIEW",
        "fitted": {"pitch_deg": 15.23, "rmse_m": 0.02},
        "configured": {"pitch_deg": 17.8},
        "old_ground_plane": {"pitch_deg": 11.4},
    }}
    (tmp_path / "depth_check_20260101T000000Z.json").write_text(json.dumps(payload))
    result = evaluate(tmp_path)
    assert result["status"] == "PASS"
    assert result["fitted_pitch_deg"] == 15.23
    assert any("15.23" in line for line in result["paste_lines"])
    assert "calibration_verified" not in "".join(result["paste_lines"])


def test_picks_the_latest_result_file(tmp_path):
    (tmp_path / "depth_check_20260101T000000Z.json").write_text(
        json.dumps({"pitch_fit": {"recommendation": "NO_CONFIG_CHANGE_FIT_UNRELIABLE"}}))
    (tmp_path / "depth_check_20260102T000000Z.json").write_text(json.dumps({
        "pitch_fit": {"recommendation": "PROVISIONAL_FIT_REVIEW",
                      "fitted": {"pitch_deg": 16.0, "rmse_m": 0.01}, "configured": {}, "old_ground_plane": {}}}))
    result = evaluate(tmp_path)
    assert result["status"] == "PASS" and "20260102" in result["source_file"]


def test_picks_latest_result_from_integrated_session(tmp_path):
    old = tmp_path / "depth_check_20260101T000000Z.json"
    old.write_text(json.dumps({"pitch_fit": {"recommendation": "NO_CONFIG_CHANGE_FIT_UNRELIABLE"}}))
    session = tmp_path / "braking_session_20260102T000000Z"
    session.mkdir()
    current = session / "depth_check_20260102T000000Z.json"
    current.write_text(json.dumps({
        "pitch_fit": {"recommendation": "PROVISIONAL_FIT_REVIEW",
                      "fitted": {"pitch_deg": 16.0, "rmse_m": 0.01},
                      "configured": {}, "old_ground_plane": {}}}))
    current_mtime = old.stat().st_mtime_ns + 1_000_000_000
    os.utime(current, ns=(current_mtime, current_mtime))
    result = evaluate(tmp_path)
    assert result["status"] == "PASS" and "braking_session" in result["source_file"]
