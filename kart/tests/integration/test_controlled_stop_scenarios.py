import json
from pathlib import Path

from kart_sim.scenario import run_scenario

ROOT = Path(__file__).parents[2]


def rows_for(name: str, tmp_path):
    output = run_scenario(ROOT / "configs/scenarios" / name, tmp_path / name)
    return [json.loads(line) for line in (output / "telemetry.jsonl").read_text().splitlines()]


def test_nominal_controlled_stop_reaches_measured_hold_and_safe(tmp_path):
    rows = rows_for("vcu_controlled_stop_nominal.yaml", tmp_path)
    assert any(row.get("vcu", {}).get("controlled_stop_phase") == "COMPLETE" for row in rows)
    final = rows[-1]
    assert final["vcu"]["state"] == "SAFE"
    assert final["vcu"]["final_command"]["drive"] == 0
    assert abs(final["state"]["u_mps"]) < .05
    assert final["subsystems"]["motor"]["actual_torque_nm"] == 0
    assert final["subsystems"]["motor"]["motor_current_a"] < 1
    assert final["subsystems"]["brake"]["hydraulic_pressure_pa"] >= 100_000
    assert all(abs(wheel["angular_speed_radps"]) < .05 / .25 for wheel in final["subsystems"]["four_wheel"]["wheels"].values())


def test_false_zero_wheels_cannot_declare_stop_while_vehicle_moves(tmp_path):
    rows = rows_for("vcu_false_zero_stop.yaml", tmp_path)
    safe_rows = [row for row in rows if row.get("vcu", {}).get("state") == "SAFE"]
    assert safe_rows
    assert all(abs(row["state"]["u_mps"]) < .05 for row in safe_rows)


def test_controlled_stop_timeout_escalates_with_physical_brake_failure(tmp_path):
    rows = rows_for("vcu_stop_timeout.yaml", tmp_path)
    failures = [row for row in rows if "CONTROLLED_STOP_FAILED" in row.get("vcu", {}).get("faults", ())]
    assert failures
    assert failures[0]["vcu"]["state"] == "EMERGENCY_STOP"
    assert failures[0]["vcu"]["motor_enable"] is False
    assert failures[0]["vcu"]["contactor_open"] is True
    assert failures[0]["vcu"]["final_command"]["brake"] == 1


def test_controlled_stop_in_corner_holds_achieved_steering_then_stops(tmp_path):
    rows = rows_for("vcu_controlled_stop_corner.yaml", tmp_path)
    transition = next(index for index, row in enumerate(rows) if row.get("vcu", {}).get("state") == "CONTROLLED_STOP")
    before = rows[max(0, transition - 1)]["subsystems"]["steering"]["linkage_angle_rad"]
    after = rows[transition]["vcu"]["final_command"]["steering_rad"]
    assert abs(after - before) < .03
    assert rows[transition]["vcu"]["final_command"]["drive"] == 0
    assert not any(
        fault.startswith("STEER-")
        for row in rows
        for fault in row.get("vcu", {}).get("faults", ())
    )
    assert rows[-1]["vcu"]["state"] == "SAFE"
