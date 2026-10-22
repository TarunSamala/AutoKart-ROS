import json
from pathlib import Path

from kart_sim.scenario import run_scenario

ROOT = Path(__file__).parents[2]


def execute(name, tmp_path):
    output = run_scenario(ROOT / "configs/scenarios" / name, tmp_path / name)
    return [json.loads(line) for line in (output / "telemetry.jsonl").read_text().splitlines()]


def test_physical_estop_during_acceleration_and_corner_has_physical_override(tmp_path):
    acceleration = execute("vcu_physical_estop_acceleration.yaml", tmp_path)
    assert any(row["subsystems"]["motor"]["actual_torque_nm"] > .5 for row in acceleration[:250])
    emergency = next(row for row in acceleration if row.get("vcu", {}).get("state") == "EMERGENCY_STOP")
    assert emergency["vcu"]["motor_enable"] is False
    assert emergency["vcu"]["contactor_open"] is True
    assert emergency["vcu"]["final_command"]["drive"] == 0
    assert acceleration[-1]["subsystems"]["motor"]["actual_torque_nm"] == 0
    assert acceleration[-1]["subsystems"]["brake"]["hydraulic_pressure_pa"] > 0

    corner = execute("vcu_physical_estop_corner.yaml", tmp_path)
    before = [row for row in corner if row["state"]["time_s"] < .25][-1]
    after = next(row for row in corner if row.get("vcu", {}).get("state") == "EMERGENCY_STOP")
    # Emergency policy holds the achieved angle rather than instantaneously snapping neutral.
    assert abs(after["vcu"]["final_command"]["steering_rad"] - before["subsystems"]["steering"]["linkage_angle_rad"]) < .03
    assert after["vcu"]["final_command"]["brake"] == 1


def test_wet_surface_changes_physics_and_safety_envelope_without_brake_fault(tmp_path):
    dry = execute("vcu_dry_surface_stop.yaml", tmp_path)
    wet = execute("vcu_wet_surface_stop.yaml", tmp_path)
    assert wet[0]["vcu"]["active_speed_limit_mps"] < dry[0]["vcu"]["active_speed_limit_mps"]
    assert wet[-1]["state"]["x_m"] > dry[-1]["state"]["x_m"]
    assert not any(fault.startswith("BRAKE-") for row in wet for fault in row.get("vcu", {}).get("faults", ()))


def test_remote_estop_split_friction_and_grade_hold(tmp_path):
    remote = execute("vcu_remote_estop.yaml", tmp_path)
    assert any(row["subsystems"]["motor"]["actual_torque_nm"] > .5 for row in remote[:250])
    emergency = next(row for row in remote if row.get("vcu", {}).get("state") == "EMERGENCY_STOP")
    assert emergency["vcu"]["final_command"]["drive"] == 0 and emergency["vcu"]["final_command"]["brake"] == 1
    assert remote[-1]["subsystems"]["motor"]["actual_torque_nm"] == 0

    split = execute("vcu_split_friction_stop.yaml", tmp_path)
    assert any(abs(row["state"]["yaw_rate_radps"]) > .001 for row in split)
    assert split[-1]["vcu"]["final_command"]["drive"] == 0
    assert not any(fault.startswith("BRAKE-") for row in split for fault in row.get("vcu", {}).get("faults", ()))

    grade = execute("vcu_grade_hold.yaml", tmp_path)
    assert grade[-1]["vcu"]["state"] == "SAFE"
    assert abs(grade[-1]["state"]["u_mps"]) < .05
    assert grade[-1]["subsystems"]["brake"]["hydraulic_pressure_pa"] > 100_000


def test_independent_safety_task_freeze_and_estop_with_stuck_torque_latch_physical_override(tmp_path):
    for scenario in ("vcu_safety_task_freeze.yaml", "vcu_estop_stuck_torque.yaml"):
        rows = execute(scenario, tmp_path)
        trip = next(row for row in rows if row.get("vcu", {}).get("independent_safety", {}).get("stop_latched"))
        safety = trip["vcu"]["independent_safety"]
        assert safety["motor_isolation_requested"] is True
        assert safety["brake_fallback_requested"] is True
        assert trip["vcu"]["motor_enable"] is False
        assert trip["vcu"]["contactor_open"] is True
        assert trip["vcu"]["final_command"]["drive"] == 0
        assert rows[-1]["subsystems"]["motor"]["actual_torque_nm"] == 0
        assert rows[-1]["subsystems"]["brake"]["hydraulic_pressure_pa"] > 100_000
        assert rows[-1]["vcu"]["independent_safety"]["stop_latched"] is True


def test_physical_estop_at_rest_is_latched_and_cannot_create_propulsion(tmp_path):
    rows = execute("vcu_physical_estop_at_rest.yaml", tmp_path)
    tripped = [row for row in rows if row.get("vcu", {}).get("independent_safety", {}).get("stop_latched")]
    assert tripped
    assert all(row["vcu"]["final_command"]["drive"] == 0 for row in tripped)
    assert all(row["vcu"]["motor_enable"] is False for row in tripped)
    assert rows[-1]["vcu"]["independent_safety"]["stop_latched"] is True
