import json
from pathlib import Path

from kart_sim.scenario import run_scenario

ROOT = Path(__file__).parents[2]


def execute(name, tmp_path):
    output = run_scenario(ROOT / "configs/scenarios" / name, tmp_path / name)
    return [json.loads(line) for line in (output / "telemetry.jsonl").read_text().splitlines()]


def has_fault(rows, code):
    return any(code in row.get("vcu", {}).get("faults", ()) for row in rows)


def test_steering_linkage_runaway_sensor_and_power_campaign(tmp_path):
    linkage = execute("vcu_linkage_break.yaml", tmp_path)
    assert any(abs(row["subsystems"]["steering"]["shaft_angle_rad"]) > .01 and
               abs(row["subsystems"]["steering"]["linkage_angle_rad"]) < .005 for row in linkage)
    assert has_fault(linkage, "STEER-004")
    assert linkage[-1]["vcu"]["final_command"]["drive"] == 0
    assert linkage[-1]["subsystems"]["brake"]["hydraulic_pressure_pa"] > 0

    runaway = execute("vcu_steering_runaway.yaml", tmp_path)
    assert any(abs(row["subsystems"]["steering"]["linkage_angle_rad"]) > .12 for row in runaway)
    assert has_fault(runaway, "STEER-003")
    assert any(row.get("vcu", {}).get("state") == "EMERGENCY_STOP" for row in runaway)

    drift = execute("vcu_steering_sensor_drift.yaml", tmp_path)
    assert has_fault(drift, "STEER-002")
    assert any(row["subsystems"]["brake"]["hydraulic_pressure_pa"] > 0 for row in drift)

    power = execute("vcu_steering_power_loss.yaml", tmp_path)
    assert all(abs(row["subsystems"]["steering"]["shaft_angle_rad"]) < 1e-9 for row in power)
    assert has_fault(power, "STEER-001")

    primary = execute("vcu_steering_primary_freeze.yaml", tmp_path)
    assert has_fault(primary, "STEER-002")
    offset = execute("vcu_steering_offset_jump.yaml", tmp_path)
    assert has_fault(offset, "STEER-002")
    road_wheel = execute("vcu_road_wheel_jam.yaml", tmp_path)
    assert any(
        abs(row["subsystems"]["steering"]["linkage_angle_rad"]) > .052
        and abs(row["subsystems"]["steering"]["left_wheel_angle_rad"]) < .01
        for row in road_wheel
    )
    assert has_fault(road_wheel, "STEER-002")
    assert road_wheel[-1]["vcu"]["final_command"]["drive"] == 0


def test_brake_leak_fade_and_pressure_sensor_are_distinguished(tmp_path):
    leak = execute("vcu_hydraulic_leak.yaml", tmp_path)
    assert any(row["subsystems"]["brake"]["actual_position_m"] > .001 for row in leak)
    assert all(row["subsystems"]["brake"]["hydraulic_pressure_pa"] == 0 for row in leak)
    assert all(row["subsystems"]["brake"]["brake_torque_nm"] == 0 for row in leak)
    assert has_fault(leak, "BRAKE-003")

    sensor = execute("vcu_pressure_sensor_fault.yaml", tmp_path)
    assert any(row["subsystems"]["brake"]["hydraulic_pressure_pa"] > 1_000_000 for row in sensor)
    assert has_fault(sensor, "BRAKE-SENSOR-001")
    assert not has_fault(sensor, "BRAKE-003")

    fade = execute("vcu_brake_fade.yaml", tmp_path)
    assert any(row["subsystems"]["brake"]["hydraulic_pressure_pa"] > 4_000_000 and
               row["subsystems"]["brake"]["brake_torque_nm"] < 150 for row in fade)
    assert has_fault(fade, "BRAKE-004")
    assert fade[-1]["vcu"]["final_command"]["drive"] == 0

    stall = execute("vcu_brake_stall.yaml", tmp_path)
    assert all(row["subsystems"]["brake"]["actual_position_m"] == 0 for row in stall)
    assert any(row["subsystems"]["brake"]["missed_steps"] > 0 for row in stall)
    assert has_fault(stall, "BRAKE-001")

    position_sensor = execute("vcu_brake_position_sensor_fault.yaml", tmp_path)
    assert any(row["subsystems"]["brake"]["actual_position_m"] > .001 and
               row["subsystems"]["brake"]["hydraulic_pressure_pa"] > 250_000 for row in position_sensor)
    assert has_fault(position_sensor, "BRAKE-SENSOR-002")
    assert not has_fault(position_sensor, "BRAKE-001")

    release = execute("vcu_brake_release_failure.yaml", tmp_path)
    assert has_fault(release, "BRAKE-RELEASE-001")
    assert any(row.get("vcu", {}).get("state") == "FAULT_LOCKOUT" for row in release)
    assert release[-1]["vcu"]["motor_enable"] is False

    power_loss = execute("vcu_brake_power_loss.yaml", tmp_path)
    assert all(row["subsystems"]["brake"]["actual_position_m"] == 0 for row in power_loss)
    assert has_fault(power_loss, "BRAKE-001")

    partial = execute("vcu_brake_partial_cable.yaml", tmp_path)
    nominal = execute("vcu_brake_partial_nominal.yaml", tmp_path)
    assert any(
        0 < row["subsystems"]["brake"]["cable_tension_n"]
        < row["subsystems"]["brake"]["actual_position_m"] * 20_000
        for row in partial
    )
    assert 0 < max(row["subsystems"]["brake"]["hydraulic_pressure_pa"] for row in partial) < max(
        row["subsystems"]["brake"]["hydraulic_pressure_pa"] for row in nominal
    )
    assert has_fault(partial, "BRAKE-005")
    assert min(row["vcu"]["active_speed_limit_mps"] for row in partial) <= 2.0
    assert partial[-1]["state"]["x_m"] > nominal[-1]["state"]["x_m"]
    assert partial[-1]["vcu"]["final_command"]["drive"] == 0


def test_traction_wrong_sign_loss_chain_and_current_bias_campaign(tmp_path):
    wrong = execute("vcu_wrong_torque_sign.yaml", tmp_path)
    assert any(row["subsystems"]["motor"]["actual_torque_nm"] < -.5 for row in wrong)
    assert has_fault(wrong, "MOTOR-004")
    assert any(row.get("vcu", {}).get("motor_enable") is False for row in wrong)

    loss = execute("vcu_torque_loss.yaml", tmp_path)
    assert all(abs(row["subsystems"]["motor"]["actual_torque_nm"]) < .1 for row in loss)
    assert has_fault(loss, "MOTOR-005")
    assert any(row["subsystems"]["brake"]["hydraulic_pressure_pa"] > 0 for row in loss)

    chain = execute("vcu_chain_break.yaml", tmp_path)
    assert all(row["subsystems"]["motor"]["chain_torque_nm"] == 0 for row in chain)
    assert has_fault(chain, "MOTOR-005")

    bias = execute("vcu_current_sensor_bias.yaml", tmp_path)
    assert not has_fault(bias, "MOTOR-003")
    assert any(row["subsystems"]["motor"]["actual_torque_nm"] > .5 for row in bias)

    controller_reset = execute("vcu_controller_reset.yaml", tmp_path)
    assert has_fault(controller_reset, "MOTOR-005")
    speed_sensor = execute("vcu_motor_speed_sensor_loss.yaml", tmp_path)
    assert has_fault(speed_sensor, "MOTOR-SENSOR-001")
    thermal = execute("vcu_motor_overtemperature.yaml", tmp_path)
    assert any(row["subsystems"]["motor"]["motor_temperature_c"] > 120 for row in thermal)
    assert has_fault(thermal, "MOTOR-005")
    controller_thermal = execute("vcu_controller_overtemperature.yaml", tmp_path)
    assert any(row["subsystems"]["motor"]["controller_temperature_c"] > 120 for row in controller_thermal)
    assert has_fault(controller_thermal, "MOTOR-005")
    bms = execute("vcu_bms_shutdown.yaml", tmp_path)
    assert all(row["subsystems"]["battery"]["bms_shutdown"] for row in bms)
    assert has_fault(bms, "MOTOR-005")
    undervoltage = execute("vcu_battery_undervoltage.yaml", tmp_path)
    assert min(row["subsystems"]["battery"]["open_circuit_voltage_v"] for row in undervoltage) < 30
    assert any(row["subsystems"]["battery"]["bms_shutdown"] for row in undervoltage)
    assert has_fault(undervoltage, "MOTOR-005")

    conflict = execute("vcu_brake_propulsion_conflict.yaml", tmp_path)
    assert has_fault(conflict, "BRAKE-PROPULSION-CONFLICT")
    conflict_row = next(row for row in conflict if "BRAKE-PROPULSION-CONFLICT" in row.get("vcu", {}).get("faults", ()))
    assert conflict_row["vcu"]["final_command"]["drive"] == 0
    assert conflict_row["vcu"]["motor_enable"] is False


def test_high_speed_steering_fault_uses_emergency_policy(tmp_path):
    rows = execute("vcu_high_speed_steering_jam.yaml", tmp_path)
    assert has_fault(rows, "STEER-001")
    emergency = next(row for row in rows if row.get("vcu", {}).get("state") == "EMERGENCY_STOP")
    assert emergency["vcu"]["motor_enable"] is False
    assert emergency["vcu"]["final_command"]["brake"] == 1


def test_excessive_steering_request_is_bounded_by_vcu_and_mechanical_limits(tmp_path):
    rows = execute("vcu_steering_mechanical_limit.yaml", tmp_path)
    assert all(abs(row["vcu"]["final_command"]["steering_rad"]) <= .6 for row in rows)
    assert all(
        row["subsystems"]["steering"]["left_wheel_angle_rad"] <= .5585053606
        for row in rows
    )
    assert has_fault(rows, "STEER-LIMIT-001")


def test_stuck_torque_with_welded_contactor_reports_isolation_failure(tmp_path):
    rows = execute("vcu_stuck_torque_welded_contactor.yaml", tmp_path)
    assert has_fault(rows, "MOTOR-003")
    assert has_fault(rows, "POWER-004")
    lockout = next(row for row in rows if row.get("vcu", {}).get("state") == "FAULT_LOCKOUT")
    assert lockout["vcu"]["motor_enable"] is False
    assert lockout["vcu"]["contactor_open"] is True
    assert lockout["subsystems"]["battery"]["contactor_state"] == "WELDED"
    assert lockout["vcu"]["final_command"]["brake"] == 1


def test_nominal_torque_removal_decays_without_false_stuck_torque(tmp_path):
    rows = execute("vcu_command_timeout.yaml", tmp_path)
    assert any(row["subsystems"]["motor"]["actual_torque_nm"] > .5 for row in rows)
    assert rows[-1]["vcu"]["final_command"]["drive"] == 0
    assert rows[-1]["subsystems"]["motor"]["actual_torque_nm"] == 0
    assert not has_fault(rows, "MOTOR-003")


def test_stuck_torque_detection_survives_wheel_spin_and_wet_braking(tmp_path):
    spin = execute("vcu_stuck_torque_wheel_spin.yaml", tmp_path)
    assert has_fault(spin, "MOTOR-003")
    assert any(
        abs(row["subsystems"]["four_wheel"]["wheels"]["rear_left"]["slip_ratio"]) > .01
        for row in spin
    )
    wet = execute("vcu_stuck_torque_wet.yaml", tmp_path)
    assert has_fault(wet, "MOTOR-003")
    assert any(row["vcu"]["final_command"]["brake"] == 1 for row in wet)
    assert not has_fault(wet, "BRAKE-003")
