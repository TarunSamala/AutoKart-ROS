from pathlib import Path
import json
from kart_sim.scenario import run_scenario
ROOT=Path(__file__).parents[2]
def test_vcu_scenario_routes_outputs_and_timeout_brakes(tmp_path):
 out=run_scenario(ROOT/"configs/scenarios/vcu_command_timeout.yaml",tmp_path)
 rows=[json.loads(x) for x in (out/"telemetry.jsonl").read_text().splitlines()]
 assert any("vcu" in r and r["vcu"]["state"]=="CONTROLLED_STOP" for r in rows)
 assert rows[-1]["subsystems"]["brake"]["hydraulic_pressure_pa"]>0

def test_vcu_freeze_and_physical_estop_override_actuators(tmp_path):
 for name in ("vcu_task_freeze.yaml","vcu_physical_estop.yaml"):
  out=run_scenario(ROOT/"configs/scenarios"/name,tmp_path/name)
  rows=[json.loads(x) for x in (out/"telemetry.jsonl").read_text().splitlines()]
  assert any(r.get("vcu",{}).get("state")=="EMERGENCY_STOP" for r in rows)
  assert rows[-1]["subsystems"]["brake"]["hydraulic_pressure_pa"]>0
  assert rows[-1]["subsystems"]["motor"]["actual_torque_nm"]==0

def test_steering_jam_feedback_triggers_physical_controlled_stop(tmp_path):
 out=run_scenario(ROOT/"configs/scenarios/vcu_steering_jam.yaml",tmp_path)
 rows=[json.loads(x) for x in (out/"telemetry.jsonl").read_text().splitlines()]
 assert any(r.get("vcu",{}).get("state")=="CONTROLLED_STOP" and "STEER-001" in r["vcu"]["faults"] for r in rows)
 assert rows[-1]["subsystems"]["brake"]["hydraulic_pressure_pa"]>0

def test_cable_break_is_physical_and_escalates_from_position_pressure_mismatch(tmp_path):
 out=run_scenario(ROOT/"configs/scenarios/vcu_brake_cable_break.yaml",tmp_path)
 rows=[json.loads(x) for x in (out/"telemetry.jsonl").read_text().splitlines()]
 assert any(r["subsystems"]["brake"]["actual_position_m"]>.001 and r["subsystems"]["brake"]["cable_tension_n"]==0 for r in rows)
 assert any(r.get("vcu",{}).get("state")=="EMERGENCY_STOP" and "BRAKE-003" in r["vcu"]["faults"] for r in rows)
 assert rows[-1]["subsystems"]["motor"]["actual_torque_nm"]==0

def test_stuck_positive_torque_is_detected_and_physically_isolated(tmp_path):
 out=run_scenario(ROOT/"configs/scenarios/vcu_motor_stuck_torque.yaml",tmp_path)
 rows=[json.loads(x) for x in (out/"telemetry.jsonl").read_text().splitlines()]
 assert any(r["subsystems"]["motor"]["actual_torque_nm"]>.5 for r in rows)
 assert any(r.get("vcu",{}).get("state")=="EMERGENCY_STOP" and "MOTOR-003" in r["vcu"]["faults"] for r in rows)
 assert any(r.get("vcu",{}).get("motor_enable") is False and r.get("vcu",{}).get("contactor_open") for r in rows)
 assert rows[-1]["subsystems"]["brake"]["hydraulic_pressure_pa"]>0
