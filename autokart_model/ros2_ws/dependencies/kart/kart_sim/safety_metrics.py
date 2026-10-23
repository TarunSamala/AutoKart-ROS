"""Explainable safety-event metrics derived from scenario telemetry."""
from __future__ import annotations
from dataclasses import dataclass


@dataclass(frozen=True)
class SafetyMetrics:
    fault_detection_s: float | None
    safety_action_s: float | None
    torque_zero_s: float | None
    brake_response_s: float | None
    stopped_s: float | None
    distance_after_detection_m: float | None
    maximum_speed_mps: float
    invariant_violations: tuple[str, ...]


def derive_safety_metrics(rows: list[dict]) -> SafetyMetrics:
    def first(predicate):
        return next((row for row in rows if predicate(row)), None)

    fault = first(lambda row: bool(row.get("vcu", {}).get("faults")))
    action = first(lambda row: row.get("vcu", {}).get("state") in {"CONTROLLED_STOP", "EMERGENCY_STOP", "FAULT_LOCKOUT"})
    torque_zero = first(lambda row: action is not None and row["state"]["time_s"] >= action["state"]["time_s"]
                        and abs(row.get("subsystems", {}).get("motor", {}).get("actual_torque_nm", 0.0)) < .1)
    brake = first(lambda row: row.get("subsystems", {}).get("brake", {}).get("hydraulic_pressure_pa", 0.0) >= 100_000)
    stopped = first(lambda row: abs(row.get("state", {}).get("u_mps", 1e9)) < .05
                    and abs(row.get("subsystems", {}).get("motor", {}).get("actual_torque_nm", 1e9)) < .1)
    violations = []
    for row in rows:
        vcu = row.get("vcu", {})
        if vcu.get("state") in {"EMERGENCY_STOP", "FAULT_LOCKOUT"} and vcu.get("final_command", {}).get("drive", 0) > 0:
            violations.append("positive_propulsion_in_critical_state")
    fault_time = fault["state"]["time_s"] if fault else None
    return SafetyMetrics(
        fault_time,
        action["state"]["time_s"] if action else None,
        torque_zero["state"]["time_s"] if torque_zero else None,
        brake["state"]["time_s"] if brake else None,
        stopped["state"]["time_s"] if stopped else None,
        None if not fault or not stopped else stopped["state"]["x_m"] - fault["state"]["x_m"],
        max((abs(row["state"]["u_mps"]) for row in rows), default=0.0),
        tuple(sorted(set(violations))),
    )
