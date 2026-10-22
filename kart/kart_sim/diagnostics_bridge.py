"""Transport-neutral projection of faults into ROS/dashboard/replay channels."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class DiagnosticStatus:
    name: str
    level: str
    message: str
    hardware_id: str
    values: tuple[tuple[str, str], ...]


def project_faults(
    namespace: str,
    timestamp_s: float,
    *,
    vcu_state: str,
    vcu_faults: tuple[str, ...],
    simulation_faults: tuple[str, ...] = (),
) -> tuple[DiagnosticStatus, ...]:
    combined = tuple(dict.fromkeys((*vcu_faults, *simulation_faults)))
    if not combined:
        return (DiagnosticStatus(
            f"{namespace}/safety",
            "OK",
            vcu_state,
            namespace,
            (("timestamp_s", f"{timestamp_s:.9f}"),),
        ),)
    level = "ERROR" if vcu_state in {"EMERGENCY_STOP", "FAULT_LOCKOUT"} else "WARN"
    return tuple(
        DiagnosticStatus(
            f"{namespace}/fault/{fault}",
            level,
            fault,
            namespace,
            (
                ("timestamp_s", f"{timestamp_s:.9f}"),
                ("vcu_state", vcu_state),
                ("visible_in", "simulation,ros_diagnostics,dashboard,logs,replay,report"),
            ),
        )
        for fault in combined
    )

