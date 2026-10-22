"""Mission state machine.

States (exactly as specified):
INITIALIZING, CAMERA_READY, PERCEPTION_READY, PATH_READY, DISARMED, DRIVING,
OBSTACLE_DETECTED, STOP_REQUESTED, STOPPED, FAULT.

This state machine decides a software request and a target speed fed to
control/throttle_preview.py. The Pico experimental harness does not transmit
the request to the VCU.
STOPPED here means "this software stopped asking for motion" -- it is
explicitly NOT a claim that the vehicle has physically stopped; that requires
independent telemetry/physical evidence per docs/BENCH_TESTS.md, which the
caller is responsible for checking before trusting this state as fact.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

STATES = ("INITIALIZING", "CAMERA_READY", "PERCEPTION_READY", "PATH_READY", "DISARMED",
          "DRIVING", "OBSTACLE_DETECTED", "STOP_REQUESTED", "STOPPED", "FAULT")

DECISION_STOP, DECISION_OBSERVE, DECISION_AT_TARGET = 0, 1, 2


@dataclass
class StopControllerInputs:
    camera_ok: bool
    frame_stale: bool
    depth_valid: bool
    corridor_status: str          # Corridor.status from planning.corridor_planner
    obstacle_blocked: bool
    obstacle_clearance_qualified: bool
    obstacle_forward_m: float | None
    vcu_link_ok: bool
    safety_state: str             # result.diagnostics['safety_state'] from autokart_perception
    vehicle_authorized: bool      # result.diagnostics['vehicle_authorized'] from autokart_perception


@dataclass
class StopControllerOutput:
    state: str
    decision: int
    target_speed_mps: float
    reason: str


class StopController:
    def __init__(self):
        self.state = "INITIALIZING"
        self._fault_reason: str | None = None

    def reset_fault(self):
        if self.state == "FAULT":
            self.state = "INITIALIZING"
            self._fault_reason = None

    def step(self, inputs: StopControllerInputs, cruise_speed_mps: float = 0.3) -> StopControllerOutput:
        s = self.state

        if s == "FAULT":
            return StopControllerOutput(s, DECISION_STOP, 0.0, self._fault_reason or "FAULT")

        if not inputs.vcu_link_ok and s not in ("INITIALIZING", "CAMERA_READY", "PERCEPTION_READY", "PATH_READY"):
            self._fault_reason = "VCU_LINK_LOST"
            self.state = "FAULT"
            return StopControllerOutput("FAULT", DECISION_STOP, 0.0, self._fault_reason)

        if not inputs.camera_ok or inputs.frame_stale:
            self.state = "INITIALIZING" if s in ("INITIALIZING", "CAMERA_READY") else "STOP_REQUESTED"
            return StopControllerOutput(self.state, DECISION_STOP, 0.0, "CAMERA_NOT_READY_OR_STALE")

        if s in ("INITIALIZING",):
            self.state = "CAMERA_READY"
            s = self.state

        if not inputs.depth_valid:
            self.state = "CAMERA_READY" if s == "CAMERA_READY" else "STOP_REQUESTED"
            return StopControllerOutput(self.state, DECISION_STOP, 0.0, "INVALID_DEPTH")

        if s == "CAMERA_READY":
            self.state = "PERCEPTION_READY"
            s = self.state

        if inputs.corridor_status in ("NO_PATH", "AMBIGUOUS") or inputs.safety_state in (
                "STALE", "SENSOR_FAULT", "AMBIGUOUS"):
            self.state = "PERCEPTION_READY" if s == "PERCEPTION_READY" else "STOP_REQUESTED"
            return StopControllerOutput(self.state, DECISION_STOP, 0.0,
                                         f"CORRIDOR_{inputs.corridor_status}_SAFETY_{inputs.safety_state}")

        if s == "PERCEPTION_READY":
            self.state = "PATH_READY"
            s = self.state

        if not inputs.vehicle_authorized:
            # The upstream perception safety assessment itself withholds authorization
            # (e.g. calibration unverified, stale, ambiguous topology). Stay parked.
            self.state = "DISARMED"
            return StopControllerOutput(self.state, DECISION_STOP, 0.0, "PERCEPTION_NOT_AUTHORIZED")

        if inputs.obstacle_blocked:
            if inputs.obstacle_clearance_qualified and (inputs.obstacle_forward_m or 0) > 1.0:
                self.state = "OBSTACLE_DETECTED"
                return StopControllerOutput(self.state, DECISION_OBSERVE, cruise_speed_mps, "OBSTACLE_AHEAD_CLEAR_TO_OBSERVE")
            self.state = "STOP_REQUESTED"
            return StopControllerOutput(self.state, DECISION_STOP, 0.0, "OBSTACLE_IN_STOP_ZONE")

        if s in ("PATH_READY", "DISARMED", "STOPPED", "STOP_REQUESTED", "OBSTACLE_DETECTED"):
            self.state = "DRIVING"
            return StopControllerOutput(self.state, DECISION_OBSERVE, cruise_speed_mps, "PATH_CLEAR")

        self.state = "DRIVING"
        return StopControllerOutput(self.state, DECISION_OBSERVE, cruise_speed_mps, "PATH_CLEAR")

    def request_stop(self, reason: str = "OPERATOR_STOP_REQUEST") -> StopControllerOutput:
        self.state = "STOP_REQUESTED"
        return StopControllerOutput(self.state, DECISION_STOP, 0.0, reason)

    def mark_stopped(self, reason: str = "STOP_ACKNOWLEDGED") -> StopControllerOutput:
        if self.state == "STOP_REQUESTED":
            self.state = "STOPPED"
        return StopControllerOutput(self.state, DECISION_STOP, 0.0, reason)
