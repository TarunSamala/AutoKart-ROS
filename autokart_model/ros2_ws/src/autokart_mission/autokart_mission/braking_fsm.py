"""Explicit mission sequencing around the reused braking geometry/physics.

This owns mission progress only. It never arms safety or sends actuator commands.
A stop remains latched until an explicit reset with current verified stationarity.
"""
from __future__ import annotations


class BrakingFSM:
    def __init__(self):
        self.state = "WAITING"
        self.start_requested = False
        self.stop_latched = False
        self.fault = None

    def request_start(self):
        if self.state != "READY" or self.stop_latched:
            return False
        self.start_requested = True
        return True

    def reset(self, *, disarmed, verification):
        if not disarmed or not verification.get("verified"):
            return False
        self.__init__()
        return True

    def update(self, *, corridor_valid, ready, stop_trigger, verification,
               emergency=False, interrupted=False):
        if emergency:
            self.state, self.stop_latched, self.fault = "FAULT", True, "EMERGENCY_STOP"
        elif interrupted:
            self.state, self.stop_latched, self.fault = "FAULT", True, "BRAKING_INTERRUPTED"
        elif self.state == "FAULT":
            pass
        elif self.stop_latched:
            if self.state == "STOP_TRIGGERED":
                self.state = "STOPPING"
            elif verification.get("verified"):
                self.state = "STOP_VERIFIED"
            else:
                self.state = "STOP_UNVERIFIED"
        elif self.state == "RUNNING" and (stop_trigger or not corridor_valid or not ready):
            self.state, self.stop_latched = "STOP_TRIGGERED", True
        elif self.state == "WAITING":
            if corridor_valid:
                self.state = "CORRIDOR_VALID"
        elif self.state == "CORRIDOR_VALID":
            self.state = "READY" if corridor_valid and ready else "WAITING"
        elif self.state == "READY":
            if not corridor_valid or not ready:
                self.state, self.start_requested = "WAITING", False
            elif self.start_requested:
                self.state, self.start_requested = "RUNNING", False
        # STOPPING is observable for at least one cycle before verification.
        return {"state": self.state, "stop_latched": self.stop_latched,
                "stop_verification": verification,
                "reason_codes": [self.fault] if self.fault else [],
                "vehicle_authorized": False}
