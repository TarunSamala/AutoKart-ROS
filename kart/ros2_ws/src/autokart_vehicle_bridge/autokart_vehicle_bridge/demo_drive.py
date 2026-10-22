"""Pure logic for DEMO DRIVE (not autonomy). No ROS/serial dependency; owned by the vehicle bridge
node, which feeds it heartbeat/request events and reads compute() every tick. compute()'s output
still passes through the existing CommandGate (ARMED + fresh-command + 2 V ceiling) in node.py's
_tick() exactly like the autonomy candidate -- this class does not and cannot bypass that gate.

Direction: vcu_firmware's F/R pin (GPIO39) is set once at boot (see config.h's
GPIO_FR_FORWARD_LEVEL comment) and the wire protocol's CMD frame has no direction field at all
(see docs/VCU_PROTOCOL.md) -- the firmware was not touched to add runtime direction switching,
per the hard rule against changing firmware/GPIO polarity. "reverse" is therefore refused here,
explicitly, rather than silently accepted and having no physical effect: an operator must never be
told a reverse request succeeded when the kart cannot do it. The direction-lock-while-moving rule
below still exists and is tested, ready for a real reverse capability if the firmware ever adds a
protocol field for it.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .protocol import THROTTLE_MAX_MV

HEARTBEAT_TIMEOUT_S = 0.5   # browser-tab dead-man; distinct from firmware's own 300 ms CMD_TIMEOUT_MS
MAX_DURATION_S = 10.0
SUPPORTED_DIRECTIONS = ("forward",)  # "reverse" is refused: not wired, see module docstring


@dataclass
class DemoDriveOutput:
    throttle_mv: int
    brake_pct: int
    reason: str


class DemoDriveController:
    def __init__(self, throttle_mv: int = THROTTLE_MAX_MV,
                 heartbeat_timeout_s: float = HEARTBEAT_TIMEOUT_S,
                 max_duration_s: float = MAX_DURATION_S):
        self.throttle_mv = min(int(throttle_mv), THROTTLE_MAX_MV)
        self.heartbeat_timeout_s = heartbeat_timeout_s
        self.max_duration_s = max_duration_s
        self.active = False            # mirrors autokart_vcu_safety's demo_drive_active (ARMED-gated)
        self.driving = False            # an operator drive request is currently in effect
        self.last_heartbeat: Optional[float] = None
        self.drive_until: Optional[float] = None
        self.moving_direction: Optional[str] = None  # set only while throttle > 0 was last commanded

    def set_active(self, active: bool) -> None:
        """Mirrors autokart_vcu_safety's demo_drive_active. Losing it (disarm, fault, stop,
        reset_fault clearing a latch, or simply never having been granted) always stops driving."""
        self.active = active
        if not active:
            self.driving = False
            self.drive_until = None
            self.moving_direction = None

    def heartbeat(self, now: float) -> None:
        """Call once per message received from the browser's demo-drive websocket channel."""
        self.last_heartbeat = now

    def request_drive(self, now: float, direction: str, duration_s: float) -> tuple[bool, str]:
        if direction not in SUPPORTED_DIRECTIONS:
            return False, "REVERSE_NOT_SUPPORTED_BY_FIRMWARE" if direction == "reverse" else "INVALID_DIRECTION"
        if not (0.0 < duration_s <= self.max_duration_s):
            return False, f"DURATION_OUT_OF_RANGE (0, {self.max_duration_s}] s"
        if not self.active:
            return False, "DEMO_DRIVE_NOT_ACTIVE"
        if self.moving_direction is not None and direction != self.moving_direction:
            return False, "DIRECTION_CHANGE_WHILE_MOVING_REFUSED"
        self.drive_until = now + duration_s
        self.driving = True
        return True, "OK"

    def request_stop(self) -> None:
        """Always works, from any state."""
        self.driving = False
        self.drive_until = None
        self.moving_direction = None

    def compute(self, now: float) -> DemoDriveOutput:
        if not self.active or not self.driving:
            return DemoDriveOutput(0, 100, "not driving")
        if self.last_heartbeat is None or now - self.last_heartbeat > self.heartbeat_timeout_s:
            self.driving = False
            self.moving_direction = None
            return DemoDriveOutput(0, 100, "browser heartbeat lost")
        if self.drive_until is not None and now >= self.drive_until:
            self.driving = False
            self.moving_direction = None
            return DemoDriveOutput(0, 100, "timed move complete")
        self.moving_direction = "forward"
        return DemoDriveOutput(self.throttle_mv, 0, "driving forward")
