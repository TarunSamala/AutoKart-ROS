"""Laptop side of the AutoKart VCU serial protocol (see docs/VCU_PROTOCOL.md). No ROS dependencies.

Matches vcu_firmware/include/config.h and vcu_firmware/src/protocol.cpp exactly; this is the single source
of truth for the wire format on the laptop side. This is a new protocol for the new vcu_firmware/ firmware,
not the NOOP-only micro-ROS bench packet decoded by autokart_diagnostics.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

PROTO_VERSION = 2
THROTTLE_MAX_MV = 2000  # absolute hard cap: mirrored in vcu_firmware/include/config.h and in core.py's gate

STATE_NAMES = {0: "SAFE", 1: "ARMED", 2: "FAULT"}
FAULT_BITS = {1: "LINK_TIMEOUT", 2: "ESTOP", 4: "DAC_FAIL", 8: "CMD_RANGE"}
NAK_TEXT = {
    1: "wrong state", 2: "E-stop active", 3: "DAC not ready", 4: "command link not fresh",
    5: "throttle not zero", 6: "fault cause still present", 7: "output wiring unverified",
}


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_frame(payload: str) -> bytes:
    raw = payload.encode("ascii")
    return b"$" + raw + b"*" + f"{crc16_ccitt(raw):04X}".encode() + b"\n"


def parse_frame(line) -> Optional[str]:
    """Return the payload of a valid frame, or None for anything malformed or with a bad CRC."""
    if isinstance(line, (bytes, bytearray)):
        try:
            line = bytes(line).decode("ascii")
        except UnicodeDecodeError:
            return None
    line = line.strip()
    if not line.startswith("$") or "*" not in line:
        return None
    body, _, crc = line[1:].rpartition("*")
    if len(crc) != 4 or not body:
        return None
    try:
        rx = int(crc, 16)
    except ValueError:
        return None
    return body if crc16_ccitt(body.encode("ascii")) == rx else None


def build_cmd(seq: int, throttle_mv: int, steer_cdeg: int, brake_pct: int) -> bytes:
    return build_frame(f"CMD,{seq & 0xFFFFFFFF},{int(throttle_mv)},{int(steer_cdeg)},{int(brake_pct)}")


def build_simple(kind: str, seq: int) -> bytes:
    assert kind in ("ARM", "DIS", "RST")
    return build_frame(f"{kind},{seq & 0xFFFFFFFF}")


def build_observation(seq: int, stamp_ms: int, cone_id: int, forward_mm: int,
                      gap_mm: int, decision: int) -> bytes:
    """Camera decision only: 0 STOP, 1 OBSERVE, 2 AT_TARGET. Never arms or commands throttle."""
    if not (0 <= seq <= 0x7FFFFFFF and 0 <= stamp_ms <= 0x7FFFFFFF and -1 <= cone_id <= 0x7FFFFFFF and
            -1 <= forward_mm <= 0x7FFFFFFF and -0x7FFFFFFF <= gap_mm <= 0x7FFFFFFF and decision in (0, 1, 2)):
        raise ValueError("invalid observation field")
    return build_frame(f"OBS,{seq},{stamp_ms},{cone_id},{forward_mm},{gap_mm},{decision}")


@dataclass
class Telemetry:
    fwseq: int
    t_ms: int
    state: int
    fault: int
    flags: int
    en: bool
    brk: bool
    dac_mv: int
    counts: int
    speed_mmps: int
    steer_raw: int
    brake_raw: int
    ack_seq: int
    link_age_ms: int

    @property
    def estop_input_fitted(self): return bool(self.flags & 32)
    @property
    def estop_ok(self): return self.estop_input_fitted and bool(self.flags & 1)
    @property
    def steer_pwm_ok(self): return bool(self.flags & 2)
    @property
    def brake_pwm_ok(self): return bool(self.flags & 4)
    @property
    def axle_motion_seen(self): return bool(self.flags & 8)
    @property
    def dac_ok(self): return bool(self.flags & 16)
    @property
    def state_name(self): return STATE_NAMES.get(self.state, "UNKNOWN")

    def fault_names(self) -> List[str]:
        return [n for b, n in FAULT_BITS.items() if self.fault & b]


def parse_message(payload: str):
    """Return ('TEL', Telemetry) | ('HELLO', (fw, proto)) | ('NAK', (seq, code)) | None."""
    f = payload.split(",")
    try:
        if f[0] == "TEL" and len(f) == 15:
            v = [int(x) for x in f[1:]]
            return "TEL", Telemetry(*v)
        if f[0] == "HELLO" and len(f) == 3:
            return "HELLO", (f[1], int(f[2]))
        if f[0] == "NAK" and len(f) == 3:
            return "NAK", (int(f[1]), int(f[2]))
        if f[0] == "OBSACK" and len(f) == 3:
            seq, decision = int(f[1]), int(f[2])
            if 0 <= seq <= 0xFFFFFFFF and decision in (0, 1, 2):
                return "OBSACK", (seq, decision)
    except ValueError:
        return None
    return None
