"""Laptop side of the AutoKart serial protocol (see docs/PROTOCOL.md). No ROS dependencies."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

PROTO_VERSION = 1
THROTTLE_MAX_MV = 2000  # hard cap, mirrored in firmware and in the command gate

STATE_NAMES = {0: "SAFE", 1: "ARMED", 2: "FAULT"}
FAULT_BITS = {1: "LINK_TIMEOUT", 2: "ESTOP", 4: "DAC_FAIL", 8: "CMD_RANGE"}
NAK_TEXT = {
    1: "wrong state", 2: "E-stop active", 3: "DAC not ready", 4: "command link not fresh",
    5: "throttle not zero", 6: "fault cause still present",
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
    def estop_ok(self): return bool(self.flags & 1)
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
    except ValueError:
        return None
    return None
