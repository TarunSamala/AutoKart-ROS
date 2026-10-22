"""Passive, read-only monitor for the experimental RP2040 Pico VCU link.

The production bridge is the sole VCU command writer. This module constructs
and transmits no ARM, CMD, OBS, RST, or DIS frames.
"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

BRIDGE_SRC = Path(__file__).resolve().parents[2] / "kart/ros2_ws/src/autokart_vehicle_bridge"
if str(BRIDGE_SRC) not in sys.path:
    sys.path.insert(0, str(BRIDGE_SRC))

from autokart_vehicle_bridge.protocol import PROTO_VERSION, parse_frame, parse_message  # noqa: E402

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None

EXPECTED_FIRMWARE = "0.1.0-rp2040-pico-bench-unwired"


class VCUInterface:
    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self._ser = None
        self._run = False
        self._thread: threading.Thread | None = None
        self.telemetry = None
        self.last_telemetry_ns: int | None = None
        self.last_hello: tuple[str, int] | None = None
        self.identity_verified = False
        self.connect_error: str | None = None

    @property
    def connected(self) -> bool:
        return self._ser is not None

    def connect(self) -> bool:
        if serial is None:
            self.connect_error = "pyserial not installed"
            return False
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.05, exclusive=True)
            self.connect_error = None
            return True
        except Exception as exc:  # depends on hardware and port ownership
            self._ser = None
            self.connect_error = f"{type(exc).__name__}: {exc}"
            return False

    def start(self) -> bool:
        if not self.connected and not self.connect():
            return False
        self._run = True
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self._run = False
        if self._thread is not None:
            self._thread.join(timeout=0.5)
            self._thread = None
        if self._ser is not None:
            self._ser.close()
            self._ser = None
        self.identity_verified = False

    def is_link_fresh(self, max_age_ms: int = 300) -> bool:
        return (self.connected and self.identity_verified and self.telemetry is not None
                and self.last_telemetry_ns is not None
                and (time.monotonic_ns() - self.last_telemetry_ns) / 1e6 <= max_age_ms
                and self.telemetry.link_age_ms <= max_age_ms)

    def _rx_loop(self):
        buf = b""
        while self._run:
            try:
                chunk = self._ser.read(256) if self._ser is not None else b""
            except Exception as exc:
                self.connect_error = f"read failed: {type(exc).__name__}: {exc}"
                self._run = False
                break
            buf = (buf + chunk)[-4096:]
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                payload = parse_frame(line)
                message = parse_message(payload) if payload is not None else None
                if message is None:
                    continue
                kind, value = message
                if kind == "HELLO":
                    self.last_hello = value
                    self.identity_verified = value == (EXPECTED_FIRMWARE, PROTO_VERSION)
                    if not self.identity_verified:
                        self.telemetry = None
                        self.connect_error = f"unexpected VCU identity: {value!r}"
                elif kind == "TEL" and self.identity_verified:
                    self.telemetry = value
                    self.last_telemetry_ns = time.monotonic_ns()


def probe(port: str, baud: int = 115200, wait_s: float = 6.0) -> dict:
    """Read only: wait for Pico HELLO and telemetry; never transmit a frame."""
    result = {"test": "pico_vcu_passive_connectivity", "tested": True, "pass": False, "detail": None}
    vcu = VCUInterface(port, baud)
    if not vcu.start():
        result["detail"] = vcu.connect_error
        return result
    try:
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline and not (vcu.identity_verified and vcu.telemetry is not None):
            time.sleep(0.05)
        result["pass"] = (vcu.identity_verified and vcu.telemetry is not None
                          and vcu.last_telemetry_ns is not None
                          and (time.monotonic_ns() - vcu.last_telemetry_ns) / 1e6 <= 500)
        result["detail"] = {"port": port, "hello": vcu.last_hello,
                            "identity_verified": vcu.identity_verified,
                            "telemetry_seen": vcu.telemetry is not None,
                            "command_link_fresh": vcu.is_link_fresh(),
                            "telemetry_link_age_ms": vcu.telemetry.link_age_ms if vcu.telemetry else None,
                            "error": vcu.connect_error}
    finally:
        vcu.stop()
    return result


if __name__ == "__main__":
    import json
    import sys as _sys
    print(json.dumps(probe(_sys.argv[1] if len(_sys.argv) > 1 else
                           "/dev/serial/by-id/usb-Arduino_RaspberryPi_Pico_A63064E62F6E204B-if00"), indent=2))
