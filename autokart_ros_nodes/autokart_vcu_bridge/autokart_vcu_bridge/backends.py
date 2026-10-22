"""VCU serial transport backends for the autokart_vcu_bridge node.

A backend is anything with ``start()``, ``send_command()``, ``send_request()``,
``poll() -> list of events``, ``close()``. It owns ONLY the wire transport
(USB-serial I/O, frame build/parse) -- deciding what to send (throttle-voltage
control law, the arm/fresh/safety gate) is node.py's job, using the reused
``autokart_vehicle_bridge.core`` module directly (see node.py), exactly the
same "backends.py = talk to the device, node.py = decide what the data means"
split used by ``autokart_hp60c`` and ``autokart_imu`` in this same library.

``SyntheticBackend`` has no external dependency: it simulates the VCU
firmware's own SAFE/ARMED/FAULT state machine and TEL/HELLO/NAK replies
directly, with no serial port and no import of anything in
``autokart_vehicle_bridge``. This is what makes the package buildable,
testable and runnable on any machine, including one with no ESP32-S3
attached (true in this development environment).

``SerialVcuBackend`` is a thin adapter around the existing, already-tested
``autokart_vehicle_bridge.protocol`` module (CRC16-CCITT framing, CMD/ARM/DIS/
RST encoding, TEL/HELLO/NAK decoding) -- imported lazily from the
``algorithm_package_path`` ROS parameter, never vendored or reimplemented, per
DEPENDENCIES.md. It additionally needs ``pyserial`` (present in this
environment) but has no physical ESP32-S3/VCU to talk to here, so its actual
read/write-over-USB path is NOT exercised by any test in this change -- only
its own fail-closed paths (missing dependency path, missing/unopenable serial
port) are. See reports/VALIDATOR_REPORT.md.
"""
from __future__ import annotations

import importlib
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple


class VcuBackendError(RuntimeError):
    """Raised for any backend fault: missing dependency, missing/unopenable
    serial port, a write/read failure."""


FAULT_BITS = {1: "LINK_TIMEOUT", 2: "ESTOP", 4: "DAC_FAIL", 8: "CMD_RANGE"}
STATE_NAMES = {0: "SAFE", 1: "ARMED", 2: "FAULT"}
THROTTLE_MAX_MV_LOCAL = 2000  # mirrors autokart_vehicle_bridge.protocol.THROTTLE_MAX_MV; duplicated here only
                              # as a data constant (the wire protocol's own documented hard cap), not as
                              # reimplemented framing/CRC logic, which stays exclusively in protocol.py.


@dataclass
class VcuTelemetry:
    """Local, dependency-free mirror of autokart_vehicle_bridge.protocol.Telemetry's
    DATA SHAPE (field names/derived properties) -- not a reimplementation of its
    CRC/framing logic, which stays exclusively in protocol.py for the real
    backend. Same reasoning as autokart_hp60c's local ``Frame`` dataclass
    mirroring autokart_perception's Frame shape without reimplementing SDK
    access."""
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


Event = Tuple[str, object]  # ("TEL", VcuTelemetry) | ("HELLO", (fw, proto)) | ("NAK", (seq, code))


class VcuBackend:
    def start(self) -> None: ...
    def send_command(self, seq: int, throttle_mv: int, brake_pct: int) -> None: ...
    def send_request(self, seq: int, kind: str) -> None: ...
    def poll(self) -> List[Event]: ...
    def close(self) -> None: ...


class SyntheticBackend:
    """Deterministic simulation of the VCU firmware's own SAFE/ARMED/FAULT
    state machine (see docs/VCU_PROTOCOL.md "VCU safety behaviour"), with no
    serial port and no reuse of autokart_vehicle_bridge. Never claims LIVE
    hardware provenance.

    Simplifications, documented rather than silently assumed exact: speed is
    directly proportional to commanded throttle (no ramp-rate modeling, no
    motor dynamics); RST always succeeds (synthetic has only one fault cause,
    CMD_RANGE, which this backend treats as already cleared once a new
    in-range command is observed); steer/brake raw readings are a fixed
    AS5600-centre value with a small deterministic ripple.
    """

    def __init__(self, hello_every_s: float = 5.0):
        self.hello_every_s = hello_every_s
        self.state = 0  # SAFE
        self.fault = 0
        self.counts = 0
        self._fwseq = 0
        self._ack_seq = 0
        self._last_hello_t = None
        self._started = False
        self._pending_nak: Optional[Tuple[int, int]] = None
        self._t = 0.0

    def start(self) -> None:
        self._started = True
        self._t = time.monotonic()
        self._last_hello_t = self._t

    def send_command(self, seq: int, throttle_mv: int, brake_pct: int) -> None:
        if not self._started:
            raise VcuBackendError("BACKEND_NOT_STARTED")
        self._ack_seq = seq & 0xFFFFFFFF
        if self.state == 1:  # ARMED
            if not (0 <= throttle_mv <= THROTTLE_MAX_MV_LOCAL):
                self.state = 2  # FAULT
                self.fault |= 8  # CMD_RANGE
                self._last_cmd = (0, 100)
                return
            self._last_cmd = (throttle_mv, brake_pct)
        else:
            self._last_cmd = (0, 100)

    def send_request(self, seq: int, kind: str) -> None:
        if not self._started:
            raise VcuBackendError("BACKEND_NOT_STARTED")
        self._pending_nak = None
        if kind == "ARM":
            if self.state == 0:
                self.state = 1
            else:
                self._pending_nak = (seq, 1)  # wrong state
        elif kind == "DIS":
            if self.state == 1:
                self.state = 0
            # FAULT is latched: DIS alone does not clear it (matches "latched until RST").
        elif kind == "RST":
            if self.state == 2:
                self.state = 0
                self.fault = 0
            else:
                self._pending_nak = (seq, 1)
        else:
            raise VcuBackendError(f"UNKNOWN_REQUEST_KIND:{kind}")

    def poll(self) -> List[Event]:
        if not self._started:
            raise VcuBackendError("BACKEND_NOT_STARTED")
        now = time.monotonic()
        events: List[Event] = []
        if self._pending_nak is not None:
            events.append(("NAK", self._pending_nak))
            self._pending_nak = None
        if now - self._last_hello_t >= self.hello_every_s or self._fwseq == 0:
            events.append(("HELLO", ("0.2.2-synthetic", 2)))
            self._last_hello_t = now

        throttle_mv, brake_pct = getattr(self, "_last_cmd", (0, 100))
        armed = self.state == 1
        speed_mmps = max(0, int(throttle_mv * 1.5)) if (armed and brake_pct < 50) else 0
        if armed and speed_mmps > 0:
            self.counts += max(1, speed_mmps // 50)
        self._fwseq += 1
        # dac_ok (bit4=16), steer_pwm_ok (bit1=2), brake_pwm_ok (bit2=4): the synthetic VCU always
        # reports its own outputs healthy, regardless of ARM state -- unconditional, not branched on
        # `armed` (a prior `16 if armed or True else 0` here was dead/misleading: the `or True` made
        # it always evaluate to 16 no matter what `armed` was, exactly the kind of "looks conditional,
        # never actually is" defect flagged in autokart_hp60c's own SENIOR_REVIEWER_REPORT.md -- fixed
        # here the same way, by writing the actually-unconditional value plainly).
        flags = 16 | 2 | 4
        steer_raw = 2048 + int(10 * ((self._fwseq % 20) - 10))
        brake_raw = 2048
        tel = VcuTelemetry(
            fwseq=self._fwseq, t_ms=int(now * 1000) & 0xFFFFFFFF, state=self.state, fault=self.fault,
            flags=flags, en=armed, brk=brake_pct >= 50, dac_mv=throttle_mv if armed else 0,
            counts=self.counts, speed_mmps=speed_mmps, steer_raw=steer_raw, brake_raw=brake_raw,
            ack_seq=self._ack_seq, link_age_ms=0,
        )
        events.append(("TEL", tel))
        return events

    def close(self) -> None:
        self._started = False


class SerialVcuBackend:
    """Adapter over the existing, reused ``autokart_vehicle_bridge.protocol``
    module, talking to a real USB-serial VCU. The algorithm-package path is
    resolved at construction time, not import time, and is never hardcoded --
    it comes from the ``algorithm_package_path`` ROS parameter (see node.py),
    same pattern as ``autokart_hp60c``'s ``HP60CBackend``.
    """

    def __init__(self, algorithm_package_path: str, port: str, baud: int = 115200):
        src = Path(algorithm_package_path).expanduser()
        if not src.is_dir():
            raise VcuBackendError(f"DEPENDENCY_PATH_NOT_FOUND:{src}")
        src_str = str(src)
        if src_str not in sys.path:
            sys.path.insert(0, src_str)
        try:
            self._protocol = importlib.import_module("autokart_vehicle_bridge.protocol")
        except Exception as exc:  # noqa: BLE001 - exercised via test_backends' fault case
            raise VcuBackendError(f"SDK_ADAPTER_IMPORT_FAILED:{type(exc).__name__}:{exc}") from exc
        try:
            import serial  # pyserial: a normal exec_depend, not lazy -- declared in package.xml
        except Exception as exc:  # noqa: BLE001
            raise VcuBackendError(f"DEPENDENCY_UNAVAILABLE:pyserial:{type(exc).__name__}:{exc}") from exc
        self._serial_module = serial
        self.port = port
        self.baud = baud
        self.ser = None
        self.rx_buf = b""

    def start(self) -> None:
        try:
            self.ser = self._serial_module.Serial(self.port, self.baud, timeout=0)
        except Exception as exc:  # noqa: BLE001 - no device at that path, permission error, etc.
            raise VcuBackendError(f"SERIAL_PORT_UNAVAILABLE:{self.port}:{type(exc).__name__}:{exc}") from exc

    def send_command(self, seq: int, throttle_mv: int, brake_pct: int) -> None:
        self._write(self._protocol.build_cmd(seq, throttle_mv, 0, brake_pct))

    def send_request(self, seq: int, kind: str) -> None:
        self._write(self._protocol.build_simple(kind, seq))

    def _write(self, data: bytes) -> None:
        if self.ser is None:
            raise VcuBackendError("BACKEND_NOT_STARTED")
        try:
            self.ser.write(data)
        except Exception as exc:  # noqa: BLE001
            raise VcuBackendError(f"SERIAL_WRITE_FAILED:{type(exc).__name__}:{exc}") from exc

    def poll(self) -> List[Event]:
        if self.ser is None:
            raise VcuBackendError("BACKEND_NOT_STARTED")
        try:
            chunk = self.ser.read(4096)
        except Exception as exc:  # noqa: BLE001
            raise VcuBackendError(f"SERIAL_READ_FAILED:{type(exc).__name__}:{exc}") from exc
        if chunk:
            self.rx_buf += chunk
            if len(self.rx_buf) > 8192:
                self.rx_buf = self.rx_buf[-8192:]
        events: List[Event] = []
        while b"\n" in self.rx_buf:
            line, self.rx_buf = self.rx_buf.split(b"\n", 1)
            payload = self._protocol.parse_frame(line)
            if payload is None:
                continue
            parsed = self._protocol.parse_message(payload)
            if parsed is None:
                continue
            kind, value = parsed
            if kind == "TEL":
                t = value
                events.append(("TEL", VcuTelemetry(
                    fwseq=t.fwseq, t_ms=t.t_ms, state=t.state, fault=t.fault, flags=t.flags, en=t.en,
                    brk=t.brk, dac_mv=t.dac_mv, counts=t.counts, speed_mmps=t.speed_mmps,
                    steer_raw=t.steer_raw, brake_raw=t.brake_raw, ack_seq=t.ack_seq,
                    link_age_ms=t.link_age_ms,
                )))
            elif kind in ("HELLO", "NAK"):
                events.append((kind, value))
        return events

    def close(self) -> None:
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None


def build_backend(name: str, **kwargs) -> VcuBackend:
    if name == "synthetic":
        return SyntheticBackend(**{k: v for k, v in kwargs.items() if k in ("hello_every_s",)})
    if name == "serial":
        return SerialVcuBackend(
            algorithm_package_path=kwargs["algorithm_package_path"],
            port=kwargs.get("port", "/dev/ttyACM0"),
            baud=kwargs.get("baud", 115200),
        )
    raise VcuBackendError(f"UNKNOWN_BACKEND:{name}")
