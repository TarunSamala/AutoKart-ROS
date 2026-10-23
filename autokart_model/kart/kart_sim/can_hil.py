"""Safe simulated CAN/HIL protocol; this module never enables real actuators."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from pathlib import Path

from .types import SimulationError, VehicleCommand


def crc8_j1850(data: bytes) -> int:
    """CRC-8/SAE-J1850: polynomial 0x1D, init/xorout 0xFF."""
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ 0xFF


@dataclass(frozen=True)
class CanFrame:
    arbitration_id: int
    data: bytes
    timestamp_s: float


@dataclass(frozen=True)
class MotionCanMessage:
    sequence: int
    speed_cms: int
    steering_millirad: int
    brake_permille: int
    valid_ms: int

    def encode(self, timestamp_s: float = 0.0) -> CanFrame:
        if not (
            0 <= self.sequence <= 255
            and -32768 <= self.speed_cms <= 32767
            and -32768 <= self.steering_millirad <= 32767
            and 0 <= self.brake_permille <= 1000
            and 0 <= self.valid_ms <= 65535
        ):
            raise SimulationError("CAN field range invalid")
        body = (
            bytes([self.sequence])
            + int(self.speed_cms).to_bytes(2, "little", signed=True)
            + int(self.steering_millirad).to_bytes(2, "little", signed=True)
            + self.brake_permille.to_bytes(2, "little")
            + self.valid_ms.to_bytes(2, "little")
        )
        return CanFrame(0x180, body + bytes([crc8_j1850(body)]), timestamp_s)

    @classmethod
    def decode(cls, frame: CanFrame) -> "MotionCanMessage":
        if (
            frame.arbitration_id != 0x180
            or len(frame.data) != 10
            or crc8_j1850(frame.data[:-1]) != frame.data[-1]
        ):
            raise SimulationError("invalid CAN command CRC/layout")
        data = frame.data
        return cls(
            data[0],
            int.from_bytes(data[1:3], "little", signed=True),
            int.from_bytes(data[3:5], "little", signed=True),
            int.from_bytes(data[5:7], "little"),
            int.from_bytes(data[7:9], "little"),
        )


@dataclass
class CounterMonitor:
    last: int | None = None
    maximum_gap: int = 3

    def accept(self, value: int) -> None:
        if not 0 <= value <= 255:
            raise SimulationError("CAN sequence outside uint8 range")
        delta = 1 if self.last is None else (value - self.last) % 256
        if not 1 <= delta <= self.maximum_gap:
            raise SimulationError("CAN sequence duplicate/reverse/excessive gap")
        self.last = value


@dataclass
class CanTimeoutMonitor:
    timeout_s: float
    last_receive_s: float | None = None

    def observe(self, timestamp_s: float) -> None:
        if not isfinite(timestamp_s) or timestamp_s < 0:
            raise SimulationError("invalid CAN receive timestamp")
        if self.last_receive_s is not None and timestamp_s < self.last_receive_s:
            raise SimulationError("CAN time moved backward")
        self.last_receive_s = timestamp_s

    def stale(self, now_s: float) -> bool:
        return self.last_receive_s is None or now_s - self.last_receive_s > self.timeout_s + 1e-12


@dataclass(frozen=True)
class HilTimingRecord:
    receive_time_s: float
    decode_complete_time_s: float
    apply_time_s: float

    @property
    def decode_latency_s(self) -> float:
        return self.decode_complete_time_s - self.receive_time_s


class SimulatedCanPlantBridge:
    """Validated low-level command boundary for a simulated plant only."""

    def __init__(self, timeout_s: float = 0.1) -> None:
        self.counter = CounterMonitor()
        self.timeout = CanTimeoutMonitor(timeout_s)
        self.last_command = VehicleCommand()
        self.timing: list[HilTimingRecord] = []

    def receive(self, frame: CanFrame, decode_complete_time_s: float) -> VehicleCommand:
        message = MotionCanMessage.decode(frame)
        self.counter.accept(message.sequence)
        self.timeout.observe(frame.timestamp_s)
        if decode_complete_time_s < frame.timestamp_s:
            raise SimulationError("CAN decode completion precedes receive time")
        self.last_command = VehicleCommand(
            drive=max(0.0, min(1.0, message.speed_cms / 800.0)),
            brake=message.brake_permille / 1000.0,
            steering_rad=message.steering_millirad / 1000.0,
        ).validated()
        self.timing.append(HilTimingRecord(frame.timestamp_s, decode_complete_time_s, decode_complete_time_s))
        return self.last_command

    def command_at(self, now_s: float) -> VehicleCommand:
        return VehicleCommand(brake=0.7) if self.timeout.stale(now_s) else self.last_command


@dataclass(frozen=True)
class VirtualCanAvailability:
    available: bool
    interface: str
    detail: str


def detect_vcan(interface: str = "vcan0") -> VirtualCanAvailability:
    exists = (Path("/sys/class/net") / interface).exists()
    return VirtualCanAvailability(
        exists,
        interface,
        "virtual CAN interface detected" if exists else "virtual CAN interface not configured",
    )


def bus_load(frame_rate_hz: float, payload_bytes: int, bitrate: int) -> float:
    if not all(isfinite(value) and value > 0 for value in (frame_rate_hz, payload_bytes, bitrate)):
        raise SimulationError("invalid bus load input")
    return frame_rate_hz * (47 + payload_bytes * 10) / bitrate
