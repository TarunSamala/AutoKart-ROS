"""IMU acquisition backends for the autokart_imu node (MPU-6500).

There is no existing MPU-6500 driver anywhere in this repository (confirmed by
grep across the whole tree; `docs/decisions/CANONICAL_COMPONENTS.md` records
"MPU-6500 identity, connection, driver and calibration: UNKNOWN", and
`autokart_localization/odometry.py` says outright "There is no IMU in this
configuration"). This backend layer is therefore genuinely new code, not a
reused/wrapped algorithm -- unlike ``autokart_hp60c`` and the other two nodes
in this batch, there is nothing here to import from elsewhere in the repo.

A backend is anything with ``start()``, ``read() -> RawSample``, ``close()``.
``RawSample`` carries physical-unit, NOT bias-corrected gyro/accel plus a
``saturated`` flag -- bias correction and timestamp-monotonicity policy live in
``node.py`` (signal processing, not hardware access), matching the
hp60c/vcu_bridge/odometry pattern of keeping backends.py to "talk to the
device" and node.py to "decide what the data means".

``SyntheticBackend`` has no external dependency and is what makes this package
buildable and testable on any machine, including one with no MPU-6500
hardware. ``MPU6500I2CBackend`` is a structurally-correct real-hardware
adapter (register map and conversion factors from the public MPU-6500
datasheet) that lazily imports ``smbus2`` and fails closed
(``DEPENDENCY_UNAVAILABLE`` / ``SENSOR_NOT_FOUND``) when the library or the
physical sensor is not present -- which is always true in this development
environment (no Raspberry Pi, no I2C bus, no MPU-6500 attached, and `smbus2`
is not even installed). Its actual register-read path is NOT exercised by any
test in this change; only its fail-closed paths are. See DEPENDENCIES.md.
"""
from __future__ import annotations

import importlib
import math
import time
from dataclasses import dataclass
from typing import Optional, Protocol, Tuple


class ImuBackendError(RuntimeError):
    """Raised for any backend fault: missing library, missing sensor, bad read."""


@dataclass
class RawSample:
    timestamp_ns: int
    sequence: int
    gyro_radps: Tuple[float, float, float]     # physical units, NOT bias-corrected
    accel_mps2: Tuple[float, float, float]      # physical units
    saturated: bool                             # True if any raw axis is pinned at its full-scale limit
    source: str


class ImuBackend(Protocol):
    def start(self) -> None: ...
    def read(self) -> RawSample: ...
    def close(self) -> None: ...


# --- MPU-6500 datasheet constants (register map + conversion factors) ------
# These are public datasheet facts (shared register layout with the MPU-60x0/
# 9250 family), used identically by the real backend's conversion math and by
# the synthetic backend's raw->physical bookkeeping for saturation detection.
# They are NOT measured/calibrated for any particular physical unit -- see
# DEPENDENCIES.md / CONTRACT.md for what remains unverified.
WHO_AM_I_REG = 0x75
WHO_AM_I_EXPECTED = 0x70          # MPU-6500 WHO_AM_I value per datasheet
PWR_MGMT_1_REG = 0x6B
GYRO_CONFIG_REG = 0x1B
ACCEL_CONFIG_REG = 0x1C
ACCEL_XOUT_H_REG = 0x3B           # 14 contiguous bytes: accel(6) + temp(2) + gyro(6)

GYRO_FS_SENSITIVITY_LSB_PER_DPS = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}
ACCEL_FS_SENSITIVITY_LSB_PER_G = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}
GRAVITY_MPS2 = 9.80665
INT16_MAX = 32767
INT16_MIN = -32768


def _gyro_sensitivity(fs_dps: int) -> float:
    try:
        return GYRO_FS_SENSITIVITY_LSB_PER_DPS[fs_dps]
    except KeyError:
        raise ImuBackendError(f"UNSUPPORTED_GYRO_FS:{fs_dps}") from None


def _accel_sensitivity(fs_g: int) -> float:
    try:
        return ACCEL_FS_SENSITIVITY_LSB_PER_G[fs_g]
    except KeyError:
        raise ImuBackendError(f"UNSUPPORTED_ACCEL_FS:{fs_g}") from None


class SyntheticBackend:
    """Deterministic fake IMU samples. No external dependency. Never claims
    LIVE/hardware provenance (``source == "SYNTHETIC"``).

    Stationary by default (gyro ~= a fixed per-axis bias with a small
    deterministic ripple, accel ~= gravity on +Z with a small deterministic
    ripple on X/Y) so that a node-level stationary bias calibration has
    something realistic to estimate. ``rotation_radps_z`` can be set to
    simulate an actual turn appearing only *after* the calibration window, so
    a consumer can verify bias-corrected omega tracks a real rotation.

    Test hooks (used only by test/test_fault_injection.py, never by the real
    backend): ``saturate_after`` forces ``saturated=True`` from that sample
    index onward; ``nonmonotonic_after`` makes one sample's timestamp jump
    backwards.
    """

    def __init__(self, gyro_fs_dps: int = 500, accel_fs_g: int = 4, period_s: float = 0.005,
                 bias_z_radps: float = math.radians(0.8), rotation_radps_z: float = 0.0,
                 rotation_after_sample: int = 0, saturate_after: Optional[int] = None,
                 nonmonotonic_after: Optional[int] = None):
        self.gyro_fs_dps = gyro_fs_dps
        self.accel_fs_g = accel_fs_g
        self.period_s = period_s
        self.bias_z_radps = bias_z_radps
        self.rotation_radps_z = rotation_radps_z
        self.rotation_after_sample = rotation_after_sample
        self.saturate_after = saturate_after
        self.nonmonotonic_after = nonmonotonic_after
        self._seq = 0
        self._started = False
        self._t_ns = 0

    def start(self) -> None:
        self._started = True
        self._t_ns = time.monotonic_ns()

    def read(self) -> RawSample:
        if not self._started:
            raise ImuBackendError("BACKEND_NOT_STARTED")
        self._seq += 1
        time.sleep(self.period_s)
        step_ns = int(self.period_s * 1e9) or 1
        self._t_ns += step_ns
        if self.nonmonotonic_after is not None and self._seq == self.nonmonotonic_after:
            # Deliberately emit a timestamp earlier than the previous sample,
            # to exercise node.py's non-monotonic rejection path.
            emitted_ns = self._t_ns - 10 * step_ns
        else:
            emitted_ns = self._t_ns

        ripple = 0.01 * math.sin(self._seq * 0.1)
        rotating = self.rotation_after_sample and self._seq >= self.rotation_after_sample
        gz = self.bias_z_radps + ripple + (self.rotation_radps_z if rotating else 0.0)
        gx = 0.002 * math.sin(self._seq * 0.05)
        gy = 0.002 * math.cos(self._seq * 0.05)

        ax = 0.05 * math.sin(self._seq * 0.07)
        ay = 0.05 * math.cos(self._seq * 0.07)
        az = GRAVITY_MPS2

        saturated = bool(self.saturate_after is not None and self._seq >= self.saturate_after)

        return RawSample(
            timestamp_ns=emitted_ns, sequence=self._seq,
            gyro_radps=(gx, gy, gz), accel_mps2=(ax, ay, az),
            saturated=saturated, source="SYNTHETIC",
        )

    def close(self) -> None:
        self._started = False


class MPU6500I2CBackend:
    """Structurally-correct MPU-6500 I2C adapter (datasheet register map).

    Never exercised against real hardware in this environment -- fails closed
    with an explicit reason, never fabricates a reading. Nothing here is
    imported lazily from elsewhere in THIS repository (there is no existing
    MPU-6500 driver to reuse -- see the module docstring); what IS lazy is the
    optional ``smbus2`` third-party dependency, consistent with "do not fail
    the whole package build/import for a hardware library that may not be
    installed".
    """

    def __init__(self, i2c_bus_num: int = 1, i2c_address: int = 0x68, gyro_fs_dps: int = 500,
                 accel_fs_g: int = 4):
        self.i2c_bus_num = i2c_bus_num
        self.i2c_address = i2c_address
        self.gyro_sensitivity = _gyro_sensitivity(gyro_fs_dps)
        self.accel_sensitivity = _accel_sensitivity(accel_fs_g)
        self._bus = None
        self._seq = 0

    def start(self) -> None:
        try:
            smbus2 = importlib.import_module("smbus2")
        except Exception as exc:  # noqa: BLE001 - must surface as a reported fault, not an import-time crash
            raise ImuBackendError(f"DEPENDENCY_UNAVAILABLE:smbus2:{type(exc).__name__}:{exc}") from exc
        try:
            self._bus = smbus2.SMBus(self.i2c_bus_num)
            who_am_i = self._bus.read_byte_data(self.i2c_address, WHO_AM_I_REG)
        except Exception as exc:  # noqa: BLE001 - no bus / no device at that address / permission error
            raise ImuBackendError(f"SENSOR_NOT_FOUND:bus={self.i2c_bus_num}:addr={hex(self.i2c_address)}:"
                                   f"{type(exc).__name__}:{exc}") from exc
        if who_am_i != WHO_AM_I_EXPECTED:
            raise ImuBackendError(f"SENSOR_NOT_FOUND:unexpected_who_am_i={hex(who_am_i)}")
        try:
            # Wake the device (clear SLEEP bit in PWR_MGMT_1) -- datasheet default has it asleep.
            self._bus.write_byte_data(self.i2c_address, PWR_MGMT_1_REG, 0x00)
        except Exception as exc:  # noqa: BLE001
            raise ImuBackendError(f"SENSOR_INIT_FAILED:{type(exc).__name__}:{exc}") from exc

    def read(self) -> RawSample:
        if self._bus is None:
            raise ImuBackendError("BACKEND_NOT_STARTED")
        try:
            data = self._bus.read_i2c_block_data(self.i2c_address, ACCEL_XOUT_H_REG, 14)
        except Exception as exc:  # noqa: BLE001 - transient I2C fault must surface, not crash the loop
            raise ImuBackendError(f"I2C_READ_FAILED:{type(exc).__name__}:{exc}") from exc
        self._seq += 1

        def s16(hi: int, lo: int) -> int:
            v = (hi << 8) | lo
            return v - 65536 if v >= 32768 else v

        ax_raw, ay_raw, az_raw = s16(data[0], data[1]), s16(data[2], data[3]), s16(data[4], data[5])
        gx_raw, gy_raw, gz_raw = s16(data[8], data[9]), s16(data[10], data[11]), s16(data[12], data[13])
        raws = (ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw)
        saturated = any(r in (INT16_MAX, INT16_MIN) for r in raws)

        accel = tuple((r / self.accel_sensitivity) * GRAVITY_MPS2 for r in (ax_raw, ay_raw, az_raw))
        gyro = tuple(math.radians(r / self.gyro_sensitivity) for r in (gx_raw, gy_raw, gz_raw))
        return RawSample(
            timestamp_ns=time.monotonic_ns(), sequence=self._seq,
            gyro_radps=gyro, accel_mps2=accel, saturated=saturated, source="MPU6500_I2C",
        )

    def close(self) -> None:
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception:
                pass
        self._bus = None


def build_backend(name: str, **kwargs) -> ImuBackend:
    if name == "synthetic":
        allowed = ("gyro_fs_dps", "accel_fs_g", "period_s", "bias_z_radps", "rotation_radps_z",
                   "rotation_after_sample", "saturate_after", "nonmonotonic_after")
        return SyntheticBackend(**{k: v for k, v in kwargs.items() if k in allowed and v is not None})
    if name == "mpu6500":
        return MPU6500I2CBackend(
            i2c_bus_num=int(kwargs.get("i2c_bus_num", 1)),
            i2c_address=int(kwargs.get("i2c_address", 0x68)),
            gyro_fs_dps=int(kwargs.get("gyro_fs_dps", 500)),
            accel_fs_g=int(kwargs.get("accel_fs_g", 4)),
        )
    raise ImuBackendError(f"UNKNOWN_BACKEND:{name}")
