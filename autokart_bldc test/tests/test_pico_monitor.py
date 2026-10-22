"""Verify passive Pico identity checks without opening hardware."""
from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vcu import serial_interface as link  # noqa: E402
from autokart_vehicle_bridge.protocol import build_frame  # noqa: E402

TELEMETRY = "TEL,1,1,0,0,0,0,1,0,-1,-1,-1,-1,0,0"


class FakeSerial:
    def __init__(self, hello):
        self.data = build_frame(f"HELLO,{hello},2") + build_frame(TELEMETRY)
        self.writes = 0

    def read(self, size):
        chunk, self.data = self.data[:size], self.data[size:]
        if not chunk:
            time.sleep(0.005)
        return chunk

    def write(self, data):
        self.writes += 1
        raise AssertionError("passive monitor wrote to Pico")

    def close(self):
        pass


class PicoMonitorTests(unittest.TestCase):
    def check(self, hello, expected):
        fake = FakeSerial(hello)
        with patch.object(link.serial, "Serial", return_value=fake):
            result = link.probe("fake-port", wait_s=0.2)
        self.assertEqual(result["pass"], expected)
        self.assertEqual(result["detail"]["identity_verified"], expected)
        self.assertEqual(result["detail"]["telemetry_seen"], expected)
        self.assertEqual(fake.writes, 0)

    def test_expected_pico_telemetry_is_passive(self):
        self.check(link.EXPECTED_FIRMWARE, True)

    def test_other_firmware_cannot_be_accepted(self):
        self.check("0.2.2-no-estop-input-locked", False)


if __name__ == "__main__":
    unittest.main()
