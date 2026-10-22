"""Thin wrapper around the existing, already-built autokart_perception HP60C bridge.

Reuses autokart_perception.camera.HP60C as-is (no reimplementation of the SDK
adapter). Adds warmup-retry handling, because the SDK's first one or two
frames after start() routinely come back CALIBRATION_UNAVAILABLE / FRAME_TIMEOUT
while intrinsics settle -- that is expected, not a fault.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

PERCEPTION_SRC = (Path(__file__).resolve().parents[2] /
                   "autonomy_stack_phase1/phase_01_perception/src")
if str(PERCEPTION_SRC) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_SRC))

from autokart_perception.camera import HP60C, CameraError  # noqa: E402


class CameraBridge:
    """Owns one HP60C handle. Not thread-safe; call read() from one thread only."""

    def __init__(self, bridge_library=None, camera_configuration=None,
                 max_frame_age_ms: int = 250, warmup_attempts: int = 20):
        self._cam = HP60C(library=bridge_library, max_age_ms=max_frame_age_ms,
                           configuration=camera_configuration)
        self.warmup_attempts = warmup_attempts
        self.started = False
        self.last_error: str | None = None

    def start(self):
        self._cam.start()
        self.started = True

    def stop(self):
        if self.started:
            self._cam.close()
            self.started = False

    def read(self, timeout_ms: int = 1000):
        """Return a Frame, or raise CameraError on the final attempt's failure."""
        if not self.started:
            self.start()
        last_exc = None
        for _ in range(max(1, self.warmup_attempts)):
            try:
                frame = self._cam.read(timeout_ms=timeout_ms)
                self.last_error = None
                return frame
            except CameraError as exc:
                last_exc = exc
                self.last_error = str(exc)
                time.sleep(0.1)
        raise last_exc

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()


def probe(bridge_library=None, camera_configuration=None) -> dict:
    """One-shot connectivity check for TEST_REPORT.md. Never raises."""
    bridge = CameraBridge(bridge_library=bridge_library, camera_configuration=camera_configuration)
    result = {"test": "camera_detection", "tested": True, "pass": False, "detail": None}
    try:
        frame = bridge.read(timeout_ms=1000)
        result["pass"] = True
        result["detail"] = {
            "rgb_shape": list(frame.rgb.shape),
            "depth_shape": list(frame.depth.shape),
            "intrinsics": {"fx": frame.intrinsics.fx, "fy": frame.intrinsics.fy,
                           "cx": frame.intrinsics.cx, "cy": frame.intrinsics.cy},
            "source": frame.source,
        }
    except CameraError as exc:
        result["detail"] = f"CameraError: {exc}"
    except Exception as exc:  # pragma: no cover - defensive, reported not swallowed
        result["tested"] = False
        result["detail"] = f"{type(exc).__name__}: {exc}"
    finally:
        bridge.stop()
    return result


if __name__ == "__main__":
    import json
    print(json.dumps(probe(), indent=2))
