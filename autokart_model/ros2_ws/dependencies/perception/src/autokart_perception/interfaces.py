"""Optional hardware-neutral interfaces; importing these needs no camera SDK."""

from __future__ import annotations

from typing import Protocol, Sequence

import numpy as np

from .types import Detection, Frame


class DetectorProtocol(Protocol):
    version: str
    backend: str
    latency_ms: float
    verified_checkpoint: bool

    def detect(self, image_bgr8: np.ndarray) -> Sequence[Detection]: ...


class FrameSourceProtocol(Protocol):
    def read(self) -> Frame | None: ...
    def close(self) -> None: ...
