"""Detector loading. Delegates entirely to autokart_perception.live_mission.live_detector
so this experimental app always uses the same pinned model identity (SHA-256 checked)
as the production diagnostic entry point, instead of a second, possibly-drifting copy.
"""
from __future__ import annotations

import sys
from pathlib import Path

PERCEPTION_SRC = (Path(__file__).resolve().parents[2] /
                   "autonomy_stack_phase1/phase_01_perception/src")
if str(PERCEPTION_SRC) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_SRC))

from autokart_perception.live_mission import live_detector  # noqa: E402
from autokart_perception.detector import DetectorError  # noqa: E402

DEFAULT_KIND = "conenet-v1"


def load_detector(kind: str = DEFAULT_KIND, model_path=None, manifest_path=None):
    """Returns a ready detector instance, or raises DetectorError."""
    return live_detector(kind, model_path=model_path, manifest_path=manifest_path)


def probe(kind: str = DEFAULT_KIND) -> dict:
    result = {"test": "detector_load", "tested": True, "pass": False, "detail": None}
    try:
        detector = load_detector(kind)
        result["pass"] = True
        result["detail"] = {
            "kind": kind,
            "display_name": getattr(detector, "display_name", kind),
            "model_sha256": getattr(detector, "model_sha256", None),
            "backend": getattr(detector, "backend", None),
            "confidence": getattr(detector, "confidence", None),
        }
    except DetectorError as exc:
        result["detail"] = f"DetectorError: {exc}"
    except Exception as exc:  # pragma: no cover
        result["tested"] = False
        result["detail"] = f"{type(exc).__name__}: {exc}"
    return result


if __name__ == "__main__":
    import json
    print(json.dumps(probe(), indent=2))
