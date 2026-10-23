"""Direct CORE runtime and the separate, nonauthorizing LAB JSON adapter."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

from .sdk import PerceptionGeometrySDK
from .shared_core import BrakeCapability, PoseEstimate
from .types import Cone


PACKAGE_ROOT = Path(__file__).resolve().parents[2]


def _pose(value: dict | None) -> PoseEstimate | None:
    return PoseEstimate(**value) if value is not None else None


def _cones(values: list[dict]) -> list[Cone]:
    return [Cone(**item) for item in values]


class CoreRuntime:
    """CORE calls the algorithms directly; no HTTP objects or server imports."""

    def __init__(self, config_path: Path | None = None, wheelbase_m: float = 1.15):
        self.sdk = (PerceptionGeometrySDK(wheelbase_m=wheelbase_m) if config_path is None else
                    PerceptionGeometrySDK.from_yaml(config_path, wheelbase_m=wheelbase_m))
        self.config = self.sdk.config
        self.algorithms = self.sdk.algorithms

    def evaluate_observations(self, payload: dict) -> dict:
        now_ns = payload["timestamp_ns"]
        brake = payload.get("brake_capability")
        return self.sdk.process_cones(
            _cones(payload.get("cones", [])), _pose(payload.get("pose")),
            mission=payload["mission"], timestamp_ns=now_ns, source=payload.get("source", "SIMULATED"),
            obstacles=payload.get("obstacles", []),
            brake_capability=BrakeCapability(**brake) if brake is not None else None,
            stop_distance_m=payload.get("stop_distance_m"),
        ).to_dict()

    def evaluate_frame(self, frame, pipeline, pose: PoseEstimate | None = None,
                       mission: str = "STRAIGHT_TRACK") -> dict:
        """Run real/replayed RGB-D perception before the same autonomy algorithms."""
        return self.sdk.process_frame(frame, pose, mission=mission, pipeline=pipeline).to_dict()


class LabAdapter:
    """JSON boundary for LAB; delegates every algorithm decision to CoreRuntime."""

    def __init__(self):
        self.core = CoreRuntime()

    def evaluate(self, payload: dict) -> dict:
        if payload.get("source", "SIMULATED") not in {"SIMULATED", "REPLAY"}:
            raise ValueError("LAB accepts only simulated or recorded inputs")
        return self.core.evaluate_observations(payload)

    def reset(self) -> None:
        self.core = CoreRuntime()


def core_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Disarmed AutoKart CORE diagnostic runtime")
    parser.add_argument("--source", choices=("sample", "replay", "hp60c"), required=True)
    parser.add_argument("--input", type=Path, help="JSONL samples or replay directory")
    parser.add_argument("--mission", default="STRAIGHT_TRACK")
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--model", type=Path, help="verified ONNX cone model for HP60C mode")
    parser.add_argument("--bridge-library", type=Path, help="built HP60C bridge .so")
    parser.add_argument("--camera-configuration", type=Path, help="vendor HP60C configuration JSON")
    args = parser.parse_args(argv)
    runtime = CoreRuntime()
    if args.source == "sample":
        if args.input is None:
            parser.error("--input is required for sample")
        with args.input.open() as stream:
            for index, line in enumerate(stream, 1):
                if args.frames and index > args.frames:
                    break
                if line.strip():
                    print(json.dumps(runtime.evaluate_observations(json.loads(line)), allow_nan=False), flush=True)
        return 0

    from .pipeline import Pipeline
    from .recording import Replay
    from .camera import HP60C, CameraError
    from .detector import ConeDetector, MODEL_SHA256

    if args.source == "replay":
        if args.input is None:
            parser.error("--input is required for replay")
        source = Replay(args.input)
        detector = None
    else:
        source = None
        model = args.model or PACKAGE_ROOT / "models/coneslayer-simplified.onnx"
        try:
            detector = ConeDetector(model, runtime.config["confidence"], runtime.config["nms_iou"],
                                    expected_sha256=MODEL_SHA256, label="orange")
        except Exception as exc:
            print(json.dumps({"status": "CONFIG_ERROR", "detail": str(exc),
                              "vehicle_authorized": False}), flush=True)
            return 2
    pipeline = Pipeline(detector, runtime.config)
    count = 0
    try:
        while not args.frames or count < args.frames:
            try:
                if args.source == "hp60c" and source is None:
                    source = HP60C(library=args.bridge_library,
                                   configuration=args.camera_configuration,
                                   max_age_ms=runtime.config["max_frame_age_ms"])
                frame = source.read()
            except (CameraError, OSError) as exc:
                pipeline.note_sensor_fault(str(exc))
                print(json.dumps({"status": "SENSOR_DISCONNECTED", "detail": str(exc),
                                  "vehicle_authorized": False}), flush=True)
                if source is not None and hasattr(source, "close"):
                    source.close()
                source = None
                time.sleep(1.0)
                continue
            if frame is None:
                break
            # No pose provider is currently qualified on the Pi. Geometry still runs;
            # planning/control remain invalid until a fresh estimate is supplied.
            result = runtime.evaluate_frame(frame, pipeline, None, args.mission)
            print(json.dumps(result, allow_nan=False), flush=True)
            count += 1
    except KeyboardInterrupt:
        return 0
    finally:
        if source is not None and hasattr(source, "close"):
            source.close()
    return 0


if __name__ == "__main__":
    sys.exit(core_main())
