#!/usr/bin/env python3
"""Read-only: confirm the configured live detector model file matches its manifest SHA-256, and report
the manifest's stage/vehicle_authorization/class list. Never edits the model, manifest, or any config.
Does not and cannot "approve" a model for vehicle use -- that is the manifest's own
vehicle_authorization field and the operator's calibration_verified decision, neither of which this
tool can set."""
from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT / "models" / "yolo26s" / "best.onnx"
DEFAULT_MANIFEST = ROOT / "models" / "yolo26s" / "best.json"


def validate(model_path: Path, manifest_path: Path) -> dict:
    result = {"model_path": str(model_path), "manifest_path": str(manifest_path),
              "checked_utc": datetime.now(timezone.utc).isoformat()}
    if not model_path.is_file():
        return {**result, "status": "FAIL", "reason": "MODEL_FILE_MISSING"}
    if not manifest_path.is_file():
        return {**result, "status": "FAIL", "reason": "MANIFEST_FILE_MISSING"}
    try:
        manifest = json.loads(manifest_path.read_text())
    except (ValueError, OSError) as exc:
        return {**result, "status": "FAIL", "reason": f"MANIFEST_UNREADABLE:{exc}"}
    actual_sha256 = hashlib.sha256(model_path.read_bytes()).hexdigest()
    manifest_sha256 = manifest.get("model_sha256")
    sha_match = actual_sha256 == manifest_sha256
    classes = manifest.get("classes", [])
    stage = manifest.get("stage", "UNKNOWN")
    authorization = manifest.get("vehicle_authorization", "UNKNOWN")
    status = "PASS" if sha_match and classes else "FAIL"
    return {**result, "status": status, "sha256_matches_manifest": sha_match,
            "model_sha256": actual_sha256, "manifest_sha256": manifest_sha256,
            "classes": classes, "class_count": len(classes), "stage": stage,
            "vehicle_authorization": authorization,
            "note": "This tool confirms file integrity and reports manifest fields only. It is not a "
                    "model-accuracy validation and does not set calibration_verified."}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default=str(DEFAULT_MODEL))
    parser.add_argument("--manifest", default=str(DEFAULT_MANIFEST))
    parser.add_argument("--out-dir", default=str(ROOT / "results"))
    args = parser.parse_args()
    result = validate(Path(args.model), Path(args.manifest))
    print(json.dumps(result, indent=2))
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"model_validation_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.json"
    path.write_text(json.dumps(result, indent=2, sort_keys=True))
    print(f"Saved {path}")
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
