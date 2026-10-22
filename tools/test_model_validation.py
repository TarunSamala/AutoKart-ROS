import hashlib
import json
from pathlib import Path

from model_validation import validate


def test_missing_model_fails(tmp_path):
    result = validate(tmp_path / "missing.onnx", tmp_path / "missing.json")
    assert result["status"] == "FAIL" and result["reason"] == "MODEL_FILE_MISSING"


def test_missing_manifest_fails(tmp_path):
    model = tmp_path / "model.onnx"
    model.write_bytes(b"fake")
    result = validate(model, tmp_path / "missing.json")
    assert result["status"] == "FAIL" and result["reason"] == "MANIFEST_FILE_MISSING"


def test_sha_mismatch_fails(tmp_path):
    model = tmp_path / "model.onnx"
    model.write_bytes(b"fake-weights")
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps({"model_sha256": "0" * 64, "classes": ["a"]}))
    result = validate(model, manifest)
    assert result["status"] == "FAIL" and result["sha256_matches_manifest"] is False


def test_matching_sha_and_classes_pass(tmp_path):
    model = tmp_path / "model.onnx"
    model.write_bytes(b"fake-weights")
    digest = hashlib.sha256(b"fake-weights").hexdigest()
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps({"model_sha256": digest, "classes": ["blue", "yellow"],
                                    "stage": "experimental", "vehicle_authorization": "DISABLED"}))
    result = validate(model, manifest)
    assert result["status"] == "PASS"
    assert result["class_count"] == 2
    assert result["vehicle_authorization"] == "DISABLED"


def test_real_repo_model_and_manifest_match():
    """Not a synthetic case: confirms the actual shipped yolo26s model/manifest pair is self-consistent."""
    root = Path(__file__).resolve().parents[1]
    result = validate(root / "models" / "yolo26s" / "best.onnx", root / "models" / "yolo26s" / "best.json")
    assert result["status"] == "PASS"
    assert result["sha256_matches_manifest"] is True
