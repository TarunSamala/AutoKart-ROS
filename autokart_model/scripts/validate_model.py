#!/usr/bin/env python3
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parents[1]
model = root / "models/yolo26s/best.onnx"
manifest_path = root / "models/yolo26s/best.json"
manifest = json.loads(manifest_path.read_text())
digest = hashlib.sha256(model.read_bytes()).hexdigest()
assert digest == manifest["model_sha256"], (digest, manifest["model_sha256"])
assert manifest["classes"] == ["blue", "large_orange", "orange", "unknown", "yellow"]
assert manifest["vehicle_authorization"] == "DISABLED"

try:
    import numpy as np
    import onnx
    import onnxruntime as ort
except ImportError as exc:
    raise SystemExit(f"Required validation dependency missing: {exc}")

checked = onnx.load(str(model), load_external_data=True)
onnx.checker.check_model(checked)
session = ort.InferenceSession(str(model), providers=["CPUExecutionProvider"])
input_meta = session.get_inputs()[0]
shape = [1 if not isinstance(value, int) or value <= 0 else value for value in input_meta.shape]
sample = np.zeros(shape, dtype=np.float32)
outputs = session.run(None, {input_meta.name: sample})
assert outputs, "model returned no outputs"
print(json.dumps({
    "status": "PASS",
    "sha256": digest,
    "input_name": input_meta.name,
    "input_shape": shape,
    "output_shapes": [list(value.shape) for value in outputs],
    "scope": "x86/target-host CPU zero-input smoke only; no HP60C or vehicle qualification",
}, indent=2))
