#!/usr/bin/env bash
set -euo pipefail

bundle_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$bundle_root"

if [[ ! -f MANIFEST.sha256 ]]; then
    echo "MANIFEST.sha256 is missing" >&2
    exit 1
fi

sha256sum --check MANIFEST.sha256

expected_model=$(python3 - <<'PY'
import json
print(json.load(open('models/yolo26s/best.json'))['model_sha256'])
PY
)
actual_model=$(sha256sum models/yolo26s/best.onnx | awk '{print $1}')
if [[ "$expected_model" != "$actual_model" ]]; then
    echo "Model manifest/hash mismatch" >&2
    exit 1
fi

echo "Bundle integrity: PASS"
echo "Model identity: PASS ($actual_model)"
echo "Safety status: NO ACTUATOR AUTHORITY; VEHICLE MOTION NO-GO"

