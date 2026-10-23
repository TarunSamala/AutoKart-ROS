# YOLO26s FSOCO 2026-10-15

- Purpose: five-class Formula Student cone detection used by the phase-1 ROS launch path.
- Architecture: YOLO26s end-to-end ONNX, 640×640, `[1,300,6]` output.
- Training source: `MyDrive/AutoKart/YOLO26s_Only/yolo26s`.
- Dataset: FSOCO public archive; five AutoKart cone classes.
- Settings: 50 epochs, AdamW, batch 8, seed 42, Ultralytics 8.4.177.
- Test metrics: precision 0.7623816, recall 0.7226058, mAP50 0.7196122, mAP50-95 0.4838672. FSOCO test only.
- Formats: PyTorch and ONNX. No NCNN artifact found.
- Measured performance: existing x86 CPU report recorded 266.6 ms mean warm inference, about 3.75 FPS. This is not a Pi result.
- Raspberry Pi 4B: ONNX is available, but suitability is unqualified without an exact-device benchmark.
- Limitations: experimental; HP60C accuracy, calibration, Raspberry Pi timing, and vehicle readiness are unverified.
- Checkpoint SHA-256: `834abec3531b654d02d70b8879356cd44f572dacc12e8757f78bfcb3671d9281`.
- ONNX SHA-256: `425a89cef2eda9db9c84fbf30cad187e510510b95d7b7e9904003fd22cec1158`.

The active source path was not changed to point into this library.
