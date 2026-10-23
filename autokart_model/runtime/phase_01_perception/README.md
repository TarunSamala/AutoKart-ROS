# Phase 1 — Perception and track geometry

## Objective and scope

This is a stationary HP60C RGB-D perception prototype. It acquires paired frames, runs a real pretrained **orange-cone-only** detector, associates registered depth, estimates local cone ground positions, computes Delaunay and Gabriel graphs, proposes track gates, and draws a provisional centerline only when boundary evidence is sufficient. It records raw frames and observations for replay. There is no actuator control or planning. Short, timestamp-aware smoothing affects the stationary display only; predicted cones never enter the metric graph.

## Hardware and SDK

- YDLIDAR HP60C, powered USB, Linux x86_64 or Raspberry Pi ARM64; existing `../check_camera.sh` and `../test_camera.sh` remain intact.
- Vendor Linux SDK pinned to Drone-SWAN revision `482a43868acfdf2b6d64dc1a9c3560af50beb44c`, package SHA-256 `24a71680e96411d3cc9217654b93d492f5afbfe212b8c01e987371d26792f404` in `../setup.sh`.
- `native/hp60c_bridge.cpp` wraps the C++ SDK with a small C ABI. The SDK USB listener opens the HP60C configuration; a callback deep-copies a complete paired BGR8/RAW16 frame into one overwriteable mailbox. It exposes SDK frame IDs, timestamps, RGB intrinsics, calibration parameters, and registration state. The `mailbox_overwrites` counter counts complete frame pairs replaced before processing; it is not a USB packet-loss counter. The SDK adapter also reports complete pairs received and elapsed capture time. Python `ctypes` consumes it without polling a vendor GUI.
- The SDK's RAW16 depth unit is millimeters, so `depth_scale=0.001` m/count. RGB is BGR8. Vendor timestamps are milliseconds; arrival age is checked with the host monotonic clock. The adapter accepts only 640×480 paired frames. A 20-frame live session on this x86_64 host measured a median 12.35 RGB/depth FPS after startup; 20 FPS was not observed.
- The SDK registration flag must be true before depth association. Unregistered frames produce `DEPTH_ALIGNMENT_UNAVAILABLE`; the SDK's extrinsic direction and physical alignment have not been bench validated, so unregistered reprojection is deliberately unavailable.

## Software architecture and algorithms

### Runtime validity contract

Every result now carries a source (`LIVE`, `REPLAY`, or `SYNTHETIC`) and a separate perception assessment: `VALID`, `DEGRADED`, `AMBIGUOUS`, `STALE`, or `SENSOR_FAULT`. It includes reason codes, frame and production monotonic timestamps, confidence, and a **0 m/s hold advisory**. `vehicle_authorized` is always false; Phase 1 has no vehicle command channel. `VALID` describes only a possible perception output after all evidence checks, not permission to drive. The current bench mount is `MOUNT_CONFIGURED_UNVERIFIED`, so current operation cannot produce a verified live path. Synthetic and replay output always carry `NON_LIVE_SOURCE` and cannot become a live-valid output.

Live capture uses the SDK's single overwriteable latest-frame mailbox. The pipeline rejects non-increasing frame timestamps or sequences, 150 ms-old frames, inference/processing overload, invalid calibration or alignment, unusable depth, and model failure. It independently checks gate width and side order, cone reuse, crossings, forward path progression, and self-intersection after graph selection. A `STALE`, `AMBIGUOUS`, or `SENSOR_FAULT` assessment clears the path from the result. A sensor fault latches until the Phase 1 process restarts; receiving another image does not re-arm it. Display smoothing remains separate from graph input. These are software checks and do not establish physical safety.

Detections, metric cones, gates, and local path samples carry the source frame's monotonic observation timestamp; diagnostics also record processing-stage monotonic timestamps. New recordings store raw-array SHA-256 digests, model and mount hashes, safety assessments, rejected gate details, and an `events.jsonl` file of fault-state transitions. Replay requires contiguous six-digit frame IDs, verifies array integrity, and compares saved mount provenance with the active configuration. Legacy recordings without a mount hash replay for inspection with `CALIBRATION_PROVENANCE_MISSING`; they cannot provide verified calibration evidence.

`camera.py` acquires frames; `detector.py` loads a pinned YOLOv7-tiny ONNX model via OpenCV DNN and applies letterboxing, confidence filtering, and NMS; `depth.py` finds a coherent near-depth cluster in the lower central detection region and validates range, support, timestamps, and registration. It uses RGB intrinsics to back-project depth. A **measured** 4×4 optical-camera-to-kart transform (kart X forward, Y left, Z up) maps the sample, and the cone-bottom image ray intersects the ground plane. Estimates are rejected when the sample and ground projection disagree beyond the configured limit. Reported uncertainty includes sample spread, cone surface radius, and projection difference. This remains a local estimate until mounting and ground assumptions are measured on hardware.

`geometry.py` triangulates unique finite metric points with SciPy Delaunay, returning explicit statuses for insufficient, collinear, or degenerate inputs. `gabriel.py` keeps edges for which every other point satisfies `(P-A)·(P-B) >= -1e-9 m²`. `boundaries.py` forms a left/right hypothesis from blue/yellow labels when available, or from lateral clusters straddling the camera-forward axis. The latter is provisional and can be ambiguous on a curved or offset track. `gates.py` evaluates plausible opposite-side pairs, using graph connectivity as a score rather than a mandatory edge, then selects an ordered, non-reusing sequence with width and crossing checks. `centerline.py` rejects ambiguous gates. `curvature.py` estimates signed curvature at observed interior points only. No track extrapolation is done.

`visualization.py` shows the BGR feed with detections and top-down geometry, status, source label, frame rates and latencies, CPU, memory, and mailbox overwrites. The top-down grid starts at 0–4 m forward and expands to include localized cones. OpenCV draws the panels; Tk displays them when the installed OpenCV build lacks HighGUI. Keys: `b` boxes, `l` labels, `d` Delaunay, `g` Gabriel, `t` gates, `c` centerline, `s` boundaries, `k` curvature, `q` quit. `webui.py` serves the same two panels and diagnostics in a browser with overlay switches. The source banner distinguishes LIVE, REPLAY, and SYNTHETIC. Display-only predictions are drawn hollow. `recording.py` writes per-frame compressed RGB/RAW16 arrays plus JSON timestamps, calibration, scale, model version, detections, localized cones, and diagnostics. Existing recording directories are refused. Replay consumes recorded observations by default for deterministic geometry without camera or model inference.

The dashboard also shows the active checkpoint and thresholds, calibration state, per-cone depth and full rejection reason, Delaunay/Gabriel edge counts, inference/depth/geometry/render timing, and bounded RGB/processed FPS history. A labeled replay reports visible-cone recall at IoU 0.50; live frames without ground truth show no recall value. Sampled surface depth is labeled separately from calibrated ground-contact distance. The **Calibration & Cone Diagnostics** panel has inputs for lens height, downward pitch, roll, and yaw. `Apply for this session` validates the values and updates the transform on subsequent frames; it clears the old image while waiting for a new frame. The page displays the current X/Y origin, SDK registration report, graph-eligible cone count, valid/coherent depth-pixel counts, depth median/spread, ground XY, and full localization status per detection. Surveyed distance and XY error appear when reviewed replay annotations supply references. A session edit never changes `config/perception.yaml` or marks the calibration verified. The LAN viewer has no authentication; use the calibration controls only on a trusted network and perform surveyed-cone validation before trusting geometry.

## Model and classification limits

Run `./models/download_model.sh` to fetch the pinned [Coneslayer](https://github.com/mkrupczak3/Coneslayer) ONNX checkpoint. It is GPL-3.0, 416×416 YOLOv7-tiny, and supports **one orange traffic-cone class only**. See [model details](models/README.md). Blue, yellow, large-orange, and unknown are not inferred by this checkpoint. The cited TraCon and BRT repositories are datasets, not checkpoints. No HP60C detection precision, recall, mAP, confusion matrix, or 99% reliability has been measured.

The HP60C SDK frame is BGR8 and is used without rotation. The detector resizes it with a 114-valued letterbox to 416×416, swaps BGR to RGB, scales to `[0,1]`, decodes the ONNX `1×10647×6` output as center box/objectness/class score, rescales boxes, then applies NMS. It uses OpenCV DNN's default CPU path. The checkpoint's documented settings are confidence 0.54 and NMS IoU 0.50, matching `config/perception.yaml`. A 640×640 input trial failed the ONNX reshape in this OpenCV build; this particular checkpoint supports 416×416 here. A unit test checks channel conversion, padding, and box scaling. Missed-cone causes in the user-provided screenshot remain **undetermined** because the corresponding raw RGB/depth frame and exhaustive manual labels are unavailable. Do not lower the live confidence threshold based on that screenshot alone.

An optional YOLO11n path now accepts a trained raw-output ONNX model with a checksum-bound `model.json` class manifest. Its decoder handles the YOLO11 class-score output and class-specific NMS for blue, yellow, orange, large orange, and unknown labels actually present in the manifest. It is **not the current live model**: no 19,385-image eligible dataset has been audited and no new checkpoint has been trained. The default dashboard still uses the pinned orange-only Coneslayer checkpoint. See [model details](models/README.md).

### Qualified and experimental YOLO11n training

The trainer expects a standard YOLO **detection** dataset with `images/train`, `images/val`, `images/test` and matching `labels/...` directories. Each label is `class x_center y_center width height` in normalized coordinates. `data.yaml` must define `path`, `train`, `val`, `test`, cone class names, nonempty `source` and `license` provenance strings, and `session_manifest: sessions.csv`. The CSV has `image,session_id` columns; every image must be listed with its path relative to `path` (for example `images/train/001.jpg`). All frames from a recording session must remain within one split. For FSOCO, group related source sequences or scenes under one session ID where provenance supports it; unknown source grouping requires manual review. Exact duplicate detection cannot rule out near-duplicate video frames. Do not count augmented variants as independent photos. The source/license strings record claims, not legal clearance.

```bash
cd phase_01_perception
./install_training.sh
./models/download_yolo11n.sh
. ../.venv-training/bin/activate
python -m autokart_perception.training audit --stage experimental --data /path/to/cone_dataset/data.yaml
python -m autokart_perception.training train --stage experimental --data /path/to/cone_dataset/data.yaml \
  --run-dir training_runs/yolo11n-experimental-001 --epochs 100 --imgsz 640 --batch 4 --device cpu \
  --weights models/yolo11n.pt
python -m autokart_perception.training audit --stage qualified --data /path/to/cone_dataset/data.yaml
python -m autokart_perception.training train --data /path/to/cone_dataset/data.yaml \
  --run-dir training_runs/yolo11n-cones-001 --epochs 100 --imgsz 640 --batch 4 --device cpu \
  --weights models/yolo11n.pt
```

The **qualified** gate remains **19,385 unique cone-positive training images**. `--stage experimental` permits a smaller audited set with separate positive validation/test splits and a manifest explicitly marked `EXPERIMENTAL_HP60C_VALIDATION_PENDING`; it does not lower the final requirement. Both stages reject malformed labels, duplicate images, and sessions shared between splits. Training starts from the official pretrained `yolo11n.pt` detection weights, validates the held-out test split, then exports ONNX and compares raw PyTorch/ONNX predictions on a test image. Training refuses an existing run directory. A successful run writes `dataset_audit.json`, `fit/weights/best.pt`, `model.onnx`, and a SHA-256-bound `model.json`. Select it explicitly with `--model`; the pinned Coneslayer checkpoint remains the default. On a GPU host, install `.[train]` after checking sufficient disk and NVIDIA/PyTorch compatibility. [Ultralytics YOLO11](https://docs.ultralytics.com/models/yolo11), [export](https://docs.ultralytics.com/modes/export), and [AGPL-3.0 terms](https://docs.ultralytics.com/help/contributing) apply.

This host has a GTX 1650 listed by PCI, but `nvidia-smi` cannot reach its driver. The separate `.venv-training` has CPU-only PyTorch 2.7.1, torchvision 0.22.1, Ultralytics 8.3.253 and ONNX 1.17.0; `torch.cuda.is_available()` is false. Official COCO YOLO11n weights were downloaded and checksum-verified, but **the 80 COCO classes do not include cones**: these are fine-tuning initialization only. About 6.9 GB remained after installation on 2026-10-09. [FSOCO](https://fsoco.github.io/fsoco-dataset/overview/) has 11,572 box images and its [archive is 24 GB](https://fsoco.github.io/fsoco-dataset/download), so it was **not downloaded**. It is the preferred experimental source once storage and rights are resolved. [BRT](https://github.com/Bauman-Racing-Team/BRT-Cone-Pose-Dataset) explicitly derives its images from FSOCO; do not add them as independent images. For local copies, `python -m autokart_perception.dataset_sources --fsoco /path/to/fsoco --brt /path/to/brt --output overlap.json` records exact/pixel matches and excludes all BRT images from the independent count. Resized/cropped overlap still needs source-ID review. No YOLO11n training or export has run here.

## Installation

From the repository root on Linux:

```bash
./check_camera.sh                 # optional hardware enumeration
./setup.sh                        # pinned vendor SDK; needed for live acquisition
./phase_01_perception/build_native.sh
cd phase_01_perception
python3 -m venv --system-site-packages .venv
. .venv/bin/activate
python -m pip install -e .
./models/download_model.sh
```

The existing viewer remains `../test_camera.sh`. On Raspberry Pi OS, install the compiler/OpenCV/USB packages listed in the root README first. OpenCV DNN ONNX support must be available. The native build selects `x86_64-linux-gnu` or `aarch64-linux-gnu` using `../platform.sh`; ARM64 execution and throughput need hardware verification. `.venv` and build outputs should remain local.

On Debian/Ubuntu, install `python3-venv` if `python3 -m venv` reports that `ensurepip` is unavailable. Install `python3-tk` for the graphical fallback if OpenCV lacks HighGUI. An editable user install (`python -m pip install --user -e .`) was also verified on the development host, which already had the Python dependencies.

## Calibration

For metric live operation, measure the optical-camera-to-bench transform and put its 4×4 homogeneous matrix in `config/perception.yaml` as `camera_to_kart`, or fill **every** field in `camera_mount`: `height_reference: optical_center`, `height_m`, `x_forward_m`, `y_left_m`, `pitch_down_deg`, `roll_deg`, `yaw_left_deg`. The translation is the optical center in the chosen ground frame. Optical axes are x right, y down, z forward; bench axes are X forward, Y left, Z up. Positive yaw turns forward toward left; positive pitch points forward down; roll follows the right-hand rule about X. The generated transform is rigid and rejected if incomplete, nonfinite, or below ground. The operator confirmed **180 mm vertically to lens center**, level and forward-facing with no roll/yaw, and chose the floor point below the lens as the stationary-bench origin. Those values are now in `camera_mount`; the status remains `MOUNT_CONFIGURED_UNVERIFIED` until surveyed-cone validation. This bench origin is not a surveyed kart-body origin. Confirm the SDK reports registration on the actual camera and validate physical RGB/depth alignment with a target at several distances. The SDK reports depth scale 0.001 m/count; measure depth accuracy before using gate distances. The default valid depth range is 0.3–4.0 m, capped at the [manufacturer's 4 m range](https://www.ydlidar.com/products/view/29.html). The SDK supplies RGB/IR intrinsics and R/T, but its R/T direction is not used here without verification.

To validate a measured mount, annotate several cones with their bounding boxes and independently surveyed ground coordinates `xy_m: [forward_m, left_m]`. Choose the tolerance before running. Then run:

```bash
python -m autokart_perception.calibration --recording recordings/cone-session \
  --annotations recordings/cone-session-annotations.json --tolerance-m 0.10 \
  --output recordings/cone-session-calibration-report.json
```

The report gives each rejection reason and median, p95, and maximum ground-position error. It returns `NOT_EXECUTED` with no configured mount or surveyed cones; `PASS` requires every surveyed cone to localize within the chosen tolerance. This validates the local bench setup only. Recheck after any camera movement.

## Execution

Run from `phase_01_perception/` after activating the environment:

```bash
python -m autokart_perception --source synthetic
python -m autokart_perception --source hp60c
python -m autokart_perception --source hp60c --record recordings/bench-001
python -m autokart_perception --source replay --path recordings/bench-001
python -m autokart_perception --source replay --path recordings/bench-001 --annotations recordings/bench-001-annotations.json
python -m autokart_perception --source hp60c --web --host 0.0.0.0 --port 8765
```

Open `http://localhost:8765` on the host or `http://<host-LAN-IP>:8765` from another device on the same LAN. The browser view has live RGB and top-down panels, eight overlay switches, rates, depth validity, cone counts, frame age, processing time, received/processed frame counts, and mailbox overwrite percentage. `--web` keeps running and retries a disconnected camera until Ctrl+C; the page reports `CAMERA_DISCONNECTED` while the HP60C is absent. Binding to `0.0.0.0` exposes the viewer on the LAN without login, so use only on a trusted network.

Enter the measured **optical-center** height in metres and angles in degrees in the calibration panel. A positive pitch tilts down; positive yaw turns toward kart left. The four fields accept height 0.05–2.0 m, pitch −60–75°, roll −45–45°, and yaw −180–180°. The existing bench X/Y translation remains fixed. The interface shows `MOUNT_CONFIGURED_UNVERIFIED` after an edit; it does not claim surveyed accuracy. To make a measured setting persist across restarts, enter it in `config/perception.yaml` and run the survey validation above.

For a machine without a display, add `--headless --frames 10`; for an absent camera, `--connect-timeout 10` controls diagnostic timeout. `--model` selects a compatible ONNX file. Synthetic mode uses labeled geometry observations, not AI detections, and is displayed as SYNTHETIC. Replay uses saved observations and is displayed as REPLAY. The default model is ignored for replay because saved observations are authoritative.

## Manual annotation and detector evaluation

Capture a **new** cone scene after arranging the cones, preserving the original recording:

Stop the live dashboard process before this separate camera capture, then restart it afterward. Use a new recording directory every time.

```bash
python -m autokart_perception --source hp60c --headless --frames 100 --record recordings/cone-session
python -m autokart_perception.evaluation --recording recordings/cone-session \
  --make-template recordings/cone-session-annotations.json
python -m http.server 8768 --bind 127.0.0.1 --directory recordings/cone-session-annotations
```

Open `http://127.0.0.1:8768/index.html`. Drag a box around **every** visible cone, choose its observed class, enter measured distance or surveyed X/Y when known, and add `small`, `distant`, `occluded`, or `low_light` difficulty tags. Mark verified empty frames and download the reviewed JSON. Copy it to a named path. `null` frames are excluded; `[]` means reviewed and cone-free. Record independent HP60C sessions that include near, distant, small, partly occluded and cone-free views; never invent a positive label from a screenshot. Keep evaluation sessions out of training and validation, and list each image's session in the training CSV.

```bash
python -m autokart_perception.evaluation --recording recordings/cone-session \
  --annotations recordings/cone-session-annotations-reviewed.json \
  --confidence 0.25 0.40 0.54 0.70 --nms-iou 0.45 0.50 0.60 \
  --output recordings/cone-session-evaluation.json
```

For the A/B test after training, run both models on the **same** manually labeled HP60C sessions, using one confidence/NMS setting and the candidate's training audit:

```bash
python -m autokart_perception.evaluation --recording recordings/heldout-hp60c-session \
  --annotations recordings/heldout-reviewed.json \
  --compare-model training_runs/yolo11n-experimental-001/model.onnx \
  --training-audit training_runs/yolo11n-experimental-001/dataset_audit.json \
  --confidence 0.54 --nms-iou 0.50 --output recordings/heldout-ab.json
```

The evaluator reruns both models on the original RGB arrays and rejects training/validation session overlap. It reports TP/FP/FN, precision, recall, class-averaged mAP50/mAP50–95 over classes present in the ground truth, distance-bin recall, difficulty coverage, and median/p95 inference latency. The saved [baseline report](benchmarks/legacy-hp60c-negatives-20261009.json) uses 20 manually reviewed cone-free HP60C frames: zero false positives, median 20.86 ms, p95 23.15 ms at confidence 0.54/NMS 0.50 on this host. Precision, recall and mAP are undefined because the recording has no cones. Distant/occluded positive HP60C evaluation is still pending.

## Tests and acceptance

From repository root:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=phase_01_perception/src python -m pytest -q phase_01_perception/tests
```

The plugin environment variable is needed on the tested host because its unrelated installed `anyio` pytest plugin conflicts with system pytest. Tests cover camera mocks and packed frame format, disconnects, freshness/pairing, depth rejection and transform, Delaunay/Gabriel, boundaries, ordered gates, curvature, display-only temporal smoothing, pipeline, UI rendering, and recording roundtrip. See [actual results](PHASE_01_TEST_REPORT.md).

Live 640×480 HP60C acquisition, SDK registration reporting, recording/replay, and the graphical viewer ran successfully on the x86_64 host. The browser dashboard passed HTTP/stream/overlay API tests, displayed live camera data after USB reconnection, and showed the new counters. In one measured 10.152-second interval it received 126 paired frames, processed 100, and overwrote 26 unread pairs (12.41 acquisition FPS, 9.85 processing FPS, 20.63% mailbox-overwrite rate). These numbers are for the current x86_64 host and room scene, not the Raspberry Pi. The measured bench pose is configured and passes a synthetic integration test, but acceptance remains pending for physical RGB/depth alignment, surveyed localization error, independent labeled **positive-cone** HP60C detection metrics, and ARM64 runtime. Phase status is **TESTING**, not VERIFIED.

## Operating envelope and release evidence

This build is for a **stationary bench**. The [YDLIDAR HP60C product specification](https://www.ydlidar.com/products/view/29.html) lists a 0.2–4 m range and 73.8° horizontal / 58.8° vertical field of view. These published limits are not a measured, reliable cone detection horizon. At 15 km/h, the illustrative assumptions of 200 ms response, 2.5 m/s² braking and 0.5 m margin yield about 4.8 m stopping distance, beyond the stated 4 m sensor range. No moving-vehicle speed is approved; a safe speed requires measured sensing, brake, E-stop and latency data. [ISO 21448:2022](https://www.iso.org/standard/77490.html) informs the intended-functionality risk and verification approach here; no compliance claim is made.

Proposed release-review gates are ≥98% recall and ≥99% precision within a defined cone envelope, ≤0.30 m p95 surveyed ground-position error, and ≤150 ms valid output age on the target Pi. These are **targets, not measured results**. Physical gate associations, independent field cases, fault recovery, braking and safety circuitry need separate validation. Passing software tests never grants vehicle operation approval. The 19,385 unique cone-positive image count remains an internal final-model qualification target, not a universal accuracy threshold; diversity and independent measured performance are also required.

Generate a machine-readable evidence inventory from the Phase 1 directory:

```bash
PYTHONPATH=src python -m autokart_perception.release_audit --run-tests \
  --evaluation-report benchmarks/legacy-hp60c-negatives-20261009.json \
  --output benchmarks/new-release-audit.json
```

The command refuses an existing output path and always sets `vehicle_operation_approval` to `BLOCKED`. It marks missing positive-case, surveyed, Raspberry Pi, braking and field evidence `NOT_EXECUTED`. The current [release report](benchmarks/release-audit-20261009-v5.json) was generated with 56 passing tests and the real 20-frame cone-free HP60C baseline. To add measured evidence later, pass reviewed evaluation and survey reports with `--evaluation-report` and `--survey-report`; they still require human review and do not unblock vehicle operation.

## Troubleshooting and limitations

- `SDK_ADAPTER_NOT_BUILT`: run `./phase_01_perception/build_native.sh` from root.
- `CAMERA_DISCONNECTED`: run `../check_camera.sh`; check USB power, cable, vendor `3482:6723`, and permissions. The CLI waits for reconnection until `--connect-timeout`.
- `MODEL_NOT_AVAILABLE`: run `./models/download_model.sh`; a checksum mismatch refuses the built-in checkpoint. Custom compatible one-class weights load as `unknown` with `USER_MODEL_UNVERIFIED` status and cannot establish colored gates.
- `DEPTH_ALIGNMENT_UNAVAILABLE`: SDK registration flag is false or unknown. Check camera configuration; do not assume pixel alignment.
- `MOUNT_CALIBRATION_UNAVAILABLE`: measure and set `camera_to_kart`.
- `CENTERLINE_AMBIGUOUS`: too few valid opposite-side observations, inconsistent widths/headings, crossed gates, or unresolved geometry-only association. Orange-only results remain provisional even when a local candidate exists.
- The desktop UI requires a graphical session and Python Tk when OpenCV has no HighGUI; the browser UI does not. `--headless` prints concise JSON diagnostics. CPU and memory values are process level. The vendor SDK may log a transient `getMjpegSize` error and the first read may time out during stream startup; the observed 640×480 stream recovered. There is short display-only temporal smoothing, but no persistent tracking, camera pose correction, or vehicle safety claim.
