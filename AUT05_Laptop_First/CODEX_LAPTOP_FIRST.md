# AUT-05A+B+C implementation — JARVIS LAPTOP FIRST

You are executing on the user's actual Ubuntu laptop, **Jarvis**, at `/home/mark1/Desktop/autonomous_kart`. Implement, test and document a single non-actuating AUT-05 diagnostic application locally. **DO NOT deploy anything to the Raspberry Pi now.** Do not request clarifications already answered; inspect what exists and use evidence. If live hardware is not plugged into Jarvis, complete replay/simulation integration and mark live hardware tests `NOT_TESTED`, not PASS.

## Verified project facts / exact paths
- First-party AUT-03 code: `autonomy_stack_phase1/phase_01_perception/src/autokart_perception`, wheel in `.../dist/autokart_perception-0.2.0-py3-none-any.whl`, `SDK.md`, `examples/geometry_sdk.py`; prior report: clean x86 wheel installation and 84 perception tests, 149 kart tests, 23 phase_sim tests. Rerun, do not assume.
- `autokart_apps/run_core.sh` and `autokart_apps/run_lab.sh` are at **project root**, not within `autonomy_stack_phase1`.
- Existing camera wrapper scripts may be at `autonomy_stack_phase1/` (reported local) or a separate `YDLIDARHP60C/` checkout. Detect, don't duplicate or overwrite.
- Camera source: `https://github.com/TarunSamala/YDLIDARHP60C` main (standalone **native SDK viewer**, not a Python API or ROS node). It has `setup.sh`, `build.sh`, `check_camera.sh`, `run_viewer.sh`, `platform.sh`, `test_camera.sh`, vendor compatibility patch. `vendor/` is ignored and downloaded by `setup.sh` from a pinned-checksum community mirror. The viewer `exec`s `./ascamera` after selecting x86_64/ARM64 libraries.
- HP60C USB ID `3482:6723`. Previously enumerated **on Pi**, not proven present on Jarvis. The README describes synchronized vendor SDK RGB and RAW16 depth at up to 640x480/20fps; real fps, encoding, units and alignment require measurement.
- Current model is **orange-cone-only**. No blue/yellow claims or unqualified color-independent planner. Current kart target speed is **10 km/h**, but vehicle operation is forbidden in this task.

## Non-negotiables
1. Preserve AUT-03 mathematics/APIs; fix only demonstrable defects with regression tests. Existing tests must still pass.
2. `vehicle_authorized=false` always. No VCU hardware commands, no armed routes, no actuators, no traction, no unattended Pi SSH writes, no firmware flashing.
3. Do not call synthetic, missing-calibration, zero-depth or stale frames LIVE. Timestamp and source every frame and derived output.
4. Provide an explicit rearward-compatible architecture for later Pi ARM64 and ROS 2. ROS 2 is **not mandatory** for laptop browser visualization; do not waste time installing it if not needed.
5. Use only one owner of the physical camera. Never rely on scraping the native viewer window or polling one-off JPG snapshots for live inference.
6. Project pixel ↔ 3D using verified intrinsics, distortion, RGB/depth registration and mounting extrinsics; fail closed when unknown. Do not pretend pixels and metres are interchangeable.
7. Read LICENSE files for SDK/vendor sources, and document redistribution constraints. Avoid committing `vendor/`, camera captures, binaries or credentials.

## Work order — execute, don't stop after a plan
### 0. Inventory and baseline
- Inspect project tree, Git status, documented startup scripts, Python package source, `camera.py`, native HP60C bridge if any, detectors, pipeline, tests, existing web UIs, `kart` and `phase_sim`.
- Print tests and run baseline; capture logs in `docs/aut05_evidence/laptop/` (ignore generated recordings in Git).
- Inspect camera wrapper + SDK C++ sample callback surface. Decide how to expose paired frames to existing Python application: reuse current native bridge if functional; otherwise implement a small native frame publisher/adapter (Unix-domain socket or bounded shared memory) with a documented schema, queue limits, timestamps, raw RGB/depth arrays and calibration metadata. Avoid adding a second camera SDK implementation.
- Inspect and lock dependency versions with a supported x86 development environment.

### A. Laptop camera backend
Implement common `FramePair`/`CameraSource` interfaces with three backends: `hp60c_live`, `recorded_rgbd`, `synthetic`. Required fields: acquisition timestamp (monotonic), sequence, source, camera model/serial, width/height, RGB format, raw depth dtype/scale, intrinsics, distortion, extrinsics, calibration identifier/checksum, frame age, validity/reasons. Ensure RGB/depth time pairing, monotonic counters, bounded queues, no stale frame reuse. `hp60c_live` must use vendor SDK, not V4L2 as assumed depth and not saved-image polling.

If HP60C is physically on Jarvis, verify `lsusb -d 3482:6723`, compile/check native SDK, capture at least 50 **actual** synchronized frame pairs, inspect RAW16 depth values and measure rates. If absent, validate with real recorded RGB-D only where available; synthetic is useful for contracts but cannot pass physical A.

### B. Single AUT-03 live pipeline and dashboard
Connect camera source → existing detector → depth association/valid 3D cone coordinates → tracking (measured vs predicted) → original Delaunay, Gabriel, gates, centerline → diagnostic Pure Pursuit. Do not authorize skidpad state machine until validated for loop topology and pose; do not force gate generation from orange-only cones if boundaries ambiguous.

Publish one immutable versioned `FrameResult` identified by `frame_id`, `seq`, `source`, `acquired_ns`, `processed_ns`, `expires_ns`, `camera_calibration_id`, `model_version`, `validity`, `reason_codes`, and geometries with coordinate frames. Camera overlay, depth preview and BEV MUST render the same `seq`. Overlay bounding boxes/confidence/cone depth; project measured 3D geometry only through valid calibration. BEV in metres: kart footprint, measured/tracked cones, Delaunay, Gabriel, gates, centerline, proposed trajectory. Visually distinguish observed/predicted/invalid cones and report unknown track-boundary colors.

Extend existing `phase_sim`/mission-control UI instead of making a duplicate UI unless the existing UI cannot be reused (justify). Dashboard should also show camera FPS, processed FPS, frame age, p50/p95/p99 latency, disconnected, depth invalid, model unqualified, no pose, stale geometry, capture record and replay. Stale frames must CLEAR geometry and proposals and show a conspicuous status, rather than leave previous path drawn as current.

### C. Runnable laptop package
Implement a documented launcher at PROJECT ROOT: `./autokart_apps/run_live.sh --mode replay|synthetic|live`. Use source-detected/verified CLI arguments, not invented ones. One application with config in one place, programmatic health and readiness, visible error/reason codes, bounded queues, structured logs, evidence recording, reproducible replay, controlled shutdown, cleanup and separate tests.

Dashboard binds **127.0.0.1** unless operator deliberately enables authenticated LAN mode. No open actuator endpoint. Auto restart, when configured, enters DIAGNOSTIC and never arms. Startup on Pi is a later deployment phase; do not enable systemd yet.

### Verification
- Repeat reported AUT-03 84/149/23 test suites and compare actual results.
- Clean-wheel install and execute `examples/geometry_sdk.py` from `/tmp` without `PYTHONPATH`, OpenCV or kart dependency.
- Demonstrate `--mode synthetic` and `--mode replay` on Jarvis; test invalid/noisy cones, duplicates, outliers, depth zeros, dropped/misaligned frames, reversed timestamps, disconnect, process crash, missing calibration, bad model output, invalid path, dashboard reconnect, snapshot synchronization and replay determinism.
- When camera is on Jarvis: execute LIVE tests, display RGB+colorized depth+BEV with overlay, save annotated real fixtures, evaluate surveyed metric positions and measure p50/p95/p99 end-to-end latency and memory.
- Unknown camera attributes, SDK quirks, registration and calibration must be BLOCKED instead of guessed. No VCU controls in this release.

## Release evidence and definition of done
Write `docs/AUT05_LAPTOP_RELEASE.md` and evidence files for each of A/B/C: test commands, outputs, actual file names, Git commit, camera source/status, snapshots and paired-frame sample, toolchain versions, measurements, risks and reproducible launch instructions. Mark individual gates PASS / FAIL / NOT_TESTED. Do not claim production quality from generated code or synthetic-only output.

Local acceptance is: on Jarvis, `./autokart_apps/run_live.sh --mode replay` (and `--mode live` **only if HP60C is connected**) opens a single dashboard with annotated RGB + depth + correctly synchronized BEV; equivalent SDK outputs and no regressions; fault cases clear stale path and always keep `vehicle_authorized=false`. When hardware absent, A-LIVE and full B-LIVE remain blocked.

**Final answer format:** `FILES CHANGED | COMMANDS ACTUALLY RUN | A/B/C VERDICTS | MEASURED PERFORMANCE | BLOCKERS | ONE NEXT ACTION`.

Start by running the local audit and implementing. Do not modify Pi, install a new ROS distribution, or attempt to test the vehicle.
