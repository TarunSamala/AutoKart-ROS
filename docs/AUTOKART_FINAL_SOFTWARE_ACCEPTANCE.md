# AutoKart — Software Status Report (2026-10-18)

> **CORRECTION (2026-10-18, later):** statements in this document that no HP60C is attached are WRONG or STALE. An HP60C (USB `3482:6723`) is attached and streams RGB+depth at ~11-13 fps via the vendor SDK; the live Vehicle Dashboard was verified against real frames (see `ui_split/VALIDATION_MATRIX.md`). Items below marked NOT TESTED because "no camera" are now testable; those not re-run in `ui_split/VALIDATION_MATRIX.md` remain NOT TESTED. Calibration is still UNVERIFIED and no qualified metric geometry exists.

This is a checkpoint, not a final RC-07/vehicle-motion acceptance. It covers the work done against the Oct-23 master execution plan in this session only, on `Jarvis` (x86_64 dev workstation, no HP60C, no Raspberry Pi). See `docs/AUTOKART_OPEN_ISSUES.md` for the full itemized gap list this table summarizes.

## Status matrix

| Subsystem | Software implemented | Integration tested | Physical validation |
|---|---|---|---|
| HP60C driver (`camera.py`) | PASS — robust disconnect/stale/mismatch handling, already present; 2 environment defects found & fixed | PASS — 7/7 `test_camera.py` (mocked SDK) | NOT TESTED — no camera |
| YOLO26s inference | PASS — real inference run against the actual `best.onnx`, correct detections on a real cone photo | PASS — pipeline + depth + decode logic, 163/163 perception suite | NOT TESTED on HP60C; measured 266 ms/3.75 FPS on this CPU only, flagged as likely too slow |
| Depth association | PASS — existing robust-statistics implementation, unmodified | PASS — `test_depth.py` et al. | NOT TESTED — no live depth stream |
| Camera geometry (pixel→camera→vehicle) | PASS — full R,t transform chain verified present and correctly separated from sightline-angle confusion | PASS — `test_mount.py`/`test_geometry.py`/`test_calibration*.py`, 27/27 | NOT TESTED — no physical survey |
| Bumper-relative distance | PASS — architecturally correct (subtracts within base_link frame, not raw camera-ray) | PASS — safety-envelope classification tests | BLOCKED — `base_link` origin never physically surveyed, so the offset constant stays PROVISIONAL |
| Generic (non-cone) obstacle detection | **FAIL/NOT IMPLEMENTED** — `/perception/obstacles` has a consumer and no producer | N/A | NOT TESTED |
| Safety envelope | PASS — CLEAR/BRAKING/CRITICAL/UNKNOWN, runtime-parameter-tamper rejection, provenance labeling | PASS — 226 tests total across this project touch it directly or indirectly; live DDS state-transition and dropout evidence in `RC06_FINAL_ACCEPTANCE_REPORT.md` | NOT TESTED — no real obstacle ever measured |
| Standalone Mission Control (`/ops`) | PASS — hard-locked to REAL_KART, no sim/lab code path, fails closed on disconnect | PASS — 47/47 vitest incl. 4 new; independently re-verified in a live browser this session | NOT TESTED against a real gateway/hardware (jsdom + mocked WebSocket only) |
| Separate simulation (default route) | PASS — untouched, still fully functional | PASS — pre-existing tests still green | Not applicable |
| Raspberry Pi deployment | NOT ATTEMPTED | NOT ATTEMPTED | **NOT TESTED** — no Pi reachable |
| Independent emergency stop | Outside this software scope | Not established | NOT TESTED |
| Actual braking performance | Outside this software scope | Not established | NOT TESTED |

## Launch command (as it exists today)

```bash
# Workspace build (isolated install dir recommended if colcon caches have stale paths):
colcon build

# Combined diagnostic stack (synthetic perception mode, no hardware needed):
ros2 launch autokart_bringup rc03_rc05_diagnostics.launch.py \
  perception_mode:=synthetic safety_data_source:=SIMULATED launch_gateway:=true

# Mission Control (dev/sim, full switcher):      http://<gateway-host>:5173/        (phase_sim, npm run dev)
# Mission Control Live (locked to REAL_KART):     http://<gateway-host>:5173/ops
# HP60C camera/perception diagnostic dashboard:   http://<gateway-host>:5173/live
```

Live HP60C operation requires `perception_mode:=live camera_library:=<path> camera_configuration:=<path> model_path:=models/yolo26s/best.onnx model_manifest:=models/yolo26s/best.json` — structurally wired, **not tested against real hardware**.

## Where everything is

- Architecture/inventory: `docs/ros2_system_inventory.md`
- Interfaces/topics: `docs/AUTOKART_ROS2_TOPICS.md`
- Camera geometry/calibration: `docs/AUTOKART_CAMERA_GEOMETRY.md`
- YOLO26s integration: `docs/AUTOKART_YOLO26S_INTEGRATION.md`
- Safety envelope: `docs/AUTOKART_SAFETY_ENVELOPE.md`
- Mission Control: `docs/AUTOKART_MISSION_CONTROL.md`
- Deployment/launch: `docs/AUTOKART_DEPLOYMENT.md`
- RC-06 live-pipeline evidence (the only true end-to-end DDS→gateway→browser test run this project has done, with a synthetic sensor standing in for HP60C): `docs/RC06_FINAL_ACCEPTANCE_REPORT.md` + `docs/RC06_evidence_2026-10-18/`
- Consolidated open issues: `docs/AUTOKART_OPEN_ISSUES.md`

## Regression total, this session

**253/253 automated tests passing** (163 perception library + 43 gateway/safety-bridge + 47 frontend; see `AUTOKART_OPEN_ISSUES.md` §"Full regression snapshot" for the per-suite breakdown), 16/16 ROS2 packages building, zero known regressions, two real environment defects found and fixed (not source bugs — stale pip install, missing optional-dependency install), one real production gap identified and explicitly left open rather than hidden (`/perception/obstacles` has no producer).

## What this report does not claim

No real HP60C camera has ever produced a frame for this project in this session. No real obstacle has ever been measured against the safety envelope. No Raspberry Pi has run any of this. Every "PASS" above is a software/synthetic-data/replay verification, stated as such — this is the honest reading of the master plan's own instruction: "Never fabricate a hardware PASS."
