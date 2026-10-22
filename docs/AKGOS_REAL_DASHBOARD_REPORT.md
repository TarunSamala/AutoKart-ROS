# AKGOS real dashboard + HP60C guided calibration — 2026-10-16

**Verdict: APPROVED WITH CONDITIONS for read-only diagnostics; vehicle NO-GO.** No firmware, actuator or VCU change. Nothing publishes `/control/motion_command` (measured: 0 publishers); the controller candidate topic has 0 subscribers.

Evidence: [`akgos_evidence_2026-10-16/`](akgos_evidence_2026-10-16/) (probe scripts, measurements, regression logs, annotated live frame).

## Root causes of the observed problems

| Observed | Cause found | Fix |
|---|---|---|
| 2.9 FPS, 500 ms frame age | (1) The REAL dashboard was fed by a perception node left running from an earlier session, from a stale `/tmp` build with `period_s:=0.3` (cap ≈ 3.3 Hz). (2) Each tick spent ~170 ms in `sensor_msgs/Image.data = bytes`: rclpy validates every byte in Python (38.4 M `isinstance` calls in 25 ticks under the profiler). | Image payload assigned as `array('B')` (rclpy typed fast path). Default `period_s` 0.02 s; the HP60C read blocks until the next 12.4 Hz frame. Stale processes stopped. |
| 477 ms capture-to-gateway | The same image-serialization stall plus polling delay. | As above. |
| 208 ms processing | YOLO26s on CPU is 104–134 ms (87% of pipeline time). `onnxruntime` is not installed, so the OpenCV DNN fallback is used. ONNX Runtime measured 86 ms in isolation but gave no gain in the loaded full graph. | Not changed; the model is fixed. |
| `MANUAL_RESTART_REQUIRED` | The pipeline latched on any SENSOR_FAULT reason, including per-frame data-quality results. One frame whose only cone foot is clipped gives `METRIC_LOCALIZATION_FAILED` and latched the node permanently (reproduced in `tests/test_fault_latch.py`). The HP60C itself raised `FRAME_TIMEOUT`/`CALIBRATION_UNAVAILABLE` only during warmup (60 s probe: 0 errors after the first frame). | Only integrity faults latch (`LATCHING_FAULTS` in `pipeline.py`): source, replay integrity, inference failure, frame format, alignment, mount, model, calibration mismatch, frozen and out-of-order frames. Data-quality frames stay invalid without latching. Operator clear: `ros2 service call /autokart_perception/reset_sensor_fault std_srvs/srv/Trigger`. Camera loss after streaming still latches, as before. |
| Orange cones labelled yellow | **Not a mapping bug.** ONNX class order (`blue, large_orange, orange, unknown, yellow`) matches the manifest, and preprocessing converts BGR→RGB correctly. The bench cones are amber, and YOLO26s labels them yellow (641/642 boxes in 20 s; swapping channels turns them `blue`). See [`live_frame_yolo_labels.jpg`](akgos_evidence_2026-10-16/live_frame_yolo_labels.jpg). | Needs HP60C-specific labelled data or retraining (out of scope). Geometry must keep not trusting colour. |
| Empty bird's-eye / missing odom, map, planner | No `/autokart/odom` producer exists. The gateway world view is anchored on odometry, and mapping rejects input without odometry. This is correct fail-closed behaviour. | Dashboard now states `WAITING FOR QUALIFIED GEOMETRY` and `NO /autokart/odom PRODUCER` instead of showing a blank view. |
| Depth scale and mount unverified | No surveyed calibration exists. | Guided calibration workflow (below). |

## Before / after (same laptop, live HP60C, unchanged 150 ms freshness gate)

| Metric | Before | After |
|---|---|---|
| Perception output rate | 2.7 Hz (node), 2.9–3.0 FPS (gateway) | 6.5–8.4 Hz (varies with host load; load average 7.7) |
| Capture → gateway | 371–477 ms | 137 ms (idle) to 218 ms p50 / 278 ms p95 (loaded) |
| Frame age at assessment | 152 ms node / 428–500 ms at UI | 118 ms (idle) to 194 ms p50 (loaded) |
| `FRAME_TOO_OLD` share | 60% | 0–93%, depending on CPU contention |
| Latched fault | Permanent after a data-quality frame | None during runs; integrity faults only |

The freshness limit was **not** relaxed. On this CPU, YOLO26s inference (~100–175 ms) plus up to 80 ms waiting for the next frame exceeds 150 ms under load. A Raspberry Pi 4 will be slower and must be measured before any rate is claimed.

Freshness test: the perception node was killed while the gateway kept running. The camera pair went `CAMERA_PAIR_STALE` within ≤1.5 s (gateway display limit 1.0 s), the perception stage went `STALE`, and `ros_connected` became false. A restarted graph re-acquired the camera with no latched fault. Physical USB unplug was **not** tested.

## Dashboard changes (existing Mission Control, no second dashboard)

The REAL ROS live panel now shows:
- **Frame data:** RGB/depth stamps and sequences, receive FPS, inference and processing time, and frame age.
- **Model identity:** file, SHA, backend and stage.
- **Latched fault:** the reason, plus the reset command.
- **Cones:** each detection marked valid, or rejected with its localization and validation reasons.
- **Status lines:** odometry status with the reason it is missing, and the geometry state.
- **CAMERA CALIBRATION** block: `UNVERIFIED / VERIFIED / FAILED`; held-out and survey position error; depth scale and camera mount `Verified / Unverified`; and the calibration reasons.

The gateway derives `depth_qualified` from verified calibration instead of hardcoding it. The 3D world shows `WAITING FOR QUALIFIED GEOMETRY` when there is no qualified odom-frame geometry. Simulated geometry is never substituted in REAL mode.

## Guided calibration (see [runbook](runbooks/HP60C_CALIBRATION.md))

`python -m autokart_perception.calibration_session plan | template | capture | evaluate`

- **Fit / validate separation:** fit and validation use different physical placements. PASS/FAIL is computed only on validation stations by the unchanged `validate_survey`, and fit cones are never re-scored.
- **Capture rejections:** stale frames, invalid depth, clipped cone feet, missing or ambiguous detection association, and a changed device configuration.
- **Report contents:**
  - fitted depth scale, height and pitch, with 1σ uncertainty
  - held-out X/Y and position error by range
  - rejection statistics and device identity
  - PASS/FAIL
- **Artifact:** written only on PASS. It is bound to the mount hash, the camera intrinsics (a per-frame mismatch reports `CALIBRATION_CAMERA_MISMATCH`), the SDK configuration hash, the USB identity, the approved grid and the operator.
- **Field of view:** with the real intrinsics (fx 585.27, cy 239.96) at the nominal 0.30 m height and 4° pitch, the **0.5 m row is outside the image in all three thirds**. The proposed substitute, which needs a named approver in the stations file, is **0.65 m** (Y ≈ +0.27 / +0.02 / −0.22 m). The 1, 2 and 3 m rows are visible. An unapproved alternative grid is refused by `verify_artifact`.
- **No physical survey yet:** it has not been performed, so no verified artifact exists and LIVE cones remain unqualified. `MODEL_UNVERIFIED` also blocks LIVE cones until the YOLO26s manifest leaves `EXPERIMENTAL_HP60C_VALIDATION_PENDING`.

## Regression (after changes)

Phase 1 162 passed (was 149) · kart 150 passed · ROS adapters + gateway + bridge 49 passed · frontend 35 passed + production build · ESP32 static 9 passed.

## Open blockers

1. Physical HP60C survey (guided workflow ready).
2. YOLO26s qualification on HP60C data: colour confusion and `MODEL_UNVERIFIED`.
3. An odometry producer (wheel encoder + IMU) for mapping and the world view.
4. Pi 4 timing measurement.
5. Physical unplug and reconnect test.
6. Independent E-stop and brake evidence.
