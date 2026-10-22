# Minimal completion plan

Audit date: 2026-10-16. These are proposed gates, not passed milestones. Keep vehicle_authorized=false and VCU transmission disabled until required evidence and approval exist.

## M1 — Calibrated cone observations

**NOT PASSED.** October 15 ten-minute capture had zero validated metric cones; depth scale, mount and surveyed accuracy unverified.

1. Freeze one camera runtime, detector/model hash, camera SDK/firmware and config; record actual mounting/frame.
2. Measure RAW16 scale and RGB/depth registration with independent surveyed targets across 0.3–4 m and image positions.
3. Survey camera-to-base height, translation, pitch/roll/yaw and uncertainty.
4. On target device test orange/blue/yellow; score class confusion, box foot, distance-stratified XYZ error, false detections, invalid depth and timestamp age.
5. Keep unknown colors/unverified points out of mapping; set limits from stopping envelope.

**Exit:** Reproducible data/hashes, calibration and surveyed accuracy/class report, fail-closed stale behavior and target Pi timing.

## M2 — Localization and mapping

**ALGORITHM CANDIDATE EXISTS; SENSOR INTEGRATION NOT PASSED.** shared_core defines bounded pose-aware mapping/winding; no qualified live pose.

1. Define base_link/odom/map frames, clock, SI units, covariance, validity and expiry.
2. Integrate rear-axle encoder or independent wheel speed and IMU; calibrated steering feedback if available. Document scale, direction, slip and reset behavior.
3. Compare synchronized pose with surveyed stationary/straight/circle motion; inject dropouts, timestamp reversal, slip and jumps.
4. Feed fresh pose and M1-qualified cones to map; test expiry, association, loop closure and recovery. Invalidate path on stale/uncertain input.

**Exit:** Synchronized logs, pose error/drift bounds, frame contract, fault tests and deterministic map replay.

## M3 — Skidpad and braking planning in simulation

**SOFTWARE CANDIDATES EXIST; EVENT ACCEPTANCE AND REAL OBSERVATION NOT PASSED.**

1. Select shared-core candidate; label browser LAB/legacy corridor as simulation/short-range diagnostic.
2. Make skidpad fixture verify two-loop geometry, left/right sequence, branch, crossover and lap count. Test gaps, colors and pose drift.
3. Require observed map and fresh pose; compare geometry/clearance to event definition without leaking exact world truth to planner.
4. Build braking-zone and stop-distance planning in replay/SIL with explicit speed, marker distance, pose uncertainty, delay and capability; fault lost marker/pose, unavailable/stuck brake and overshoot.
5. Label simulated deceleration as simulation; do not infer real capability.

**Exit:** Deterministic SIL/replay, ground-truth comparison, fault tests, mission-rule acceptance and justified speed/stopping envelope. Simulation alone does not release vehicle.

## M4 — Raspberry Pi / ESP32 integration

**ROS CONTRACTS COMPILE; LIVE DATA/COMMAND TRANSPORT NOT PASSED.**

1. Identify actual Pi image, ROS, camera SDK, services, model/config/calibration hashes, ESP32 board/firmware and wiring/protocol.
2. Add read-only ROS publishers for qualified cones/diagnostics and encoder/IMU odometry/VCU feedback. Verify TF, clock, QoS, sequence, range and validity.
3. Connect shared planning/controller in SIL with command ownership, sequence, timestamp/expiry, limits, arbitration and stale-data safe response.
4. Before transport, implement mock VCU; test serialization, restart, USB loss, ROS shutdown and stale/duplicate/reordered/fault cases.
5. Select one firmware variant only after wiring compatibility. Preserve backup and separately authorize isolated bench flash.
6. Prove telemetry/feedback on traction-isolated bench. HTTP dashboard is not autonomy protocol.

**Exit:** Pi ROS graph/timing, replay trace, protocol conformance, bench-only command/feedback/timeout/fault results. No motion before M5.

## M5 — Independent safety validation

**NOT PASSED / CRITICAL UNKNOWN.** No evidence proves software-independent E-stop or safe mechanical stop on power loss.

1. Reconcile schematic and as-built E-stop, relay/contactor, traction inhibit, feedback, power and mechanical brake. Stop must not depend on Pi/Wi-Fi/ROS/ESP32 scheduling/radio.
2. Prove boot/reset/power-loss defaults and no unexpected re-arm after power restore.
3. Bench-test brake application/release, output feedback, jam/linkage break, stuck output, MCU freeze, cable break and comm loss with traction isolated.
4. Measure loss-to-safe latency and brake response; verify latched faults and deliberate manual reset.
5. Obtain qualified human safety review before closed course.

**Exit:** Wiring review, independent stop measurements, fault logs, brake/traction response, reset criteria and signed acceptance.

## M6 — Low-speed vehicle validation

**NOT RUN.** This audit does not approve physical operation at or below 10 km/h.

1. Establish controlled site, barriers, trained observer/driver, rescue/abort plan, physical isolation and speed ceiling; progress stationary→wheels-up→very-low-speed.
2. Verify sensors, feedback, steering sign/limit, speed and brake plus independent E-stop before trajectory.
3. Measure stop distance over load/surface; derive speed envelope from worst-case result and latency.
4. Validate braking separately from skidpad with manual stop operator.
5. Then validate skidpad at low speed with observer and synchronized sensor→pose→plan→command→feedback logs.

**Exit:** Signed run sheets, raw logs, measured speed/stop/steering, abort/fault record and repeatable criteria. A demo is not production readiness.

## Order and immediate task

Order: **M1 calibration → M2 pose/map → M3 SIL missions → M4 Pi/ESP32 integration → M5 safety acceptance → M6 low-speed validation.** As-built safety inspection can start alongside M1–M3; M6 remains blocked until M5.

**Immediate engineering task: create and review the verified as-built baseline.** Identify the real Pi runtime/model/config/calibration, ESP32 image, sensor mounts/wiring, brake feedback and independent E-stop/traction path. Record hashes/photos/schematics and explicit UNKNOWNs. This selects the implementation to integrate without changing production code.
