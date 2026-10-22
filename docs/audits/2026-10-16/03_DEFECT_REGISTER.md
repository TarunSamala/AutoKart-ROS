# Defect register

Audit date: 2026-10-16. Findings prioritize operational consequence. References are repository-relative. Fixes are proposals only; no production code changed.

## P0 — unsafe or blocks vehicle operation

### AK-P0-01 — End-to-end autonomy command path absent

- **Subsystem:** Integration, ROS, VCU.
- **Evidence:** kart/ros2_ws/src/autokart_bringup/launch/full_stack.launch.py:6-16 starts only vcu_bridge. autokart_sim_bridge/vcu_bridge.py:23-34 says runtime binding is deferred and stores only latest MotionCommand. autokart_control/CMakeLists.txt:1-5 and autokart_safety_bridge/CMakeLists.txt:1-4 install config/package metadata only. Phase 1 __main__.py:18 identifies no actuator interface; live_mission.py:348-349 sets ready=false and vehicle_authorized=false.
- **Consequence:** A topic or dashboard does not control or monitor the vehicle. Camera→pose→planner→controller→safety→ESP32 chain absent.
- **Minimal fix:** Implement/test read-only perception and odometry ROS adapters, then software planner/controller wiring. Keep hardware command output disabled until VCU/safety contract validated.
- **Verification:** Replay checks timestamps, frames, units, expiry, stale rejection and determinism; separately validate isolated bench round trip.

### AK-P0-02 — Independent emergency stop and power-loss stop unproven

- **Subsystem:** Safety, traction disable, brake.
- **Evidence:** AutoKart_RC_Dynamics_ESP32S3_v1/main/main.c:151-155 samples auxiliary E-stop GPIO; it does not prove main contactor opens. Nested AutoKart_ESP_IDF_R86mini_WebStop/README.md:64-87 calls external wired E-stop the independent stop and states software stop is not it. XIAO_C3_RA02_ESTOP_PRODUCTION/lora_driver.py:37-42 returns success without transmission; receiver_esp32s3_rp2350.py:34-39 prints instead of actuating. No current wiring, power-loss brake test, contactor feedback or loaded stopping measurement found.
- **Consequence:** No evidence traction is removed on Pi/MCU/network/software failure or kart stops on brake/actuator power loss.
- **Minimal fix:** Document/inspect actual hardwired chain, contactor and mechanical fallback. Prove de-energized safe state and deliberate reset with traction isolated. Do not substitute LoRa or software E-stop.
- **Verification:** Electrical review, continuity/logic, open-wire/stuck-output/power-loss injection, then loaded stop test per approved plan.

### AK-P0-03 — Root firmware needs brake feedback reported absent

- **Subsystem:** ESP32/VCU, brake.
- **Evidence:** AutoKart_RC_Dynamics_ESP32S3_v1/main/main.c:156-165,276-279 requires two AS5600s and faults when either is missing; brake guard compares step count with brake_measured at :217-219. Project context says mechanical brake has no position encoder. Nested WebStop README.md:72-76 says no steering or brake encoder.
- **Consequence:** Root candidate cannot arm as wired; nested candidate has open-loop brake travel. Neither proves brake response.
- **Minimal fix:** Identify installed image and sensor placement. Select valid feedback or fail-safe mechanical brake contract before integration; do not fabricate feedback.
- **Verification:** Bench sensor/actuator calibration, then disconnected sensor, jammed linkage and power-loss tests.

## P1 — blocks end-to-end autonomy

### AK-P1-01 — Live cone coordinates are not calibrated metric observations

- **Subsystem:** Perception/calibration.
- **Evidence:** _autokart_audit/hp60c-geometry-10min-20261015.json: 600 s, 2,523 unique frames, 15,336 detections, 9,827 localization candidates, 0 validated metric cones and 0 geometry frames. Rejections include 9,827 DEPTH_SCALE_UNVERIFIED, 9,827 MOUNT_TO_BASE_LINK_UNVERIFIED and 4,931 CONE_FOOT_CLIPPED. Calibration grid has no surveyed cones and is NOT_EXECUTED.
- **Consequence:** Geometry cannot be trusted for physical route choice.
- **Minimal fix:** Survey targets over 0.3–4 m; calibrate RAW16 scale, RGB/depth registration and camera-to-base extrinsics; qualify color separately.
- **Verification:** Report per-range position error, uncertainty, false positives/misses, rejection rate and provenance.

### AK-P1-02 — YOLO26s stream fails freshness gate

- **Subsystem:** Perception timing/model.
- **Evidence:** _autokart_audit/yolo26s-phase6-evidence-20261015.json: 118 synchronized unique frames, zero HTTP/sequence errors, max 7 detections; inference p95 139.3 ms; frame age p95 354.1 ms vs 250 ms limit; pass=false and authorized_count=0. Ten-minute capture frame-age p95 378 ms/max 821 ms.
- **Consequence:** Perception may reach planning too late. The 10 km/h target does not waive freshness gates.
- **Minimal fix:** Profile one selected pipeline on target Pi under intended load; apply measured configuration/model changes while retaining stale rejection.
- **Verification:** Long Pi run with p50/p95/p99 age, drops, detection quality and stale recovery; limits derived from measured stop envelope.

### AK-P1-03 — Live localization input absent

- **Subsystem:** Localization/odometry.
- **Evidence:** autokart_apps application path passes pose=None in HP60C mode (application.py:131-134); live_mission.py:210-212 creates pose only in sim mode. shared_core.py:160-180,334-338 refuses map/planning without fresh pose. No connected rear-axle encoder, IMU or steering publisher found.
- **Consequence:** No persistent map, loop progress or valid live controller candidate.
- **Minimal fix:** Bring up read-only odometry with frames, time, speed, yaw, uncertainty, wheel scale/IMU calibration and invalid/stale status.
- **Verification:** Synchronized stationary, straight, circular, slip and dropout replay; bounded drift/recovery criteria.

### AK-P1-04 — Local corridor cannot represent complete skidpad loop

- **Subsystem:** Geometry/planning.
- **Evidence:** boundaries.py:20,44,50-53 requires points ahead and increasing X; gates.py:121-127 uses forward-X order/delta; centerline uses same local progression. A loop includes non-increasing-X sectors. shared_core.py has observed-ring/winding candidate but no qualified live pose/map.
- **Consequence:** Local corridor tests cannot establish full lap path, crossover choice, lap count or clearance.
- **Minimal fix:** Reuse shared-core observed-ring candidate behind fresh-pose contract; validate full-course surveyed/replay cases. Do not extend the local forward-X routine with threshold edits.
- **Verification:** Both loops, crossover, missing cones, wrong colors, pose drift/dropout, branch ambiguity and lap assertions in SIL/replay.

### AK-P1-05 — Braking lacks marker/stop-zone input and physical capability

- **Subsystem:** Braking planning/actuator.
- **Evidence:** shared_core.py:131-144,381-441 requires brake capability and stop distance, but no real provider exists. phase_sim LAB assumes fixed reaction/deceleration/margin. kart/kart_sim/formula_student.py:120-135 evaluates EBS after an external trigger; it does not detect marker or actuate brake. No calibrated brake response/stopping distance found.
- **Consequence:** No verified brake trigger, stop position, stopping envelope or authority.
- **Minimal fix:** Implement marker/zone interpretation and uncertainty-aware planning in replay/SIL with explicit capability input. Characterize mechanical brake separately on isolated bench.
- **Verification:** Lost marker/pose, stale perception, brake unavailable/stuck and overshoot tests; physical stop/overshoot test before low-speed use.

### AK-P1-06 — Selected runtime/model/firmware ambiguous

- **Subsystem:** Configuration/release control.
- **Evidence:** CLI defaults to orange-only ConeSlayer (__main__.py:23,35-44); live dashboard defaults to ConeNet v1 (live_mission.py:528); YOLO26s optional. ESP32 variants have incompatible pins/step/stop assumptions. No selected installed firmware manifest. Dashboard user services checked 2026-10-16 inactive.
- **Consequence:** Test evidence may refer to different model, firmware, calibration or interlock than assumed runtime.
- **Minimal fix:** Record actual Pi executable/model/config/calibration and ESP32 firmware/protocol/wiring hashes; select one candidate per subsystem after review.
- **Verification:** Capture hashes after clean boot; match board identity/image/interface/wiring revisions.

## P2 — reliability, semantics or performance

### AK-P2-01 — Gabriel disconnected from gate selection

- **Evidence:** pipeline.py:202-206 computes Gabriel then passes Delaunay graph.edges to select_track_gates; gates.py:105-110 says connectivity only improves score; tests allow valid gates with empty edges.
- **Consequence:** Documentation can overstate Delaunay→Gabriel→gate filtering. It is not proven Gabriel should always constrain transverse gates.
- **Minimal fix:** Declare Gabriel diagnostic or acceptance constraint; change policy only after evidence.
- **Verification:** Junction, clutter, uncertain boundary and valid non-Gabriel crossing cases.

### AK-P2-02 — Simulator skidpad event does not enforce configured course

- **Evidence:** kart/configs/events/skidpad.yaml requests two left/two right laps; kart/kart_sim/formula_student.py:38,111-119 uses generic lap completion and ignores those fields. kart/configs/tracks/skidpad_event.yaml:7-20 is a single ring.
- **Consequence:** Simulation completion may not exercise intended two-lobe sequence.
- **Minimal fix:** Fixture must assert left/right sequence/counts or label simplified event.
- **Verification:** Reject one-lobe, wrong direction/count and skipped crossover.

### AK-P2-03 — Control calibration remains simulation-only

- **Evidence:** kart/kart_sim/autonomy.py:198-233 has wheelbase-aware Pure Pursuit and limits but default wheelbase is 1.0 m; real wheelbase, sign, angle and tire response unmeasured. ROS bridge is inert.
- **Consequence:** Formula/sign tests do not prove actual steering conversion.
- **Minimal fix:** Keep as simulation candidate; measure wheelbase, linkage ratio, wheel angle and sign before live adapter.
- **Verification:** Steering sweep/feedback and SIL/HIL against measured transfer function.

### AK-P2-04 — ROS schemas lack as-built sensor mapping

- **Evidence:** ConeArray/MotionCommand have useful fields; ActuatorFeedback includes brake position/pressure, torque, speed, current and temperature without mapping to installed sensors. Gateway topics lack active producers.
- **Consequence:** Defaults/stale values could be read as valid without data-source provenance.
- **Minimal fix:** Publish only measured fields; define clock, covariance, status, expiry and feedback requirements.
- **Verification:** Serialization round trip and missing/stale/malformed/range tests.

### AK-P2-05 — Planner implementations duplicated

- **Evidence:** 11 camera/team snapshots, duplicated phase_sim, and separate kart_sim, browser LAB, Phase 1 corridor and shared-core paths.
- **Consequence:** Tests/fixes can target inactive versions.
- **Minimal fix:** Record authoritative path/hash per runtime; keep candidates separate until promotion.
- **Verification:** Clean-copy replay/build and source/model/config hashes.

## P3

### AK-P3-01 — Browser bundle advisory

phase_sim build succeeded with 905.32 kB JS (247.31 kB gzip). Potential UI load cost, not a vehicle safety blocker. Split only if target-browser profiling justifies it.

## Blocking unknowns

Installed image; wiring and traction isolation; E-stop relay/contactor; spring-brake behavior; actual brake/speed sensors; Pi runtime; camera survey; physical 10 km/h enforcement; loaded mass, friction and stopping distance. These are UNKNOWN, not assumed passes/failures.
