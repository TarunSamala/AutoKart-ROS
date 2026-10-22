# AutoKart release status — RC-05 skidpad / RC-07 dashboard, 2026-10-16

## 2026-10-17 VCU ROS Stage 1 addendum

The existing 16-package ROS workspace built in isolated baseline and final
overlays. A read-only `autokart_diagnostics` executable now projects the
NOOP-only micro-ROS bench status array into a typed
`autokart_msgs/BenchVcuStatus` topic. Nine package manifests were corrected
from accidental Catkin classification so all 16 packages are discoverable in
the sourced ROS 2 overlay. This stage did not run the ROS node against a Pi or
ESP32, send a command, build/flash firmware, or operate hardware. It does not
provide production VCU feedback or vehicle authority. **Vehicle release
remains NO-GO.** See the [VCU ROS Stage 1 report](VCU_ROS_STAGE1.md) and
[build/discovery evidence](vcu_ros_evidence/discovery.log).

**Vehicle release: NO-GO.** `vehicle_authorized=false`; this run published no vehicle command, did not flash firmware, and did not operate actuators. The primary dashboard defaults to REAL/LIVE. An attached HP60C (`3482:6723`) was subsequently verified through the existing SDK and ROS perception publisher: the dashboard displayed actual paired RGB/depth frames and LIVE ROS perception diagnostics. No qualified metric depth, real odometry, mapping, planner route or VCU feedback was available.

The skidpad LAB route anomaly had two confirmed causes: a side-of-loop Gabriel gate was admitted as an EXIT gate (first EXIT target x=2.827 m), and the old completed frame reused a stale lookahead with an empty selected path and 2.6 m/s request. The corrected EXIT target is an observed central gate at x=0, and terminal STOP has zero speed and no lookahead. A nominal 1.6 × 1.2 m **proxy** footprint avoids cone centers in sampled LAB replay; the proxy is not a surveyed kart envelope. The synthetic ROS route probe reached two right laps, two left laps, EXIT, feedback-pending hold and STOP. The ROS planner still publishes diagnostic candidates only.

**Open acceptance failure:** the LAB case with 15% cone dropout did not complete and collided in simulation. The ROS planner holds when observed topology or fresh pose is missing, but this does not validate the LAB fallback behavior. The real HP60C stream was diagnostic only: YOLO boxes were displayed from actual RGB frames, but all 20 samples in the later cone-view trial had unqualified depth/metric status. Mean capture-to-gateway delay was 512 ms at 2.2 fps under that load; boxes and class labels have no measured accuracy. Curvature and steering-rate limits have no as-built qualification. The release verdict stays NO-GO.

Validation: [detailed report](RC05_RC07_VALIDATION_REPORT.md), [live HP60C boxes and dashboard](skidpad_live_evidence/dashboard_real_hp60c_boxes_final.png), [final live timing/API samples](skidpad_live_evidence/hp60c_live_boxes_final_api_probe.json), [route coordinates](skidpad_live_evidence/lab_before_after_coordinates.json), [full ROS route probe](skidpad_live_evidence/ros_full_route_probe_final.log), [LAB tests](skidpad_live_evidence/phase_sim_tests_clock_final.log), [ROS Python tests](skidpad_live_evidence/ros_python_tests_final.log), [clock regression](skidpad_live_evidence/ros_perception_tests_clock_final.log), [Phase 1 tests](skidpad_live_evidence/perception_tests_final.log), [kart tests](skidpad_live_evidence/kart_tests_final.log), and [frontend build](skidpad_live_evidence/phase_sim_build_clock_final.log). The combined focused ROS Python suite passed 32; the frontend passed 35, Phase 1 passed 140, and kart passed 150. The changed ROS packages built in temporary overlays. Prior RC-03–05 evidence remains below.

---

# Integration status (RC-02)

## RC-03–05 release review, 2026-10-16

**Verdict: MORE DATA REQUIRED; vehicle NO-GO.** The disarmed software adapters
are runnable and selected synthetic/replay paths passed. They are not an
approved live autonomy system. `vehicle_authorized=false` and
`published_to_vcu=false` remain explicit; no `MotionCommand` publisher or
hardware operation was introduced. One engineer performed this implementation
and review; no independent validator claim is made.

| Item | Status | Evidence | Next fix |
|---|---|---|---|
| RC-03 ROS executable | PASS, software/replay scope | `replay_launch.log` exited 0; `ros2 pkg executables`; focused tests | Survey RAW16 scale, registration, extrinsics and Pi timing before LIVE metric publication. |
| RC-04 ROS executable | PASS, simulated map scope | `ros_probe_result.log`, `ros_skidpad_result.log`, mapping tests | Connect qualified real odometry/TF and test drift, dropout, outliers and loop closure. |
| RC-05 ROS executable | PASS, selected SIL fixtures | `ros_skidpad_result.log` (2R/2L/STOP), `ros_braking_result.log` (`CORRIDOR_BLOCKED`, no valid path) | Complete adversarial mission/fault matrix, measured brake envelope and independent route review. |
| Integrated read-only ROS replay/SIL | PARTIAL PASS | Synthetic cone/odom/gateway probe and full-loop cone fixture; no command output | Replay qualified camera-derived metric cones and real odometry when available. |
| Existing dashboard diagnostic view | PASS, API/build scope | Gateway HTTP probe, 11 ROS contract tests, LAB build/32 tests | Browser session inspection with stale/reconnect and actual Pi telemetry. |
| Pi live calibration/performance | NOT RUN | No surveyed camera or current Pi runtime in this run | M1 survey and target CPU/freshness measurement. |
| Hardware/VCU/independent stop | NO-GO / NOT RUN | No flash, serial, wheel/actuator or traction test | Wiring review, isolated bench tests and independent stop evidence. |

Exact source and package hashes are in `docs/rc03_rc05_evidence/post_manifest.txt`;
the pre-edit hash record is `baseline_manifest.txt`. The root project has no
Git repository. Phase 1 `src/` was already untracked inside its nested Git
checkout, so Git cannot recover the original version; a SHA-verified pre-edit
copy of `shared_core.py` is saved at
`/home/mark1/Desktop/AutoKart_RC03_RC05_rollback_2026-10-16/shared_core.py`.
No `.git` directory, firmware or model file was moved or modified.
The final fresh ROS build classified `autokart_bringup` as `ros.ament_cmake`
after its package metadata was repaired, and the 18 registered colcon test
entries passed with no failures or skips. The pre-edit metadata is in the
rollback snapshot.

Remaining defects include no live qualified metric cones, no real odometry
estimator, no measured braking capability, incomplete G3 fault coverage, no
target Pi time/memory profile, and no physical stop-chain proof. The highest
risk next task is the M1 calibration and as-built sensor baseline from
`05_COMPLETION_PLAN.md`; it must precede any LIVE mapping/path approval.

## Historical RC-02 decision

The remainder of this document records the earlier package-scaffold review.
The current RC-03–05 verdict and executable evidence are above.

**MORE DATA REQUIRED / VEHICLE NO-GO.** The unmodified software baseline passed as recorded in `TEST_MATRIX.md`. The package grouping and initial contracts are being established; no executable ROS autonomy nodes or end-to-end live pipeline have been delivered in this stage.

## Completed

- Enumerated 13 existing ROS packages; all names are unique.
- Confirmed the Python authoritative algorithm paths and searched ROS package/launch references before relocating the empty `autokart_perception` ROS shell.
- Created `autokart_nodes/` as a grouping directory with no root package manifest; established the four child ROS package paths.
- Preserved the Phase 1 Python package and existing planning/control compatibility shells.
- Installed the authoritative Phase 1 Python modules through the ROS package's CMake install rule and verified source/install hashes for three representative files.
- Ran the untouched software baseline before source edits.
- Re-ran the regression suite after changes: 139 perception, 150 kart (including the new package-discovery test), 32 LAB, 10 ROS contract and 9 firmware static checks passed; the ROS workspace built 16 packages.

## Not demonstrated

No executable ROS autonomy node or end-to-end live pipeline has been tested in this stage. No Pi runtime, HP60C calibrated metric detections, IMU/wheel odometry, installed VCU identity, real actuator feedback, independent E-stop behavior, measured brake capability, ROS command/feedback round trip, or physical skidpad/braking performance has been tested. No hardware was operated; no firmware was built or flashed.

Keep `vehicle_authorized=false`, command publishers absent/disabled, and all physical actuator connections isolated. Do not report production readiness or vehicle operation approval.
