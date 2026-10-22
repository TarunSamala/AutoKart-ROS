# AutoKart RC-03–05 implementation and file report

**Date:** 2026-10-16  
**Scope:** disarmed ROS 2 perception, mapping, planner and read-only diagnostics  
**Release verdict:** MORE DATA REQUIRED; vehicle NO-GO

This report describes the files changed or used for this run. The five
repository-wide audit reports under `docs/audits/2026-10-16/` remain the
inventory for the entire repository. The four RC-03–05 task documents on the
Desktop were treated as implementation specifications; no independent agent
review or physical vehicle validation is claimed.

## Starting point and source of truth

- `colcon list --base-paths src` discovers **16 ROS packages**, with no
  duplicate package names. The existing `autokart_planning` and
  `autokart_control` shells remain; the new adapters use distinct package
  names inside `autokart_nodes/`.
- `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/` is the
  authoritative Python perception and shared geometry library. ROS imports
  it; the adapter uses `autokart_perception_ros` as its distinct module name.
- `kart/kart_sim/autonomy/` remains the authoritative Pure Pursuit source.
  The `autokart_controller` adapter remains an empty shell in this run.
- `models/yolo26s/best.onnx` and `best.json` were left unchanged. SHA-256:
  `425a89cef2eda9db9c84fbf30cad187e510510b95d7b7e9904003fd22cec1158`
  and `c5659588cfb0f8c4f8b5c726ed596cae849634fcb866d7e00a344495b7fc6b57`.
  The model is about 37 MB and is experimental; no LIVE metric calibration
  was demonstrated.
- Firmware, camera SDK, Git metadata, and vehicle safety/actuation code were
  not moved or modified. No VCU command publisher was added.

## Files and what each contains

Paths in the tables are relative to the repository root. The exact checksum
for each changed source/model/config file is in
`docs/rc03_rc05_evidence/post_manifest.txt`; pre-edit checksums are in
`baseline_manifest.txt`. A path-preserving copy of every previously existing
file changed in this run is saved at
`/home/mark1/Desktop/AutoKart_RC03_RC05_rollback_2026-10-16/`.

| File or file group | Contents and purpose | Behavior/risk note |
|---|---|
| `kart/ros2_ws/src/autokart_nodes/README.md` | Names the four grouped ROS packages and the authoritative Python imports. | The grouping directory has no `package.xml`; it is not a package. |
| `autokart_nodes/autokart_perception/autokart_perception_ros/node.py` | RC-03 ROS transport for Phase 1 `CoreRuntime`, `Pipeline`, `Synthetic`, `Replay`, `HP60C` and YOLO26 ONNX. Emits image, delivery metadata, stage status and qualified `ConeArray`. | Defaults disabled. LIVE cone publication is withheld until calibration is verified. Missing live model/SDK/config fails closed. |
| `autokart_nodes/autokart_perception/scripts/perception_node` | Installed executable that calls the RC-03 node. | No actuator interface. |
| `autokart_nodes/autokart_perception/launch/perception_replay.launch.py` | Finite replay launch argument and node startup. | Exits after the recording; does not authorize recorded data automatically. |
| `autokart_nodes/autokart_perception/CMakeLists.txt`, `package.xml` | Install authoritative Phase 1 modules, ROS wrapper executable, launch file and adapter test; declare ROS dependencies. | Preserves ROS package name and Python import separation. |
| `autokart_nodes/autokart_perception/test/test_ros_adapter.py` | Source qualification, missing-model and import-shadowing regressions. | Test data only. |
| `autokart_nodes/autokart_mapping/autokart_mapping/node.py` | RC-04 pose-aware observed cone map using `CoreRuntime().algorithms` and `fit_skidpad_topology`. Pairs status/cones with historical odometry and publishes observed `odom` cones, topology and reasons. | Clears the map on invalid input, duplicate cones, odometry jumps or timeouts. LIVE requires explicit odometry verification and covariance. |
| `autokart_nodes/autokart_mapping/scripts/mapping_node` | Installed mapping executable. | Publishes diagnostic map/status only. |
| `autokart_nodes/autokart_mapping/CMakeLists.txt`, `package.xml` | Mapping module installation, dependencies and test registration. | Package name is unique. |
| `autokart_nodes/autokart_mapping/test/test_mapping_adapter.py` | Pairing, reset, duplicate ID and frame rejection tests. | Simulated input only. |
| `autokart_nodes/autokart_planner/autokart_planner/mission.py` | RC-05 read-only mission engine: observed-ring skidpad progression, gated crossover, braking horizon, swept centerline obstacle test and hold reasons. | Uses no surveyed track truth or command output. Simulated brake capability is never evidence of physical braking. |
| `autokart_nodes/autokart_planner/autokart_planner/node.py` | ROS pairing of map/status/odom/obstacles/feedback and publication of expiring diagnostic `nav_msgs/Path` and stage status. | No `MotionCommand` publisher. Invalid input emits empty path and HOLD. |
| `autokart_nodes/autokart_planner/scripts/planner_node` | Installed planner executable. | Diagnostic only. |
| `autokart_nodes/autokart_planner/CMakeLists.txt`, `package.xml` | Planner module installation, dependencies and test registration. | Package name is unique. |
| `autokart_nodes/autokart_planner/test/test_mission_adapter.py` | Four-lap, wrong-direction, incomplete ring, swapped color, brake and swept-obstacle tests. | Selected SIL cases, not a complete adversarial matrix. |
| `autokart_nodes/test/ros_straight_pipeline_probe.py` | Launches the actual RC-03–05 ROS graph, injects explicitly simulated odometry and optional obstacle, checks gateway HTTP frames and fail-closed planning. | Isolated ROS domain; no hardware or command topic. |
| `autokart_nodes/test/ros_skidpad_route_probe.py` | Injects a qualified simulated cone-ring fixture plus 164 odometry samples into actual mapping/planner nodes; checks 2R/2L, transition and feedback-gated STOP. | Does not exercise camera-derived cones or measured braking. Child process groups are cleaned up. |
| `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/shared_core.py` | Small correction to observed skidpad topology fitting: ignores the crossover ambiguity band and admits the intended angular sample gap. | Existing Delaunay/Gabriel, map and Pure Pursuit algorithms were not rewritten. Original SHA-verified copy is in the rollback snapshot. |
| `autonomy_stack_phase1/phase_01_perception/tests/test_shared_core.py` | Adds regression for the observed two-ring/crossover topology. | Existing Phase 1 suite still passes. |
| `kart/ros2_ws/src/autokart_bringup/launch/rc03_rc05_diagnostics.launch.py` | Disarmed composed launch of the three adapters and optional read-only dashboard gateway. Requires external odometry; simulated brake parameters are explicit arguments. | Existing `full_stack.launch.py` is unchanged. |
| `kart/ros2_ws/src/autokart_bringup/package.xml` | Adds adapter dependencies and an `ament_cmake` build-type export. | Repair makes `ros2 launch autokart_bringup ...` discoverable from a fresh overlay; its pre-edit copy is backed up. |
| `kart/ros2_ws/src/autokart_dashboard_gateway/autokart_dashboard_gateway/core.py` | Adds three diagnostic stages, tracked observed cones and frame metadata to stored Mission Control frames. | Ground truth remains separate from observed data. |
| `kart/ros2_ws/src/autokart_dashboard_gateway/autokart_dashboard_gateway/gateway.py` | Subscribes to simulated odometry and mapped cones; refuses to plot `base_link` detections as world coordinates. | Read-only gateway. |
| `kart/ros2_ws/src/autokart_dashboard_gateway/test/test_core.py` | Checks diagnostics, observed map and absent target speed in display data. | Gateway contract only; no browser interaction. |
| `phase_sim/src/mission/schema.ts` | Adds typed diagnostic stage payload to the Mission Control frame schema. | Display data only. |
| `phase_sim/src/mission/MissionControl.tsx` | Renders source, validity, frame, sample time and reason-code strip. | No UI-side autonomy calculation or command path. |
| `kart/tests/integration/test_ros2_workspace.py` | Updates launch inventory assertion for the added diagnostics launch. | Full kart pytest stayed at 150 passing tests. |
| `docs/NODE_CONTRACTS.md` | Frozen RC-03–05 topics, frame/time rules, stage validity and disarm contract. | Contract document, not measured vehicle evidence. |
| `docs/INTEGRATION_DECISIONS.md` | Package/source ownership, qualification choices, staged migration and rollback notes. | RC-02 history is retained. |
| `docs/DASHBOARD_DATA_CONTRACT.md` | Diagnostic frame and display provenance fields. | Ground truth cannot be used as live observations. |
| `docs/TEST_MATRIX.md` | Untouched baseline, final software results and remaining test gaps. | Historical RC-02 results are labeled separately. |
| `docs/RELEASE_REPORT.md` | Current status and vehicle NO-GO verdict. | No independent validator or physical readiness claim. |

## Runtime data flow

The same flow is available as `docs/RC03_RC05_DATA_FLOW.svg`.

```text
Synthetic / provenance-qualified replay / guarded LIVE source
    -> RC-03 /perception/cones + perception status + RGB/depth/metadata
    -> RC-04 observed cone map in odom + geometry/status
    -> RC-05 expiring path candidate + mission/HOLD status
    -> read-only gateway -> existing Mission Control UI

Externally supplied odometry -> RC-04 and RC-05
Externally supplied brake capability and optional obstacle -> RC-05
No edge in this graph publishes a VCU MotionCommand.
```

All stages carry `source`, `frame_id`, sample time, validity, reason codes and
sequence metadata. The output status explicitly includes
`vehicle_authorized=false` and `published_to_vcu=false`.

## Replay frame and image notes

The tested `recordings/final-smoke-20261008/` directory contains exactly two
frame pairs. `000001.json` and `000002.json` each contain timestamp, alignment,
intrinsics, raw depth scale, six preannotated blue/yellow cone observations,
and empty `calibration`; both state `source=SYNTHETIC` and `model_version=none`.
`000001.npz` and `000002.npz` each contain one 640×480×3 `uint8` RGB array and
one 640×480 `uint16` depth array. The two RGB arrays are identical; the two
depth arrays are identical and contain constant raw value 4000 (the metadata
declares scale 0.001). They are synthetic test frames, not surveyed camera
images or proof of measured depth. Because replay provenance is incomplete,
the RC-03 launch marked both frames degraded and published no qualified metric
cones. The `.json` and `.npz` inputs were read only and were not modified.

## Verification performed

| Check | Untouched baseline | Final result |
|---|---:|---:|
| Phase 1 pytest | 139 passed | 140 passed |
| Kart pytest | 150 passed | 150 passed |
| Existing ROS/gateway contract pytest | 10 passed | 11 passed |
| LAB Vitest | 32 passed | 32 passed |
| LAB frontend build | pass | pass |
| Firmware static pytest | 9 passed | 9 passed |
| Nested WebStop static | pass | pass |
| Focused new adapter pytest | absent | 11 passed |
| Fresh ROS colcon build | prior 16-package RC-02 record | 16 packages built |
| Registered colcon tests | prior RC-02 suite | 18 entries, 0 errors/failures/skips |
| Replay launch | not in baseline | 2 frames, clean exit, degraded/withheld |
| Straight skidpad probe | not in baseline | incomplete loops held, no valid planner path |
| Braking obstacle probe | not in baseline | `CORRIDOR_BLOCKED`, no valid planner path |
| Full skidpad fixture | not in baseline | 164 samples, 2R/2L, feedback-gated STOP |

Evidence logs are under `docs/rc03_rc05_evidence/`. The initial baseline
colcon build overlapped source edits and is **not** counted as an untouched
build comparison. The final fresh build is `verified_full_build.log`, with
`verified_full_test.log` and `verified_full_test_result.log`. Earlier interim
failures and repairs are retained in the evidence directory; final results
above are from the corrected source and install space.

## Open work and rollback

Physical release remains blocked by unverified HP60C metric calibration and
Pi runtime, no qualified real odometry/TF, no measured brake capability,
incomplete fault/reconnect matrix, no independent stop-chain evidence, and no
vehicle trial. The pre-existing `autokart_msgs` package still classifies as
`ros.catkin` under `colcon list`; generated Python message imports worked in
these tests, but its packaging should be reviewed separately before broader
ROS deployment. The existing `autokart_controller` shell was not implemented.

Rollback instructions and all pre-edit changed-file copies are at
`/home/mark1/Desktop/AutoKart_RC03_RC05_rollback_2026-10-16/ROLLBACK.md`.
The project root has no Git repository, so that snapshot is necessary to
restore pre-edit Phase 1 source. No files were permanently deleted.
