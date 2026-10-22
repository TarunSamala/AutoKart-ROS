# End-to-end architecture and interface audit

Audit date: 2026-10-16. Source existence is distinguished from executed tests and demonstrated end-to-end behavior.

## Current data flow

The real camera diagnostic reaches an immutable dashboard result, but stops before trusted metric cones and has no live pose. Shared core can turn cones plus pose into map, mission, path, speed and controller candidates in simulation/replay; it withholds vehicle authority. ROS schemas and a VCU-named subscriber exist, but no live ROS chain connects camera/planner or command/ESP32. E-stop behavior is not independently proven.

HP60C → paired RGB/depth → detector → depth/mount candidate → local Delaunay/Gabriel/boundary/gates/centerline → dashboard/recording.

Qualified cones + fresh odometry → shared map → skidpad/braking candidate → PP/speed candidate → vehicle_authorized=false. Missing: qualified live pose, ROS integration, command transport and independent stop evidence.

## Stage trace

| Stage | Source, contract and units | Cadence/freshness/failure | Integration |
|---|---|---|---|
| Camera | autonomy_stack_phase1/phase_01_perception/src/autokart_perception/camera.py and native/hp60c_bridge.cpp. Paired RGB/RAW16/intrinsics/sequence and monotonic callback timestamp. | Saved captures measured 12–13 Hz callback, 4–5 Hz processing; mailbox drops backlog. Exposure timestamp not verified. | Acquisition demonstrated in saved bench data; not connected to controls. |
| Detector | detector.py, live_mission.py, __main__.py. 640×480 RGB to boxes/class/score/model hash. | ONNX per frame; live wrapper 250 ms freshness limit; error/fault yields no trusted result. | Different defaults: CLI orange-only ConeSlayer, dashboard ConeNet v1, optional YOLO26s. Freshness and side-color classification fail qualification. |
| RGB/depth association | depth.py, camera.py, C++ bridge. Detection boxes plus registered depth/intrinsics to depth candidates. | Sequence/timestamp/support/range/freshness rejection. | Sequence parity measured; pixel alignment, raw scale and exposure-to-result age unverified. |
| Camera transform | mount.py, depth.py and YAML. Optical point to candidate ground-contact XY in base_link, X-forward/Y-left, metres, uncertainty/reasons. | Static mount; clipped foot/poor depth rejected. | Mount/depth scale remain unverified; surveyed calibration absent. |
| Cone observations | types.py and pipeline.py. IDs, labels, XY, uncertainty, timestamp/status. | Current frame only; temporal filter is display-only. Fault/stale results suppress path and authority. | No ROS ConeArray publisher; latest capture had 0 validated cones. |
| Geometry | geometry.py, gabriel.py, boundaries.py, gates.py, centerline.py, curvature.py. | Per-frame finite/duplicate/degenerate and width/topology checks. Local forward-X ordering only. | Software tests pass; live valid points absent. Gabriel computed but gate selection receives Delaunay edges. |
| Localization/map | shared_core.py:111-201: pose in odom, ns stamp, position/yaw/speed/uncertainty; 256-entry map, 10 s expiry; 150 ms pose age. | Map refuses update without fresh pose. | No connected encoder/IMU/steering source; core_main passes None; live wrapper provides pose only in SIM. |
| Skidpad | shared_core.py:32-109,205-248,342-379 observed-ring fit/winding; separate LAB and simulator algorithms. | Requires observed topology and fresh pose; invalid evidence withholds path. | Candidate only; no measured full-loop/lap evidence. |
| Braking/speed | shared_core.py:131-144,381-441 explicit capability/stop distance; phase_sim assumes fixed deceleration; kart simulator evaluates models. | Capability/pose absence withholds validity. | No marker-to-stop-zone connection or measured brake response/stopping distance. |
| Steering control | kart/kart_sim/autonomy.py:198-233 wheelbase-aware PP; Stanley also present. Sim state/trajectory to sim steering command. | dt, angle/rate limits and command validity. Vehicle signs/limits/wheelbase unmeasured. | Software candidate, no ROS controller executable or live pose. |
| ROS/VCU | kart/ros2_ws/src/autokart_msgs; full_stack.launch.py; autokart_sim_bridge/vcu_bridge.py:14-43. MotionCommand carries time/expiry/source/sequence/SI values/limits/validity. | vcu_bridge queue depth 1, stores last object; no validation, expiry, arbitration, serialization or forwarding. | Disconnected. Full-stack launch starts only inert subscriber; no Pi USB Agent or ESP32 session. |
| ESP32/actuator | AutoKart_RC_Dynamics_ESP32S3_v1/main/main.c; nested WebStop is a separate HTTP firmware. | Root candidate has 300 ms timeout/sequence/token; candidate assumptions differ across firmware. | No autonomy subscriber; idf.py absent; installed image and measured outputs unknown. |
| E-stop | Root firmware samples auxiliary input; LoRa prototype has placeholders. | Software response not independent of MCU/network. | Hardwired traction disable and power-loss brake unknown. |

## Contract findings

- Perception uses base_link X-forward/Y-left in metres; shared core poses/map use odom. No connected TF publisher or frame transform is present.
- ConeArray and MotionCommand have useful timestamps, frame, sequence, expiry and SI fields. ActuatorFeedback schema exceeds the verified installed sensor set. No live producer/consumer checks frame, covariance, clock origin, age or sequence.
- kart_sim.ros2_wire is serialization helper code, not live transport. Dashboard gateway is read-only.
- Control/safety bridge CMake packages install metadata/config only. Topic declarations are not nodes.
- phase_sim SIM/REAL expects external ROS data; LAB/DEMO are generated. Shared-core dashboard is diagnostic and unauthorized.

## Minimum missing connections

1. Survey/qualify cone metric scale, extrinsics, registration, color, uncertainty and Pi timing.
2. Connect calibrated encoder/wheel speed and IMU odometry with explicit frame/time/covariance contract.
3. Feed qualified data to one shared core and validate skidpad/braking in replay/SIL.
4. Publish observations/diagnostics read-only, then test controller/expiry via mock VCU.
5. Identify one ESP32 image/protocol and verify wiring, feedback, independent stop and brake on isolated bench before vehicle operation.
