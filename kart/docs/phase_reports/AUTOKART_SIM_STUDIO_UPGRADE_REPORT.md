# AutoKart Sim Studio Upgrade Report

```text
TASK:
FSDS-, F1TENTH-, and Gazebo-class simulator upgrade

STATUS:
PARTIAL

VERDICT:
PROTOTYPE_TEST_REQUIRED
```

## Previous platform

- Deterministic planar and four-wheel plants with actuator, electrical and
  thermal models.
- VCU validation, independent safety, E-stop, controlled stop, fault campaigns,
  telemetry, replay and web dashboard.
- Pre-follower acceptance regression baseline: 125 passed.

No existing plant, safety, fault, dashboard, scenario or test subsystem was
discarded.

## Implemented

- **Fast racing mode:** deterministic Gym-style single/vector-agent API,
  LiDAR, progress/limits, VCU-routed actions and baseline controllers.
- **Engineering mode:** existing dynamic four-wheel plant and complete
  actuator/VCU/independent-safety path retained.
- **Gazebo mode:** native Classic 11 static visual/sensor follower, immutable
  reset-versioned state, interpolated pose/joints, sole clock/TF/joint source,
  rendered RGB/depth/LiDAR and internal IMU delivery.
- **Formula Student mode:** acceleration, skidpad, autocross, trackdrive and
  EBS schemas/evaluators and provisional event tracks.
- **SIL/HIL mode:** safe presets and ROS/CAN boundaries; real output remains
  disabled.
- **Robot description:** required Xacro/URDF/SDF links, steering/wheels,
  common solid rear axle, primitive inertial/visual/collision geometry and
  provisional sensors.
- **Track/world system:** one YAML source with internal, SDF, dashboard, map,
  raceline and cone conversions.
- **Sensors:** timestamped delivery contracts, bounded queues, capture and
  delivery times, reset flush, noise/fault hooks.
- **ROS 2:** 12-package Humble workspace, domain messages, native follower and
  package-local tests.
- **TF:** one `map -> odom -> base_link` tree with wheels and optical frames.
- **Autonomy:** Pure Pursuit full native lap plus Stanley, PID, gap and
  cone-midpoint baselines.
- **Multi-kart/race control/dashboard/RViz/editors:** prior framework retained.
- **Fault integration:** prior detailed campaigns retained across the VCU and
  independent-safety path.

## Physics authority

- Nominal Gazebo configuration is internal authority plus
  `visual_sensor_follower`.
- Gazebo model is static, has no force/drive/controller plugin, and consumes
  only authoritative snapshots.
- Measured steering and wheel/rear-axle rotations drive visuals.
- Runtime checks reject a second dynamics authority and freeze/diagnose stale
  or disconnected following.
- DP1 produced byte-identical authoritative logs with follower off and at
  20/100/200 Hz. DP2 rejected conflicting dynamics. DP3 left contact
  visual-only.

## Test results

- **Unit/integration/regression:** baseline 125 retained; follower-focused 13
  added; final full suite 138 passed in 61.92 s.
- **Robot description/SDF:** native model and follower-world validation PASS.
- **Gazebo:** continuous full lap, pose/joints, clocks and payloads PASS.
- **ROS:** package build PASS; `colcon test` 6 tests, 0 failures.
- **Autonomy:** one 12.260 s closed lap and controlled stop PASS.
- **Safety:** final VCU `SAFE`; pause/reset/disconnect/process-loss and
  no-bypass contracts PASS.
- **Multi-agent:** existing namespace/vector/race smoke remains PASS.

## Performance

- **Tier 0–2:** existing measurements remain recorded in
  `AUTOKART_SIM_STUDIO_PERFORMANCE.yaml`.
- **Tier 3:** native software-render lap RTF 0.197; recorder process about 103%
  CPU and 176.7 MiB maximum RSS; follower queue depth one.
- **Tier 4:** NOT_EXECUTED_EXTERNAL_DEPENDENCY; Chrono bindings unavailable.

## Native integrations executed

- ROS 2 Humble message generation, package build and six package-local tests.
- Gazebo Classic 11 SDF validation, continuous pose/joints and lockstep clock.
- Native RGB/depth/LiDAR payload delivery and internal IMU ROS delivery.
- Native pause/resume/reset/disconnect and Gazebo-process-loss rejection.
- Complete autonomous lap and four-rate no-double-physics campaign.

## Not executed

- RViz/Gazebo GUI screenshot regression.
- Calibrated CameraInfo, device-like depth point cloud and alignment tests.
- Native Gazebo multi-kart/TF isolation.
- Chrono, Jetson, physical VCU/CAN, actuators and real safety hardware.

## Known limitations

- Robot/sensor/track parameters remain simulation-only until measured.
- Tier-3 camera/depth are 320x240 to satisfy state-age limits under llvmpipe.
- Twenty-five RGB, 26 depth and one LiDAR warm-up frames were deliberately
  rejected as stale and counted.
- Gazebo contact is visual-only in this authority mode.
- Full raw rosbag and GPU metrics were not retained; checksummed samples and
  complete metadata were retained.
- The scenario/track editors remain schema/CLI tools rather than full GUI
  editors.

## Real-kart readiness

**REJECTED unless separately proven.**

## Next engineering task

Implement and natively validate calibrated CameraInfo plus degraded RGB-D
point-cloud delivery against measured sensor calibration.
