# Gazebo Follower Acceptance Report

```text
TASK:
Continuous Gazebo visual and sensor follower acceptance

STATUS:
COMPLETE

VERDICT:
APPROVED_WITH_CONDITIONS
```

## Baseline

- Regression tests: `125 passed in 47.23s` before modification.
- Existing Gazebo capability: static spawn/stability and sensor interface
  advertisement; no continuous pose/joints or rendered ROS payload evidence.

## Physics authority

- Authoritative backend: internal deterministic dynamic four-wheel plant.
- Gazebo role: static `visual_sensor_follower`.
- Double-physics prevention: configuration/SDF/runtime rejection, no force or
  controller path, visual-only contacts and four-rate trajectory comparison.

## Implemented

- Authoritative snapshot: frozen, finite, monotonic, configuration-hashed and
  reset-versioned.
- Pose follower: continuous world pose and twist for sensor consistency.
- Joint follower: measured left/right steering, front wheels and common rear
  axle rotation/speed.
- Clock synchronization: sole internal-authority `/clock`, interpolated 1 ms
  Gazebo steps and 2 ms runtime threshold.
- TF: sole dynamic `odom -> base_link` plus static robot/sensor tree.
- RGB: native `320x240 rgb8`, non-empty and pose-dependent.
- Depth: native `320x240 32FC1`, finite valid regions and changing payload.
- LiDAR: native 1080-sample scan with changing cone/wall returns.
- IMU: sole internal source at 200 Hz with established bias/noise/timing layer.
- Sensor queues: bounded, epoch-flushed, latency/drop/queue counters.
- Pause: native world, clock and sensors freeze.
- Reset: new epoch, explicit rewind and queue flush; no autonomy source exists
  inside the follower.
- Failure handling: bounded bridge-delay rejection, pause/disconnect watchdog
  and main-run full-coverage rejection on Gazebo loss.
- Recording: checksummed logs, metadata, representative samples and 12 plots.

## Full-lap demonstration

- Track: `configs/tracks/research_circle.yaml`.
- Controller: Pure Pursuit high-level request through VCU and independent
  safety.
- Lap completed: yes.
- Lap time: 12.260 s.
- Final controlled stop: yes, at 16.560 s.
- Final VCU state: `SAFE`.

## Follower error

- Maximum position error: 0 m.
- RMS position error: 0 m.
- Maximum orientation error: 0 rad.
- Maximum steering-joint error: 0 rad.
- Maximum wheel-rotation error: 0 rad.
- Maximum clock offset: 0.001 s.

## Sensor results

- RGB frames captured/delivered/dropped: 496 / 471 / 25.
- Depth frames captured/delivered/dropped: 496 / 470 / 26.
- LiDAR scans captured/delivered/dropped: 330 / 329 / 1.
- IMU samples captured/delivered/dropped: 3312 / 3312 / 0.
- Maximum latency: 0.021 s.
- Timestamp violations: 0.
- Empty payloads: 0 for every sensor.

Drops are explicit stale-render warm-up rejection, not hidden empty messages.
Maximum queue depth was one.

## No-double-physics results

- Follower disabled: lap complete, SHA-256 `28164b960133…349a2`.
- Low update rate: 20 Hz, identical.
- Normal update rate: 100 Hz, identical.
- High update rate: 200 Hz, identical.
- Maximum authoritative trajectory difference: 0 m.
- Invariant violations: 0.

DP2 conflicting dynamics was rejected. DP3 Gazebo contact remained visual-only.

## Test results

- Unit: follower suite 13 passed.
- Integration: continuous pose/joints/sensors/clock plus native sync probe PASS.
- Full lap: PASS.
- Safety: VCU/independent safety retained; pause/reset/disconnect/process-loss
  PASS.
- Regression: 138 passed in 61.92 s, including all 13 follower tests.
- ROS packages: 6 tests, 0 failures via `colcon test`.

## Performance

- Real-time factor: 0.197 with software rendering.
- CPU: approximately 103% for the timed ROS recorder process.
- GPU: `NOT_EXECUTED_EXTERNAL_DEPENDENCY` (`nvidia-smi` unavailable).
- Memory: 176.7 MiB maximum RSS for the timed recorder process.
- ROS queue depth: maximum native sensor delivery queue depth one.

## Files created

- `kart_sim/gazebo_follower.py`, `kart_sim/gazebo_full_lap.py`.
- Native follower/sensor plugins and three ROS messages.
- Follower world, scenario and acceptance configuration.
- Five follower run/test/report scripts plus sync and process-loss probes.
- `tests/gazebo_follower/` and ROS package-local contract tests.
- Six follower design/evidence documents and this report.
- `artifacts/gazebo_follower/` evidence tree.

## Files modified

- Fast environment observer/controlled-stop API.
- Gazebo model, bridge clock config, ROS package build metadata and runner.
- Demonstration, user, test, status, change-manifest and platform reports.
- One robot-world-count regression assertion, strengthened from six to seven.

## Known limitations

- Simulation thresholds are not real-kart limits.
- Camera/depth point cloud and CameraInfo calibration remain incomplete.
- RViz/Gazebo GUI screenshots and GPU metrics were not executed.
- Full rosbag is opt-in and not committed; samples plus complete metadata are
  retained.
- The evidence recorder observed 461 of 471 published RGB frames under
  llvmpipe; all 470 depth and 329 LiDAR deliveries were observed. Plugin
  delivery counters and checksummed representative payloads are retained.
- Gazebo contacts do not affect the internal plant in follower mode.

## Real-kart readiness

**REJECTED.**

## Next engineering task

Implement and natively validate calibrated CameraInfo plus degraded RGB-D
point-cloud delivery against measured sensor calibration.
