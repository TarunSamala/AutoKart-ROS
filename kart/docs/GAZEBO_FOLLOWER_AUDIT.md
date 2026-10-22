# Gazebo Follower Authority and Publisher Audit

Audit date: 2026-07-29

Baseline command:

```bash
python3 -m pytest -q
```

Baseline result: `125 passed in 47.23s`.

## Existing authoritative path

The deterministic scheduler in `kart_sim/scheduler.py` invokes the scenario
plant at the configured fixed step. `kart_sim/scenario.py` routes each
high-level request through `VCU.cycle()`, applies the independent-safety
override every physics tick, advances the actuator models, and only then calls
the internal vehicle model. The resulting `PlanarKartState` and four-wheel
telemetry are the existing authoritative state.

The current plant does not publish an immutable, reset-versioned vehicle
snapshot. Scenario callbacks receive a mutable JSON-compatible telemetry
record after each step. Wheel angular speeds and road-wheel steering angles
exist, but accumulated wheel rotations do not.

## Existing pose path

- Internal pose authority: `PlanarKartState` in `kart_sim/types.py`.
- Integrator: `FourWheelKart.step()` in `kart_sim/four_wheel.py`.
- Existing visual adapters: the transport-neutral Unreal/CARLA adapter and the
  contract-only `GazeboVisualFollower`.
- Gazebo model: `simulation/models/autokart/model.sdf`.
- Runtime internal-to-Gazebo pose publisher: **none**.
- Gazebo model pose controller/plugin: **none**.
- Dashboard pose source: scenario telemetry JSONL, read-only.

The Gazebo model is currently `<static>true</static>`. Its pose does not move
after spawn. Static status prevents gravity/contact integration from becoming
a second vehicle solution, but static spawn is not a continuous follower.

## Existing joint path

The internal four-wheel model exposes:

- measured left/right road-wheel steering angles;
- four wheel angular speeds;
- common rear-axle angular speed.

It does not persist accumulated wheel or axle rotation. No Gazebo joint
adapter exists, and no ROS `/joint_states` publisher is implemented.

The SDF contains left/right steering joints, two front wheel joints, and a
common rear axle joint. Rear wheels are fixed to the common axle, matching the
solid-axle visual relationship. No joint controller or effort plugin is
loaded.

## Existing clock path

`SimulationClock` in `kart_sim/ros2_sil.py` is the transport-neutral internal
clock contract with pause and reset epochs. It is not wired to the native
Gazebo launch.

`ros2_ws/src/autokart_gazebo/config/bridge.yaml` maps Gazebo `/clock` to ROS.
That direction conflicts with the intended internal-time authority if enabled
with the follower. The launch file starts Gazebo without the state bridge.

Current native clock authority: undefined for an integrated run.

## Existing sensor path

The SDF declares RGB, depth, ray LiDAR, and IMU sensors. Native smoke testing
previously proved that the Gazebo transport topics exist, but it did not prove
non-empty rendered payload capture or timestamp-preserving ROS delivery.

The ROS bridge YAML lists LiDAR and IMU mappings only. It has no RGB/depth
mapping, no delivery metadata, no reset epoch, and no processing/delivery
latency record. The installed ROS bridge is for modern Gazebo transport while
the repository currently launches Gazebo Classic 11, so this YAML is not
evidence of a working Classic sensor bridge.

## TF publisher audit

Repository-wide source inspection found no runtime:

- `map -> odom` publisher;
- `odom -> base_link` publisher;
- `/joint_states` publisher;
- native dynamic TF broadcaster.

`build_frame_tree()` is a validation contract, not a native publisher.
`robot_state_publisher` is declared as a description-package dependency but is
not launched. There are therefore no duplicate runtime publishers today, but
there is also no complete frame tree.

The follower implementation must be the sole dynamic publisher of
`odom -> base_link` and `/joint_states`. Static sensor transforms may be
published by one `robot_state_publisher` instance only.

## Existing plugin and double-integration audit

No `<plugin>` element exists in the kart SDF. Repository source contains no
Gazebo diff-drive, Ackermann, wheel-torque, steering, brake, or vehicle
dynamics plugin. The ROS bridge subscriber accepts only high-level
`MotionCommand` and explicitly rejects direct actuator mode.

Potential double-authority entry points are:

1. selecting internal physics while configuring Gazebo as dynamics authority;
2. adding a drive, motor, steering, brake, ros2_control, Ackermann, or
   differential-drive plugin to the follower model;
3. accepting Gazebo pose/contact state back into the internal plant;
4. allowing both Gazebo and the internal bridge to publish `/clock`;
5. launching a second odometry/TF or joint-state publisher;
6. using a non-static Gazebo chassis whose gravity/contact response is treated
   as vehicle truth.

All six require explicit configuration and runtime rejection.

## Existing reset and pause behaviour

- `FixedStepScheduler` has no pause/reset API.
- `SimulationClock.reset()` increments its epoch, resets time, and remains
  paused.
- `GazeboVisualFollower` validates a fixed lease epoch but has no queue.
- `VisualFollowerSession.reset()` clears ticks and pauses.
- Scenario reset is performed by constructing a new run.
- No native Gazebo reset flushes sensor messages.
- No native bridge disconnect/staleness diagnostic exists.

## Synchronization risks and missing timing data

- mutable simulator objects could be read asynchronously;
- Gazebo render time could advance independently from internal time;
- Gazebo `/clock` could overwrite internal `/clock`;
- callback arrival time could be mistaken for capture time;
- sensor queues could deliver frames from a previous reset;
- lower follower rate has no interpolation or state-age bound;
- static Gazebo sensor topics currently carry no repository-managed reset
  epoch, delivery time, latency, quality, or fault metadata;
- wheel rotation cannot be reproduced until the internal bridge accumulates it
  from measured wheel speeds;
- process loss currently leaves no explicit invalid/frozen follower status.

## Required correction

The accepted configuration for this task is:

```yaml
simulation:
  physics_backend: internal
gazebo:
  role: visual_sensor_follower
  vehicle_dynamics_authority: false
```

The implementation must add a frozen authoritative snapshot, a one-way
snapshot queue, a static/kinematic Gazebo model plugin, the sole dynamic
clock/TF/joint publishers, bounded timestamp-preserving sensor queues, and
fail-closed diagnostics. Gazebo contacts remain visual-only and must never be
fed back into the internal plant.

## Post-implementation resolution

The audit findings were resolved by the dedicated read-only follower and
sensor-delivery plugins. The final native run continuously followed 1,629
authoritative updates, preserved capture timestamps for non-empty
RGB/depth/LiDAR/IMU payloads through the controlled stop, and reported zero
pose, orientation, joint, timestamp, or double-physics violations. Native
pause/resume/reset/disconnect probes passed, and the final repository suite
passed 138 tests. Detailed evidence is recorded in
`docs/phase_reports/GAZEBO_FOLLOWER_ACCEPTANCE_REPORT.md`.
