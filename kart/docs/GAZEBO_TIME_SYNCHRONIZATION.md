# Gazebo Time Synchronization

Internal simulation time is authoritative. Browser time, render wall time and
callback arrival time are never used as measurement time.

## Step policy

1. Advance the internal plant by its fixed 1 ms step.
2. Create a frozen state snapshot after actuator and vehicle response.
3. Publish selected snapshots at 100 Hz to a bounded reliable bridge.
4. Advance Gazebo in 1 ms steps and interpolate pose/joints between snapshots.
5. Pause Gazebo at the newest applied authoritative time.
6. Capture rendered sensors at Gazebo simulation time.
7. Queue payloads until processing plus transport latency is due.
8. Publish payload headers with capture time and metadata with all three times.

The follower publishes `/clock`; the old Gazebo-to-ROS clock mapping was
removed. Native acceptance records internal time, Gazebo time, capture time,
processing completion, delivery time and wall performance separately.

## Rate policy

The acceptance scenario configures 1000 Hz physics, 250 Hz VCU, 100 Hz
follower, 30 Hz RGB, 30 Hz depth, 20 Hz LiDAR and 200 Hz IMU. Follower
interpolation is limited to 20 ms and never extrapolates. Rendered state older
than 20 ms is dropped and counted.

## Pause, reset and loss

- Pause: the publisher remains connected but stops snapshots. Gazebo freezes,
  `/clock` does not advance and sensor delivery quiesces.
- Resume: the next monotonic snapshot resumes without a pose jump or duplicate
  sequence.
- Reset: a greater reset epoch permits an explicit rewind, clears queues and
  leaves command/autonomy re-arming outside the follower.
- Bridge delay: interpolation is bounded to 20 ms. A subsequent snapshot with
  a larger interval freezes and invalidates the follower, records the state
  age, stops world/sensor progression and emits a critical diagnostic.
- Disconnect: after 500 ms with no publisher, state is frozen and invalid.
- Gazebo loss: the main acceptance runner requires full pose/sensor coverage
  through controlled stop and exits non-zero if the process disappears.

`artifacts/gazebo_follower/synchronization/sync_probe.json` records SY1 through
SY5. `artifacts/gazebo_follower/failure/process_loss.log` records SY6.

The measured nominal maximum Gazebo/internal clock offset was 1 ms, below the
2 ms simulation acceptance threshold.
