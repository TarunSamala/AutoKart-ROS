# AutoKart Sim Studio User Guide

Install, validate and build:

```bash
./scripts/setup.sh
./scripts/check_sim_dependencies.sh
./scripts/build_ros2.sh
python3 -m pytest -q
```

## Common launches

```bash
./scripts/run_fast_sim.sh
./scripts/run_gazebo_sim.sh
./scripts/run_formula_student_event.sh skidpad
./scripts/run_multi_kart_race.sh
./scripts/run_dashboard.sh
./scripts/run_ros_sil.sh
./scripts/run_fault_demo.sh steering_jam
```

## Native follower

```bash
./scripts/run_gazebo_follower.sh \
  --scenario configs/scenarios/gazebo_follower_full_lap.yaml
./scripts/report_gazebo_follower.sh
```

The command runs Gazebo Classic in lockstep with a static follower model. It
does not make Gazebo a vehicle dynamics authority. Artifacts are written to
`artifacts/gazebo_follower/full_lap`.

To record a large local rosbag:

```bash
./scripts/record_gazebo_full_lap.sh
```

Do not commit large bags without repository policy approval. The normal run
keeps checksummed metadata, plots and representative sensor samples.

## Verification

```bash
./scripts/test_gazebo_synchronization.sh
./scripts/test_gazebo_process_loss.sh
./scripts/test_no_double_physics.sh
./scripts/test_gazebo_follower.sh
```

`direct_test` remains a labelled low-level component mode. Autonomous and
safety evaluation must use VCU mode. Do not add a Gazebo drive/controller
plugin while `physics_backend: internal` is selected. Real actuator outputs
remain disabled in repository presets.

See `GAZEBO_FOLLOWER_ARCHITECTURE.md`, `GAZEBO_TIME_SYNCHRONIZATION.md` and
`GAZEBO_SENSOR_PIPELINE.md` for integration details.

## Phase 16 real-time presets

The three native follower presets preserve the same internal-authority command
path and 1 kHz physics, VCU, and independent-safety rates:

```bash
./scripts/run_gazebo_realtime_sil.sh
./scripts/run_gazebo_balanced.sh
./scripts/run_gazebo_offline_fidelity.sh
```

`gazebo_realtime_sil` is the workstation wall-synchronized qualification
candidate. One run passed every gate, while the exact repeat had two VCU
deadline misses and was correctly classified `REALTIME_DEGRADED`; stable
zero-miss qualification is therefore still open. The preset uses 160x90 RGB
and depth at 10 Hz, 180-sample LiDAR at 10 Hz, and internal IMU at 200 Hz.
These are SIL transport settings, not evidence for high-resolution perception.

Use the following commands to reproduce the Phase 16 evidence:

```bash
./scripts/profile_autokart_sim.sh
./scripts/test_realtime_qualification.sh
./scripts/run_realtime_soak.sh 1800
./scripts/report_realtime_performance.sh \
  artifacts/phase16/realtime_qualified
./scripts/run_phase16_dashboard.sh \
  artifacts/phase16/realtime_final
```

The web dashboard performance tab reads the selected run's bounded evidence
and displays RTF, task timing, follower/sensor latency, queue depths, categorized
drops, resource status, and the qualification class. A real-time-qualified
workstation run does not qualify Jetson, HIL, or a physical kart.
