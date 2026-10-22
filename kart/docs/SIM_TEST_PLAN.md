# Simulation Platform Test Plan

## Automated matrix

| Area | Evidence |
|---|---|
| Regression | complete pytest suite; pre-task baseline 125, final 138 passed |
| Follower unit | snapshot validation, conversion, interpolation, epochs, stale state, queues, conflicts |
| Authority/safety | VCU path, independent override, direct-owner rejection, E-stop/reset guards |
| Gazebo | native SDF, continuous pose/joints, rendered payloads, clocks, TF and diagnostics |
| Synchronization | native pause/resume/reset/disconnect plus process-loss rejection |
| No double physics | disabled/20/100/200 Hz identical log, conflict rejection, visual-only contact |
| ROS packages | `colcon test` package-local bridge and plugin contracts |
| Autonomy | full Pure Pursuit lap, path-end controlled stop, final VCU `SAFE` |
| Performance | RTF, CPU, RSS, rates, queue depth; GPU explicitly unavailable |

## Commands

```bash
python3 -m pytest -q
python3 -m pytest tests/gazebo_follower -v
./scripts/test_gazebo_follower.sh
./scripts/test_gazebo_synchronization.sh
./scripts/test_gazebo_process_loss.sh
./scripts/test_no_double_physics.sh

source /opt/ros/humble/setup.bash
source /tmp/autokart-ros-install/setup.bash
colcon test --packages-select autokart_sim_bridge autokart_gazebo
colcon test-result --verbose

GAZEBO_MODEL_PATH="$PWD/simulation/models" \
  gz sdf -k simulation/models/autokart/model.sdf
GAZEBO_MODEL_PATH="$PWD/simulation/models" \
  gz sdf -k simulation/worlds/autokart_gazebo_follower_world.sdf
```

## Simulation acceptance thresholds

The position/orientation/steering thresholds are 5 mm or 5 mrad, wheel
rotation 20 mrad, clock offset 2 ms and state age 20 ms. These are simulation
acceptance thresholds, not real-kart limits.

## Remaining external/native scope

- RViz/Gazebo GUI screenshot regression;
- depth point-cloud and camera-info calibration validation;
- native multi-kart Gazebo/TF isolation;
- Chrono/Tier-4 parity;
- Jetson, physical VCU/CAN and real independent-safety hardware.

These remain unexecuted and are not substituted by structural tests.

## Phase 16 real-time qualification

```bash
python3 -m pytest tests/realtime -v
./scripts/test_realtime_qualification.sh
./scripts/run_realtime_soak.sh 1800
```

The native gate requires aggregate RTF of at least 1.0, zero VCU and
independent-safety deadline misses, zero timestamp and double-physics
violations, non-empty payloads, bounded sensor queues, no stale valid
deliveries, and a controlled stop ending in `SAFE`.

Physics-rate qualification compares 500, 400, 250, and 200 Hz against the
validated 1 kHz reference over acceleration, braking, cornering, slalom, wet
stop, steering jam, brake fault, and stuck torque. No lower rate passed all
declared convergence thresholds, so the operational rate remains 1 kHz.

Phase 16 performance evidence is hardware-profile-specific. P3 Gazebo GUI,
the complete P4-P8 ablation matrix, native full-stack 30-minute soak, Jetson,
and physical HIL remain `NOT_EXECUTED`; they are not inferred from the
headless or workstation follower runs.

### Phase 16 matrix status

| ID | Result | Evidence or limitation |
|---|---|---|
| P1 | PASS | native baseline reproduced at RTF 0.181988 |
| P2 | PASS | headless authoritative core RTF above 4; native synchronized follower above 1 |
| P3 | NOT_EXECUTED | Gazebo GUI performance was not substituted by headless rendering |
| P4 | NOT_EXECUTED | RGB-only native ablation |
| P5 | NOT_EXECUTED | RGB+depth native ablation |
| P6 | PASS | RGB+depth+LiDAR+IMU native full lap |
| P7 | PARTIAL | dashboard API/UI tests pass; concurrent native performance run not executed |
| P8 | PASS | bounded evidence recorder enabled; full rosbag disabled |
| P9 | DEGRADED | full native sensor stack sustained RTF, repeat had two VCU misses |
| P10 | PARTIAL | 30-minute headless core soak passed; native full-stack soak not executed |
| N1 | PASS | lower physics rates rejected; retained 1 kHz |
| N2 | PASS | existing render-rate independence hash retained |
| N3 | PASS | sensor preset changes did not change authoritative log hash |
| N4 | PASS | bounded recorder retained authoritative log hash |
| N5 | NOT_EXECUTED | concurrent dashboard independence |
| S1–S7 | PASS_HEADLESS | all seven load campaigns passed; native load repeat not executed |
| A1 | PASS | Pure Pursuit native lap and controlled stop |
| A2–A7 | PARTIAL / NOT_EXECUTED | see `AUTONOMY_ROBUSTNESS.md` |
