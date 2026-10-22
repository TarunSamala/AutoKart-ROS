# Demonstration Matrix

Commands are reproducible; status records native execution on 2026-07-29.

| # | Demonstration | Command | Status |
|---:|---|---|---|
| 1 | Continuous kart in Gazebo | `./scripts/run_gazebo_follower.sh` | PASS: complete internal-authority lap |
| 2 | Onboard RGB camera | same | PASS: 496 native captures, changing non-empty payloads |
| 3 | LiDAR | same | PASS: 330 native scans, cone/wall and pose-dependent returns |
| 4 | RGB-D depth | same | PASS: 496 native `32FC1` captures with finite depth |
| 5 | RViz stack | `rviz2 -d .../full_autonomy.rviz` | configuration exists; GUI screenshot not executed |
| 6 | Web dashboard | `./scripts/run_dashboard.sh` | existing automated dashboard tests PASS |
| 7 | Fast lap | `./scripts/run_fast_sim.sh` | VCU-routed deterministic fast mode PASS |
| 8 | Acceleration | `./scripts/run_formula_student_event.sh acceleration` | event foundation/command available |
| 9 | Skidpad | `./scripts/run_formula_student_event.sh skidpad` | event foundation/command available |
| 10 | Cone lap | `./scripts/run_formula_student_event.sh autocross` | framework PASS; native cone perception lap remains separate |
| 11 | Multi-kart | `./scripts/run_multi_kart_race.sh` | automated namespace/race smoke PASS |
| 12 | Steering fault | `./scripts/run_fault_demo.sh steering_jam` | existing latched-stop campaign PASS |
| 13 | Brake cable | `./scripts/run_fault_demo.sh brake_cable_break` | existing safety response coverage |
| 14 | Stuck torque | `./scripts/run_fault_demo.sh stuck_motor_torque` | existing isolation/E-stop coverage |
| 15 | VCU freeze | `./scripts/run_fault_demo.sh vcu_freeze` | existing independent-watchdog coverage |
| 16 | Physical E-stop simulation | `./scripts/run_fault_demo.sh physical_estop` | existing propulsion/latch coverage |
| 17 | Wet stop | `./scripts/run_fault_demo.sh wet_surface_stop` | existing surface/stop coverage |
| 18 | Replay/timeline | dashboard export/serve | existing replay and fault timeline tests PASS |

Follower-specific evidence:

```bash
./scripts/test_gazebo_synchronization.sh
./scripts/test_gazebo_process_loss.sh
./scripts/test_no_double_physics.sh
```

See `GAZEBO_FULL_LAP_DEMO.md` and
`artifacts/gazebo_follower/full_lap/run_manifest.json`.

## Phase 16 performance demonstrations

| Demonstration | Command | Result |
|---|---|---|
| Real-time native lap | `./scripts/run_gazebo_realtime_sil.sh` | Throughput PASS: RTF 1.00494/1.00475, final `SAFE`; repeat gate DEGRADED by two VCU misses |
| Balanced native lap | `./scripts/run_gazebo_balanced.sh` | Throughput target PASS at RTF 0.74007; timing class degraded by one VCU outlier |
| Offline fidelity preset | `./scripts/run_gazebo_offline_fidelity.sh` | Configuration validated; native run not executed |
| Physics convergence | `./scripts/profile_autokart_sim.sh` | PASS: retain 1 kHz; lower candidates rejected |
| Critical faults under load | `./scripts/test_realtime_qualification.sh` | Seven headless load cases PASS; native Gazebo load not executed |
| 30-minute core soak | `./scripts/run_realtime_soak.sh 1800` | See `artifacts/phase16/soak/soak_results.json` |
| Phase 16 dashboard | `./scripts/run_phase16_dashboard.sh artifacts/phase16/realtime_final` | PASS: 16,560 native snapshots and degraded timing status load directly |
| Jetson environment | `./scripts/check_jetson_environment.sh` | `NOT_EXECUTED_EXTERNAL_DEPENDENCY` on workstation |

The real-time lap uses the ground-truth-debug path reference with genuine
closed-loop Pure Pursuit commands routed through the VCU. It does not claim
rendered-sensor cone perception.
