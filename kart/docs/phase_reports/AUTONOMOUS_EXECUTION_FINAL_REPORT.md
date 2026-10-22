# Autonomous Execution Final Gap Report

## Phases and verdicts

| Phase | Local software/framework | Native/physical evidence | Overall | Verdict |
| --- | --- | --- | --- | --- |
| 7 | COMPLETE; 79/79 mandatory software gates PASS | 3 hardware validations not executed | COMPLETE software gate | APPROVED_WITH_CONDITIONS |
| 8 | COMPLETE | native ROS not executed | PARTIAL | APPROVED_WITH_CONDITIONS |
| 9 | COMPLETE headless/static framework | native live ROS/rendering not executed | PARTIAL | APPROVED_WITH_CONDITIONS |
| 10 | COMPLETE campaign framework; tested catalog partial | real fault distributions absent | PARTIAL | APPROVED_WITH_CONDITIONS |
| 11 | COMPLETE backend boundary | native Chrono not executed | PARTIAL | MORE_DATA_REQUIRED |
| 12 | COMPLETE world/track/callback boundary | native CARLA/Unreal not executed | PARTIAL | MORE_DATA_REQUIRED |
| 13 | COMPLETE simulation CAN/HIL boundary | vCAN/native hardware not executed | PARTIAL | MORE_DATA_REQUIRED |
| 14 | COMPLETE calibration framework | real data not executed | PARTIAL | MORE_DATA_REQUIRED |
| 15 | COMPLETE prototype handover framework | external/real demonstration absent | PARTIAL | MORE_DATA_REQUIRED |

## Tests

- Full locally executable total: 98 passed, 0 failed, 0 skipped.
- Focused Phase 7: VCU 10, controlled stop 9, E-stop 5, envelope 4.
- Critical fault campaign: 3/3 pass; tested catalog IDs 3/3; invariant
  violations 0.
- Native/external tests were not marked skipped or passed; they are
  `NOT_EXECUTED_EXTERNAL_DEPENDENCY`.

## External integrations

Executed:
- Deterministic mock ROS transport and wire conversion.
- Analytical backend and explicit Chrono gate.
- Headless world follower/track/sensor callback mock.
- Simulation-only CAN plant bridge.
- Synthetic calibration recovery.

Not executed:
- Native ROS graph/DDS/rosbag.
- Project Chrono dynamics.
- CARLA/Unreal rendering and native sensor callbacks.
- vCAN, Jetson, RP2350 VCU, physical CAN/HIL.
- Real-kart data collection, fit or held-out validation.

## Fault and safety results

- Phase 7 covers command loss, steering faults, brake-chain faults, traction
  faults, VCU/safety-task freeze, physical-input E-stop simulation, wet/split
  friction, stop completion, reset guards, safety envelope and combined
  stuck-torque/welded-contactor response.
- Command-timeout reference: action 0.205 s, torque zero 0.215 s, brake
  response 0.213 s, stop 0.880 s, 0.821 m after detection.
- Stuck-torque reference: detection 0.253 s, torque below threshold 0.320 s.
- Physical-input E-stop simulation: action 0.251 s, torque zero 0.263 s,
  pressure response 0.259 s, stopped 0.696 s.
- Wet/cable-break scenarios that did not stop inside their configured horizon
  are not reported as successful stops.

## Gap classification

Software defects:
- No known failing Phase 7 mandatory software item.
- Fault benchmark coverage is risk-prioritized, not exhaustive.
- Phases 5/6 retain reduced track/sensor/rendering fidelity limitations.

Missing native dependencies:
- `rclpy`, Project Chrono, CARLA and Unreal APIs.

Missing hardware:
- Jetson, RP2350 VCU, vCAN/physical CAN test bench, independent watchdog,
  contactor/brake/steering bench and physical E-stop loop.

Missing physical measurements:
- Actuator onset/response, pressure/deceleration, torque decay, contactor time,
  watchdog time, sensor latency/failure behavior, tire/surface parameters.

Missing real-kart evidence:
- Approved staged testing, immutable datasets, calibration/validation split,
  parameter uncertainty and independent sim-to-real validation.

## Component status

- Chrono: interface/configuration only; native validation not executed.
- CARLA/Unreal: authority/coordinate/track/callback contracts only; rendered
  fidelity not executed.
- SIL/HIL: mock ROS and simulated CAN plant only; no hardware success claim.
- Calibration: synthetic only; digital-twin claim not justified.

## Release artifacts

- Phase reports and machine-readable Phase 7 checklist.
- `docs/EXTERNAL_INTEGRATION_STATUS.md`
- `docs/CHANGE_MANIFEST.md`
- `docs/GIT_RECOVERY.md`
- Reference safety/backend/world/HIL/calibration configurations.
- Portable source archive recorded in the change manifest.
- Archive: `artifacts/kart-source-checkpoint-20260729.tar.gz` (140 KiB),
  SHA-256
  `54c1b059b84a4b5eb065ceede144ac4e6e80818f0abb20dca6a0760340ccb8ee`.

## Repository

REPOSITORY COMMIT:
NOT AVAILABLE — INVALID GIT METADATA

## Real-vehicle readiness

REAL-VEHICLE READINESS:
REJECTED

This verdict cannot change without physical E-stop, actuator, contactor,
independent watchdog, HIL, progressive low-speed test and independent
sim-to-real evidence.

## Exact reproduction commands

```bash
./scripts/test.sh
PYTHONPATH=. python3 -m kart_sim.cli safety validate configs/safety/prototype_safety.yaml
PYTHONPATH=. python3 -m kart_sim.cli safety campaign --output-dir runs/safety-campaign
PYTHONPATH=. python3 -m kart_sim.cli test-model controlled-stop
PYTHONPATH=. python3 -m kart_sim.cli test-model emergency-stop
PYTHONPATH=. python3 -m kart_sim.cli ros2 status
```
