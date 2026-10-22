# Project Status

## Environment audit — 2026-07-29

- Ubuntu 22.04-family, Python 3.10.12.
- Full locally executable suite after the AutoKart Sim Studio upgrade:
  **125 passed, 0 failed, 0 skipped**. The pre-upgrade baseline was 107 passed.
- Git metadata is invalid: `.git` is an empty directory. Recovery options are
  documented in `docs/GIT_RECOVERY.md`; no destructive repair was attempted.
- ROS 2 Humble CLI and `rclpy`: detected. Twelve new workspace packages build
  natively under `/tmp`; a custom-message graph round trip passed over local
  shared memory. UDP transport is restricted by the sandbox.
- Gazebo Classic 11.10.2: detected. Native SDF checks, empty-world launch,
  static visual-follower spawn/stability, and RGB/depth/LiDAR/IMU interface
  advertisement passed. Continuous state following and payload capture remain
  incomplete.
- Project Chrono bindings: unavailable.
- CARLA and Unreal Python APIs: unavailable.
- `vcan0`, Jetson, RP2350 VCU and physical CAN/HIL hardware: unavailable.
- Real-kart datasets and measurements: unavailable.

## Phase status

- Phases 1–6 remain research-prototype implementations with the limitations
  documented in their model documents. Phase 5/6 do not satisfy every original
  high-fidelity rendering/sensor criterion.
- Phase 7 software/simulation acceptance is **COMPLETE**: 79/79 mandatory
  software checklist entries PASS. Physical E-stop, actuator timing and
  independent watchdog hardware evidence are
  `NOT_EXECUTED_EXTERNAL_DEPENDENCY`.
- Phase 8 software framework is complete locally; native ROS graph validation
  is `NOT_EXECUTED_EXTERNAL_DEPENDENCY`. Overall Phase 8 remains PARTIAL.
- Phase 9 dashboard software is complete locally: paced live simulator tail,
  bounded WebSocket, replay/export/comparison, Phase 5 track and measured kart
  scene, engineering panels/plots and persistent safety display. Real kart CAD,
  surveyed track, real sensor media, native ROS feed, and automated headless
  pixel-regression evidence remain unavailable.
- Phase 10 deterministic campaign framework, critical smoke catalog, resume,
  pairwise sampling and minimization are implemented; tested catalog coverage
  is not exhaustive.
- Phase 11 backend boundary/configuration is implemented; native Chrono
  dynamics are `NOT_EXECUTED_EXTERNAL_DEPENDENCY`.
- Phase 12 world/coordinate/track/sensor callback contracts are implemented;
  native CARLA/Unreal rendering is `NOT_EXECUTED_EXTERNAL_DEPENDENCY`.
- Phase 13 CAN codec/counters/CRC/timeouts/plant bridge are implemented;
  vCAN/native HIL is `NOT_EXECUTED_EXTERNAL_DEPENDENCY`.
- Phase 14 data governance/splits/registry/optimizer/report framework is
  implemented; real-data validation is `NOT_EXECUTED_EXTERNAL_DEPENDENCY`.
- Phase 15 remains a prototype research release with external integrations and
  physical validation outstanding.

## AutoKart Sim Studio upgrade

- Status: **PARTIAL**
- Physics-authority and multi-mode architecture: implemented and tested.
- Fast VCU-routed racing/RL, track conversion, cones, baseline autonomy,
  multi-kart and race-control core: implemented and tested.
- Xacro/URDF/SDF, six Gazebo worlds, ROS 2 messages/packages/launches and nine
  RViz configurations: implemented; native build/structural/spawn smokes pass.
- Continuous internal-state-to-Gazebo driving follower, ROS rendered-sensor
  payload bridge, full event demonstrations, Gazebo-authoritative plugin,
  Chrono and physical SIL/HIL: incomplete or not executed.
- Detailed verdict: `docs/phase_reports/AUTOKART_SIM_STUDIO_UPGRADE_REPORT.md`.

## Safety verdict

REAL-VEHICLE READINESS: **REJECTED**

Simulation acceptance does not establish safe real-vehicle operation,
functional-safety compliance, digital-twin validity, or hardware readiness.
See `docs/EXTERNAL_INTEGRATION_STATUS.md` and the phase reports.
