PHASE:
Phase 15 — Final Research Demonstration and Release

SOFTWARE RELEASE FRAMEWORK:
COMPLETE

EXTERNAL VALIDATION:
NOT_EXECUTED_EXTERNAL_DEPENDENCY

OVERALL STATUS:
PARTIAL

VERDICT:
MORE_DATA_REQUIRED

RELEASE:
- Version: 0.1.0-prototype.
- Commit: NOT AVAILABLE — INVALID GIT METADATA.
- Full local suite: 98 passed, 0 failed, 0 skipped.
- Portable checkpoint, phase reports, configs, safety evidence and change
  manifest are provided.

DEMONSTRATED LOCALLY:
- Deterministic analytical dynamics, sensor/timing foundations, VCU-integrated
  physical fault scenarios, 79/79 Phase 7 software gates, mock SIL, read-only
  dashboard exports, critical campaign, backend/world/HIL/calibration
  interfaces.

NOT EXECUTED:
- Native ROS graph, Chrono, CARLA/Unreal, vCAN/real HIL, Jetson/RP2350, real
  sensors, real kart calibration, independent sim-to-real validation.

RESEARCH CLAIM:
- Prototype software architecture and deterministic simulation evidence only.
- No certification, production VCU, native backend parity, validated digital
  twin, or real-vehicle safety claim.

REAL-VEHICLE READINESS:
REJECTED

REPRODUCTION:
- `./scripts/test.sh`
- `PYTHONPATH=. python3 -m kart_sim.cli safety campaign --output-dir runs/safety-campaign`
- `PYTHONPATH=. python3 -m kart_sim.cli ros2 status`
- See `docs/EXTERNAL_INTEGRATION_STATUS.md`.
