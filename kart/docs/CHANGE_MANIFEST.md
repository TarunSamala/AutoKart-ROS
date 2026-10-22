# Change Manifest

Git history is unavailable; this file is the portable engineering change
record. `docs/GIT_RECOVERY.md` records the exact failure.

## Phase 7 remediation

Files created include:

- `kart_sim/communication.py`, `fault_manager.py`, `watchdog.py`,
  `safety_config.py`, `safety_health.py`, `safety_diagnostics.py`,
  `safety_metrics.py`
- Phase 7 VCU-mode scenarios under `configs/scenarios/vcu_*.yaml`
- `configs/safety/prototype_safety.yaml`
- Phase 7 unit/integration campaigns and safety metrics tests
- VCU, command, watchdog, plausibility, independent-safety, fault, FMEA,
  requirements, checklist, and report documentation

Files materially modified include:

- `kart_sim/vcu.py`, `scenario.py`, `dynamic_kart.py`, `actuators.py`,
  `powertrain.py`, `electrical.py`, `four_wheel.py`, `cli.py`
- `docs/STATUS.md`, `ASSUMPTIONS.md`, `FMEA.md`,
  `SAFETY_REQUIREMENTS.md`

Configuration changes:

- Explicit `direct_test`/`vcu` control modes, physical actuator fault
  scenarios, provisional safety limits, no automatic autonomous restart.

Verification:

- Baseline preserved: 31/31 before remediation.
- Current full suite: 98/98 passed on 2026-07-29.
- Focused CLI: VCU 10 passed; controlled stop 9 passed; E-stop 5 passed;
  safety envelope 4 passed; critical campaign 3/3 passed.

## Phase 8 local framework

- Added transport-neutral ROS/SIL contracts, ROSIDL message files,
  CRC-preserving wire conversion, clock/frame/QoS behavior, mock transport,
  trajectory/controllers, and optional package build boundary.
- Native ROS runtime test remains external.

## Phase 9 local framework

- Added bounded telemetry store, deterministic replay/comparison, static
  HTML/CSV/JSON exports, fault visibility, decimation and connection-loss
  state. Dashboard remains read-only.

## Phase 9 dashboard rebuild

Created:

- Modular `dashboard/` frontend with a perspective 3D track/kart scene,
  responsive engineering layout, seven cameras, overlays, subsystem/safety
  panels, synchronized plots, friction circle, timeline, replay and comparison.
- `kart_sim/dashboard_server.py` with GET-only API, bounded WebSocket and live
  JSONL tail.
- Track-backed research-circle and scheduled-slalom scenarios, visual replay
  generator, frontend and Python tests, audit/user/model/safety documentation,
  and `PHASE_09_DASHBOARD_REBUILD_REPORT.md`.

Modified:

- Simulator telemetry now records explicit surface-frame z/roll/pitch,
  requested/validated/limited/final commands, Phase 5 body/wheel projection,
  wheel materials, safety limits and stopping-distance components.
- CLI now provides `dashboard serve`, `dashboard launch`, and full static bundle
  export. Optional `aiohttp` dependency is documented.

Verification:

- Baseline: 98/98.
- Current: 107/107.
- Local API returned 123 real Phase 5 track samples and 6,000 measured
  telemetry samples; mutation returned HTTP 405.
- WebSocket verification observed changing measured pose from
  `(10.0, 0.003)` to `(9.998, 0.619)`.

## Phase 10 local framework

- Added machine-readable critical catalog, physical telemetry safety oracle,
  deterministic runner/coverage, resume, pairwise generation, seeded parameter
  sampling, and one-minimal failure reduction.

## Phase 11 local framework

- Added backend-neutral protocol, analytical adapter, explicit Chrono
  dependency boundary, configuration and no-silent-fallback tests.
- Native Chrono dynamics remain external.

## Phase 12 local framework

- Added authority/synchronization and coordinate contracts, visual follower,
  actor validation, deterministic track export, and render-sensor callback
  mock.
- Native CARLA/Unreal execution remains external.

## Phase 13 local framework

- Added simulation-only CAN codec, SAE-J1850 CRC, counters, timeouts, timing,
  bus load, vCAN detection, simulated plant bridge, protocol YAML, and HIL
  safety documentation.
- No real interface or hardware was activated.

## Phase 14 local framework

- Added data metadata/split schema, immutable checksum, parameter registry,
  CSV import, bounded deterministic optimizer, synthetic recovery and report
  template.
- No real dataset was available.

## Phase 15 release/handover

- Updated phase reports, external gap status, reproducibility commands, safety
  evidence, and real-readiness rejection.

REPOSITORY COMMIT:
NOT AVAILABLE — INVALID GIT METADATA

## Portable checkpoint

- File: `artifacts/kart-source-checkpoint-20260729.tar.gz`
- Size: 140 KiB
- SHA-256:
  `54c1b059b84a4b5eb065ceede144ac4e6e80818f0abb20dca6a0760340ccb8ee`
- Excludes `.git`, test caches, Python bytecode caches, generated runs and
  existing artifacts.
- The archive was built immediately before adding this checksum paragraph; the
  checksum paragraph itself is therefore recorded in the live workspace and
  final response, not recursively inside the archive.
# 2026-07-29 — Continuous Gazebo follower acceptance

- Added immutable authoritative snapshots, wheel-rotation accumulation,
  bounded interpolation, reset epochs and state-age validation.
- Added native static Gazebo pose/joint follower with sole clock, odometry,
  dynamic TF, joint state, diagnostics and pause/disconnect watchdog.
- Added timestamp-preserving RGB/depth/LiDAR delivery plugins and cumulative
  bounded-queue metadata; retained internal IMU as sole authority.
- Added the follower world, full-lap scenario, acceptance thresholds and
  320x240 Tier-3 render configuration.
- Added native full-lap, synchronization, process-loss, no-double-physics,
  recording and reporting scripts.
- Added ROS messages for authoritative state, follower state and sensor
  delivery metadata.
- Added follower unit/integration/native contract tests and ROS package-local
  tests.
- Added checksummed full-lap artifacts, representative sensor samples and
  twelve plots.
- Baseline before task: 125 tests passed. Existing simulator, VCU, independent
  safety, faults, telemetry and dashboard were retained.
- Final validation: 138 repository tests and six ROS package tests passed;
  native lap, synchronization, process-loss and no-double-physics checks
  passed.

# 2026-07-30 — Phase 16 real-time SIL qualification

- Added real-time, balanced, and high-fidelity Gazebo presets plus workstation,
  software-render, Jetson, and CI hardware profiles.
- Added task timing budgets, bounded percentile/jitter monitoring, scheduler
  modes, native qualification gates, and categorized sensor-drop counters.
- Isolated the authoritative plant from the ROS executor using a bounded
  one-way snapshot queue and separate process/CPU assignments.
- Reduced snapshot allocation and evidence hashing without reducing the
  validated 1 kHz physics, VCU, independent-safety, or actuator execution.
- Added runtime SDF generation so sensor fidelity is explicit and
  preset-specific.
- Added the dashboard performance panel and API for RTF, task latency,
  follower/sensor latency, queues, drops, resources, and deadline misses.
- Added flat authoritative-snapshot replay normalization and a Phase 16
  dashboard launcher; the final native artifact loads all 16,560 snapshots.
- Added physics-rate convergence, critical-fault load, 30-minute soak, native
  qualification, profiling, and Jetson preparation commands.
- Workstation native result: first run RTF `1.004939`, final `SAFE`, zero
  VCU/safety misses. The exact repeat reached RTF `1.004746` and the same
  authoritative hash but recorded two 1 ms VCU wall-deadline misses, so the
  final repeat is honestly classified `REALTIME_DEGRADED`.
- Final Phase 16 repository regression: 145 passed in 54.53 s.
- ROS dependency-closure build passed; package-local result was 6 tests,
  0 errors, 0 failures, and 0 skipped.
- Real Jetson, physical HIL, rendered-sensor cone perception, and the full
  native robustness/soak matrix remain unexecuted.
