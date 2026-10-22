# AutoKart project index

This is a source workspace for simulator, perception, diagnostic UI, bench firmware and safety evidence. The workspace root is **not a Git repository**. The main perception checkout and the ESP32 project have separate nested Git metadata; the `_camera_*` directories are linked worktrees in the perception repository. Use the evidence and decisions below before changing paths.

## Start here

- [Repository map](docs/architecture/REPOSITORY_MAP.md): ownership, entry points and active versus incomplete responsibilities.
- [Canonical component selection](docs/decisions/CANONICAL_COMPONENTS.md): current local software command/model choice and what remains unknown.
- [Repository disposition](docs/decisions/REPOSITORY_DISPOSITION.md): keep/archive/cache decisions and approval gates.
- [Verification runbook](docs/runbooks/VERIFY_REPOSITORY.md): baseline commands and expected results.
- [Node contracts](docs/NODE_CONTRACTS.md): staged ROS adapter inputs/outputs and freshness gates.
- [Integration decisions](docs/INTEGRATION_DECISIONS.md): package name/migration resolution and authoritative algorithm imports.
- [Test matrix](docs/TEST_MATRIX.md): untouched baseline and post-change regression results.
- [Dashboard data contract](docs/DASHBOARD_DATA_CONTRACT.md): existing read-only gateway/UI data flow.
- [Integration status](docs/RELEASE_REPORT.md): completed scope and remaining evidence gates.
- [VCU Stage 0 command gate](vcu/README.md) and [test record](vcu/TEST_REPORT.md): host-only, non-actuating C safety boundary; no vehicle firmware or hardware qualification.
- [VCU ROS bench status contract](docs/NODE_CONTRACTS.md): typed, read-only projection of the existing NOOP bench packet; separate from LIVE vehicle state.
- [VCU ROS Stage 1 build report](docs/VCU_ROS_STAGE1.md): package metadata repair, 16-package builds, discovery and remaining M4 gates.
- [RC-01 audit copies](docs/audits/2026-10-16/): dated audit evidence, retained alongside the originals under `_autokart_full_audit/`.
- [Symlink inventory](docs/audits/2026-10-16/SYMLINK_INVENTORY.csv): all 55 symlink paths and recorded targets.
- [Cleanup diff](docs/audits/2026-10-16/CLEANUP_DIFF.md): this documentation-only change, generated build outputs and before/after sizes.

## ⚠️ NO-GO — new VCU integration pieces added, none run against hardware

A later change added the pieces this index previously listed as missing for one mission (drive straight between
cones at a fixed low speed, stop before the cone ahead): `vcu_firmware/` (new, separate ESP32-S3 firmware,
[README](vcu_firmware/README.md)), `kart/ros2_ws/src/autokart_vehicle_bridge` (real USB-serial bridge),
`kart/ros2_ws/src/autokart_localization` (wheel-encoder odometry, filling a previously-empty package shell),
and `kart/ros2_ws/src/autokart_vcu_safety` (the VCU arming safety supervisor — distinct from
`autokart_safety_bridge`'s diagnostic-only front obstacle envelope monitor). Existing perception, mapping,
planner, controller, and `phase_sim`/`autokart_dashboard_gateway` Mission Control were reused, not rewritten;
the planner gained one new parameter (`brake_margin_m`) and the dashboard gateway gained one bounded
arm/stop/reset_fault/confirm_manual_power_cut request path. See [docs/VCU_PROTOCOL.md](docs/VCU_PROTOCOL.md) and
[docs/BENCH_TESTS.md](docs/BENCH_TESTS.md).

**None of this has been flashed, run against hardware, or bench-tested.** Only pure logic is unit-tested on the
host (firmware state machine/protocol/CRC, the bridge's command gate and throttle-voltage mapping, odometry,
the safety supervisor's state machine, and the planner's stop-distance margin behaviour). `vehicle_authorized`
stays `false` everywhere; `calibration_verified`, `vehicle_config_verified` and `braking_verified` stay `false`
in `kart/ros2_ws/src/autokart_vcu_safety/config/vcu_safety.yaml` until a person completes
`docs/BENCH_TESTS.md` and sets them — nothing in this change sets them automatically.

## Current safety boundary

Vehicle operation is not authorized. Current diagnostics keep `vehicle_authorized=false`; no VCU command interface has been verified end to end. The Pi runtime, installed ESP32 image, IMU connection, as-built wiring, independent physical E-stop, brake feedback and stopping performance remain unknown or unverified. Do not infer physical capability from simulation, a firmware build, a model hash or a passing software suite.

RC-01 documented the existing tree. RC-02 begins the ROS adapter integration by grouping the ROS package shells and installing the existing Phase 1 Python source through the ROS package build. It does not merge branches, choose a deployed firmware image, change model defaults, alter safety logic, or move/delete historical worktrees or artifacts.
