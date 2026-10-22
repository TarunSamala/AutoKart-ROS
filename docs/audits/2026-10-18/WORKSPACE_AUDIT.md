# AutoKart workspace audit — 2026-10-18

## Scope and method

This is a stationary, non-actuating audit of `/home/mark1/Desktop/autonomous_kart` as found on 2026-10-18. No vehicle command, firmware flash, or physical stop test was run. The entire tree was inventoried by path, size, and symlink state. Project-owned source and data were checked with parsers, archive CRC checks, model hashes, software suites, and focused review of the safety and launch paths. This does not mean that every generated dependency, binary, image, or historical source line received a semantic review.

The root is a multi-project container, not a Git repository. The largest file groups are `autonomy_stack_phase1` (67,196 paths, 7.9 GB), `phase_sim` (5,735 paths), `kart` (3,702 paths), `datasets` (3,066 paths), and the ESP32 project (2,935 paths). The full walk counted **84,775 files**, **11,865 directories**, and **59 symlinks**. Roughly 9,000 paths remain after common dependency, build, and generated-output exclusions. Eleven `_camera_*` paths are linked Git worktrees, not independent repositories.

## Findings, in priority order

### P0 — E-stop candidates are not functional safety systems

- `XIAO_C3_RA02_ESTOP_PRODUCTION/lora_driver.py:32-42` has no SX127x initialization or transmit implementation. `send()` returns `True` without sending. Its receiver prototype, `receiver_esp32s3_rp2350.py:14-39`, does not parse or qualify a packet; `emergency_stop()` only prints a message. The matching root ZIP has the same files byte for byte.
- In `ESP32S3_RA02_ESTOP_RECEIVER_PRODUCTION.zip`, `lora_driver.py` has `receive()` returning `None`. `main.py` can call `clear_stop()` on a CRC-valid state-0 packet without a manual re-arm; its sequence variable is updated but never used to reject replay. The output pin is constructed without an explicit safe initial value. These are archive contents, not evidence of an installed image. CRC-valid ZIP storage is not functional validation.
- The separate host VCU gate in `vcu/` has no actuator interface by design. Its passing host test does not mitigate these physical E-stop gaps. **Vehicle operation remains NO-GO.**

### P1 — Pin references disagree; deployed firmware identity is unknown

- The six-sheet `AutoKart_S3_Pin_and_Power_Budget_RevA.xlsx` opens. Its 23 GPIO clash formulas in `ESP32-S3 Pin Map!J5:J27` cache `OK`, and the 23 listed GPIO numbers are internally unique. The `Overview`, `Power Budget`, `Signals & Interfaces`, `Wiring Matrix`, and `Open Items & Tests` tabs retain pending physical verification.
- The spreadsheet pin map and `AutoKart_RC_Dynamics_ESP32S3_v1/main/rc_config.h:10-19` assign different functions to at least GPIO **7, 13, 14, 15, 16, 17, 18, and 21**. Examples: spreadsheet GPIO7 is E-stop status while the firmware makes it a service key; spreadsheet GPIO15/16 are wheel encoder channels while the firmware makes them BLDC run/brake-release sinks. These are different design candidates. Do not combine their wiring instructions or infer which image is installed.
- The root `AutoKart_RC_Dynamics_ESP32S3_v1.zip` is readable but differs from the expanded project in **8 of 11** file entries that have a current counterpart. The ZIP is therefore not an exact snapshot of the current expanded source.

### P1 — Source-control coverage is incomplete

- `autonomy_stack_phase1` is at `87f2e288cb1505a9eb233e077603f19689cc3114`, but `git ls-files phase_01_perception` returns **0**. The active perception source is untracked. The main worktree also has modified `.gitignore`, `README.md`, and `setup.sh`, plus untracked `docs/` and `runs/`.
- Six of the eleven `_camera_*` linked worktrees have nonempty `git status` output. They include unique untracked or modified work. Preserve their branch and working-tree state until it is reviewed and captured. A checkout of `main` alone does not recreate the current diagnostic runtime.

### P1 — The launch path does not establish a vehicle command chain

- `kart/ros2_ws/src/autokart_bringup/launch/full_stack.launch.py:6-16` starts only `autokart_sim_bridge`'s `vcu_bridge` with simulated time. `kart/ros2_ws/src/autokart_sim_bridge/autokart_sim_bridge/vcu_bridge.py:27-35` stores the latest `MotionCommand` in memory. It does not transmit to an ESP32 or command an actuator.
- The current Mission Control launch wrapper, `autokart_apps/run_live.sh`, starts the disarmed Python diagnostic. Its default detector is ConeNet v1 (`live_mission.py:528`); YOLO26s must be selected explicitly. Model identity and a working dashboard do not qualify metric cone geometry, odometry, braking, or vehicle authority.

### P2 — Bench static validator is out of step with the files it checks

- `python3 bench/validate_bench.py` **fails**: `steering/main.py` uses `FW_VERSION`, and `web_ui.py` is an HTML asset with no firmware version, while `validate_bench.py:54-55` requires `FIRMWARE_VERSION` in every active `.py`. This is a validation-contract failure; it does not prove that steering motion or stopping is safe or unsafe. No source was changed.

### P3 — Generated build links are stale

- Eight symlinks under the two ESP-IDF `build/esp-idf/mbedtls/.../library/` trees point to deleted `/tmp/autokart-*-idf/` source files. They are generated build references, not active source links. A fresh ESP-IDF build may require regenerating those build trees; no firmware build or flash was attempted.

## Checks completed in this audit

| Check | Current result | Boundary |
|---|---:|---|
| Full filesystem inventory | 84,775 files; 59 links, 8 broken | Metadata, not semantic review of every byte |
| Project-owned Python/JSON/YAML/CSV/XML parse | 1,718 files parsed; 0 parser errors | Generated, vendored and dependency trees excluded |
| Markdown relative links | 92 checked; 0 missing | Simple local-link parser |
| Project ZIP CRC and path traversal check | 6 ZIPs pass CRC; 0 unsafe names | Does not validate behavior |
| Pin/power workbook | 6 sheets open; 23 formula caches `OK`; no duplicate listed GPIO | No recalculation or electrical measurement |
| ARECE dataset | 1,529 images decoded; 1,529 labels paired; 30,803 boxes structurally valid; 0 exact duplicate images | Does not establish label correctness or HP60C performance |
| Three active ONNX model/manifest hashes | YOLO26s, ConeNet v1 and v2 all match | Hashes do not qualify detection accuracy |
| Phase 1 perception tests | 163 passed | Software only |
| Kart tests | 150 passed | Software/simulation only |
| Mission Control Vitest and TypeScript | 47 passed; `tsc --noEmit` passed | UI fixtures and types, not live hardware |
| Mission Control production build | Passed; 954 kB JS chunk advisory | Generated `phase_sim/dist` refreshed |
| ROS 2 fresh build | 16 packages built in isolated `/tmp/autokart_audit_20261018_*` paths | CMake warned that colcon could not register pytest in that environment |
| Direct ROS tests against fresh overlay | 79 passed | No live vehicle graph or physical transport |
| ESP32 static tests and nested WebStop static check | 9 passed; WebStop check passed | No image build, flash, or output verification |
| VCU Stage 0 host test | Adversarial C assertions passed | Host-only gate; `vcu/tests/test_guard` rebuilt |
| Bench static validation | **FAILED** on two version-field checks | Validator/file contract drift |

The pre-existing `kart/ros2_ws/install` overlay lacked generated `MissionDebug`; direct ROS tests failed during import against it. A fresh `/tmp` build generated the current message and all 79 selected ROS tests then passed. This is an installed-overlay freshness issue, not evidence that the source package is missing the message.

## Open qualification evidence

No surveyed HP60C depth scale, RGB/depth alignment accuracy, mounted transform, independently measured cone positions, qualified model labels, live odometry, installed ESP32 image/pin map, actuator feedback, independent traction isolation, physical E-stop, or braking performance was established in this audit. Hardware USB enumeration or passing software tests cannot close these gates. **Software-only stationary diagnostics: GO. Vehicle motion: NO-GO.**

## Audit side effects

No application source, firmware, model, spreadsheet, or historical worktree was edited. `npm run build` refreshed generated `phase_sim/dist` output, `make test` rebuilt `vcu/tests/test_guard`, and the ROS build wrote only under `/tmp/autokart_audit_20261018_*`. This report is the only authored audit file in the workspace.
