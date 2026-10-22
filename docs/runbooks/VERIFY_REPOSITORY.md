# Verify repository state (RC-01)

Run commands from the stated directories. They are software checks only; do not attach a kart, enable actuators, flash firmware or issue motion commands.

## Baseline and post-documentation checks

| Purpose | Command | RC-01 result |
|---|---|---|
| Phase 1 perception | `cd /home/mark1/Desktop/autonomous_kart/autonomy_stack_phase1/phase_01_perception && PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src:/home/mark1/Desktop/autonomous_kart/kart python3 -m pytest -p no:cacheprovider -q tests` | Baseline: 139 passed, exit 0, 7.14 s. Post-doc: 139 passed, exit 0, 8.29 s. |
| Kart simulator | `cd /home/mark1/Desktop/autonomous_kart/kart && PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=. python3 -m pytest -p no:cacheprovider -q tests` | Baseline: 149 passed, exit 0, 64.13 s. Post-doc: 149 passed, exit 0, 55.54 s. |
| LAB UI tests | `cd /home/mark1/Desktop/autonomous_kart/phase_sim && npm run test -- --run` | Baseline: 8 files / 32 tests passed, exit 0, 7.97 s. Post-doc: 8 files / 32 tests passed, exit 0, 9.25 s. |
| LAB UI build | `cd /home/mark1/Desktop/autonomous_kart/phase_sim && npm run build` | Baseline and post-doc passed, each 53 modules; 0.77 s / 1.53 s. 905.32 kB JS bundle (247.31 kB gzip) and >500 kB advisory. |
| ROS contract tests | `cd /home/mark1/Desktop/autonomous_kart/kart && PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=.:ros2_ws/src/autokart_dashboard_gateway:ros2_ws/src/autokart_sim_bridge python3 -m pytest -p no:cacheprovider -q ros2_ws/src/autokart_dashboard_gateway/test ros2_ws/src/autokart_sim_bridge/test ros2_ws/src/autokart_gazebo/test` | Baseline: 10 passed, exit 0, 0.04 s. Post-doc: 10 passed, exit 0, 0.12 s. |
| ROS build | `cd /home/mark1/Desktop/autonomous_kart/kart/ros2_ws && source /opt/ros/humble/setup.bash && colcon --log-base /tmp/autokart_rc01_ros_log build --base-paths src --build-base /tmp/autokart_rc01_ros_build --install-base /tmp/autokart_rc01_ros_install` | Baseline: 13 packages passed, exit 0, 49.2 s. Post-doc: 13 packages passed, exit 0, 5.42 s (incremental). All output is under `/tmp`; known CMake/Python packaging warnings only. |
| ESP32 source static tests | `cd /home/mark1/Desktop/autonomous_kart/AutoKart_RC_Dynamics_ESP32S3_v1 && PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -p no:cacheprovider -q tests/test_static.py` | Baseline: 9 passed, exit 0, 0.03 s. Post-doc: 9 passed, exit 0, 0.05 s. |
| Nested WebStop source checks | `cd /home/mark1/Desktop/autonomous_kart/AutoKart_RC_Dynamics_ESP32S3_v1/AutoKart_ESP_IDF_R86mini_WebStop && PYTHONDONTWRITEBYTECODE=1 python3 tests/static_checks.py` | Baseline and post-doc printed PASS for endpoints, pins, pulses, stop paths and embedded HTML. |

Environment: 2026-10-16, Ubuntu laptop host, Python 3.10.12, Node.js 22.23.2, npm 10.9.8, ROS 2 Humble. A ROS `colcon test` result is separate from the direct 10-test contract suite: before documentation, only the simulator bridge ran 2 colcon tests; dashboard gateway ran 0; Gazebo pytest was unavailable. `colcon test` was not rerun after the documentation-only additions.

Revision/provenance at both verification points: `autonomy_stack_phase1` was on `main` at `87f2e288cb1505a9eb233e077603f19689cc3114`; the Phase 1 source directory under test was untracked in that checkout. `kart/` and `phase_sim/` have no Git metadata at their roots. The ESP32 `.git` path is an empty directory, so firmware source is not tied to an identifiable Git revision in this workspace. All 11 camera snapshot branches are listed with their HEAD hashes in `REPOSITORY_DISPOSITION.md`.

## Git and workspace inspection

```bash
find . -maxdepth 1 -mindepth 1 -printf '%y %f -> %l\n' | sort
find . -name .git -print -prune -o -type l -printf 'SYMLINK %p -> %l\n' -print
git -C autonomy_stack_phase1 worktree list --porcelain
git -C autonomy_stack_phase1 status --short --branch
git -C AutoKart_RC_Dynamics_ESP32S3_v1 status --short --branch
```

The workspace root has no Git metadata. Do not move or remove a linked worktree or edit its `.git` pointer. See `docs/decisions/REPOSITORY_DISPOSITION.md` for the approval-gated path list and backup location.

## Not established by this runbook

These checks do not verify a Raspberry Pi runtime, camera depth accuracy, IMU operation, installed firmware, wiring, VCU telemetry, E-stop independence, physical braking, vehicle speed or skidpad completion.

`idf.py` was not present on the host, so no ESP-IDF firmware compile or flash was run. No hardware motion test was attempted.
