# Test matrix (RC-02)

## RC-03–05 execution evidence (2026-10-16, host ROS Humble)

The untouched test baseline was rerun before source edits and saved in
`docs/rc03_rc05_evidence/baseline_*.log`. The initial baseline colcon build
started before edits but overlapped them, so it is **not** used as an untouched
build comparison. The prior RC-02 report is the 16-package baseline; a later
fresh `/tmp/autokart_rc03_clean_*` build verified the final source.

| Check | Before | After | Evidence |
|---|---:|---:|---|
| Phase 1 perception/shared core pytest | 139 passed | 140 passed | `baseline_perception.log`, `post_perception.log`; new simulated crossover regression. |
| Kart pytest | 150 passed | 150 passed | `baseline_kart.log`, `post_kart.log`; launch inventory assertion updated for the sixth file. |
| Existing ROS/gateway contracts | 10 passed | 11 passed | `baseline_ros_contracts.log`, `post_ros_contracts.log`. |
| LAB Vitest | 32 passed | 32 passed | `baseline_lab.log`, `post_lab.log`. |
| LAB frontend build | passed | passed | `post_lab_build.log`; existing bundle-size advisory only. |
| Firmware static pytest | 9 passed | 9 passed | `baseline_firmware_static.log`, `post_firmware_static.log`. |
| Nested WebStop static | pass | pass | `post_webstop_static.log`. |
| ROS packages built | 16 | 16 | `post_colcon_list.log`, `clean_colcon_build.log`, final incremental build logs. |
| New adapter pytest | absent | 11 passed | `focused_adapters.log`; includes missing model, map invalidation, 4-lap, brake segment and wrong-color cases. |
| Colcon registered tests | prior RC-02 suite | 18 test result entries, zero failures/skips | `verified_full_test.log`, `verified_full_test_result.log`; `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` avoids the host's incompatible anyio pytest plugin. |

The runnable ROS evidence is under `docs/rc03_rc05_evidence/`:

The final fresh `verified_full_build.log` built 16 packages. The added
`ament_cmake` export in `autokart_bringup/package.xml` lets a sourced overlay
discover its launch files with `ros2 pkg prefix autokart_bringup`. All three
new adapter executables were also discovered with `ros2 pkg executables`.

- `replay_launch.log`: `ros2 launch autokart_perception perception_replay.launch.py replay_dir:=.../recordings/final-smoke-20261008` exited 0 after two frames. These are legacy **synthetic** recordings with empty calibration provenance; replay correctly marked both frames degraded and published no qualified metric cones.
- `ros_probe_result.log`: synthetic RC-03 cones plus explicitly simulated odometry reached RC-04 and RC-05; incomplete loops were held, and the gateway API exposed the actual stage reasons.
- `ros_skidpad_result.log`: a simulated **qualified cone fixture**, 164 time-stamped ROS samples, exactly two right/two left laps, transition and feedback-gated STOP; 160 diagnostic path samples. This is not a camera or vehicle run.
- `ros_braking_result.log`: a simulated obstacle between sparse centerline waypoints caused `CORRIDOR_BLOCKED`, zero valid planner samples and a gateway diagnostic reason. Some pair timeouts also produced invalid HOLD, as designed.

All probes used isolated ROS domain IDs and explicitly simulated odometry.
No real Pi timing, surveyed camera geometry, wheel/IMU odometry, VCU feedback,
physical brake or independent stop was measured. The G3 adversarial matrix is
not complete: dropout/noise, TF, restart/reconnect, delayed feedback, full
dashboard browser inspection and physical stop remain open.

## Historical RC-02 package-scaffold record

The section below records the earlier RC-02 package-scaffold stage. Its
post-change counts and "no executables" statement describe that stage, not
the RC-03–05 implementation and current results above.

Baseline was run before RC-02 repository edits on 2026-10-16. The checks below validate that earlier untouched source baseline. Structural package edits were followed by a fresh colcon build and relevant regressions; those earlier counts are reported separately.

| Requirement / suite | Baseline command | Baseline result | Limitation |
|---|---|---|---|
| Phase 1 perception/core | `cd autonomy_stack_phase1/phase_01_perception && PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src:/home/mark1/Desktop/autonomous_kart/kart python3 -m pytest -p no:cacheprovider -q tests` | Baseline **139 passed**; post-change **139 passed** | Source tree is untracked in the nested `autonomy_stack_phase1` Git checkout. |
| Kart simulator, Pure Pursuit and ROS layout contracts | `cd kart && PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=. python3 -m pytest -p no:cacheprovider -q tests` | Baseline **149 passed**; post-change **150 passed** | One new test verifies grouped package discovery and uniqueness. Simulation evidence only. |
| Existing ROS contracts | `cd kart && PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=.:ros2_ws/src/autokart_dashboard_gateway:ros2_ws/src/autokart_sim_bridge python3 -m pytest -p no:cacheprovider -q ros2_ws/src/autokart_dashboard_gateway/test ros2_ws/src/autokart_sim_bridge/test ros2_ws/src/autokart_gazebo/test` | Baseline **10 passed**; post-change **10 passed** | Gazebo pytest is unavailable in the colcon environment; direct suite uses host pytest. |
| ROS workspace build | `cd kart/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --base-paths src --build-base /tmp/autokart_nodes_final_build --install-base /tmp/autokart_nodes_final_install` | Baseline **13 packages built**; post-change **16 packages built** | Existing CMake/setuptools warnings; no hardware runtime. |
| LAB UI | `cd phase_sim && npm run test -- --run` | Baseline and post-change **8 files / 32 tests passed** | UI tests use fixtures; not a live ROS graph. |
| LAB build | `cd phase_sim && npm run build` | Baseline and post-change passed, **53 modules** | Existing 905.32 kB JavaScript bundle advisory. |
| ESP32 source static checks | `cd AutoKart_RC_Dynamics_ESP32S3_v1 && PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -p no:cacheprovider -q tests/test_static.py` | Baseline and post-change **9 passed** | No firmware compile or flash. |
| Nested WebStop static checks | `cd AutoKart_RC_Dynamics_ESP32S3_v1/AutoKart_ESP_IDF_R86mini_WebStop && PYTHONDONTWRITEBYTECODE=1 python3 tests/static_checks.py` | Baseline and post-change **PASS** | Static source checks only. |

After build, a sourced ROS overlay imported `autokart_perception` from the ROS install space and `autokart_perception_ros` from the wrapper package. SHA-256 comparisons matched the installed `__init__.py`, `shared_core.py`, and `geometry.py` to the Phase 1 `src/` copies. The full kart suite first exposed the old package path in its layout assertion; that assertion was updated to the new grouped package locations, then the complete suite passed 150 tests.

## After structural changes

The three new adapter packages and relocated perception package are importable package scaffolds; no ROS node executables, topics, or command publishers are installed. The master-plan adapter round-trip, replay/fault, mission, controller and dashboard integration tests have not yet been implemented or run.
