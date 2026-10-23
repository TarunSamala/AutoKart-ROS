# Independent validator report — autokart_dashboard_gateway

Performed as a distinct, sequential validation pass: re-ran the reused
package's own existing test suites unmodified, built and tested this
wrapper in an isolated colcon workspace, and exercised the export/
clean-workspace path end to end (including a live launch and HTTP check) —
after the reviewer's perception-path-substitution fix.

## Environment

- ROS 2 Humble, `colcon` present.
- No real browser, no real camera hardware, no live vehicle bridge/safety
  node running in this environment — software-only validation.
- This machine has `autokart_perception` permanently importable via a
  site-wide editable pip install, independent of any path configuration —
  see `reports/SENIOR_REVIEWER_REPORT.md`'s defect writeup and
  `DEPENDENCIES.md`.

## Runs performed and their actual output

1. **Reused package's own existing test suites, run unmodified, read-only**
   (confirms this change did not depend on behavior those suites would
   catch, and that they still pass today):
   - `cd kart/ros2_ws/src/autokart_dashboard_gateway && python3 -m pytest test/test_core.py -q`
     → **12 passed**.
   - `python3 -m pytest test/test_live_gateway.py -q` → **21 passed**.
   - No file under `kart/ros2_ws` was modified to run either.

2. **This package's own test suite, isolated workspace, real dependencies
   resolvable** (`gateway_package_path`/`perception_package_path` pointed at
   the real, in-repo checkouts):
   `cd autokart_ros_nodes/autokart_dashboard_gateway && python3 -m pytest test/ -q`
   → **13 passed** (`test_adapter.py` ×7, `test_node_integration.py` ×4,
   `test_fault_injection.py` ×2).

3. **Reviewer's defect, reproduced and then confirmed fixed**: reverted the
   perception-path-verification check in `adapter.py` locally, re-ran
   `test_load_gateway_wrong_perception_path_is_rejected_not_silently_substituted`
   alone with a deliberately wrong `perception_package_path` → **did not
   raise** (the real gateway module imported successfully anyway, via the
   globally-installed `autokart_perception`, confirming the pre-fix defect
   exists on this machine as described). Restored the fix, re-ran → raised
   `PERCEPTION_DEPENDENCY_PATH_MISMATCH` as expected.

4. **Clean-workspace portability test**, workspace built with only
   `/opt/ros/humble/setup.bash` sourced (neither `kart/ros2_ws` nor
   `autonomy_stack_phase1` on the build/runtime path):
   - `export_node.sh` → tarball containing only `autokart_dashboard_gateway/`
     (no `autokart_msgs` bundled — not needed to build this package; see
     DEPENDENCIES.md).
   - Extracted into a brand-new directory;
     `colcon build --packages-select autokart_dashboard_gateway` →
     `Summary: 1 package finished`, no errors.
   - `ros2 launch autokart_dashboard_gateway dashboard_gateway.launch.py
     config_file:=.../dashboard_gateway_synthetic.yaml` (no dependency paths
     set) → process started; `ros2 topic list` showed no topic from this
     node (only the `ros2` CLI's own transient introspection-node topics);
     `curl http://127.0.0.1:8765/health` returned
     `{"status": "degraded", "gateway_available": false, "reason":
     "DEPENDENCY_PATH_NOT_SET", "vehicle_authority": "OFF",
     "actuator_authority": "NONE"}` — exactly the documented fail-closed
     contract, with **zero ROS node created** in this state (confirmed, not
     just claimed).
   - No leftover `dashboard_gateway_node`/`ros2 launch` process after the
     run — checked and confirmed with `pgrep -af`.

## Acceptance matrix (software-only)

| Capability | Status | Evidence |
|---|---|---|
| Builds as standalone ament_python package, `kart/ros2_ws` not sourced | VALIDATED | isolated-workspace colcon build, develop tree + exported tarball |
| Dependency unavailable → degraded `/health`, zero ROS node, zero publisher | VALIDATED | live launch + curl + `ros2 topic list`; `test_fault_injection.py` |
| Actuation-authority boundary: exactly two publishers, both bounded-whitelist request strings | VALIDATED | `test_node_integration.py::test_gateway_node_has_exactly_two_publishers_both_bounded_requests` (real rclpy graph introspection, not source-reading) |
| No service clients | VALIDATED | `test_node_integration.py::test_gateway_node_has_no_service_clients_either` |
| Demo-drive reverse / out-of-range duration refused | VALIDATED | `test_node_integration.py::test_demo_drive_reverse_is_refused_not_silently_dropped` |
| Perception-path substitution defect (reviewer's fix) | VALIDATED | reproduced pre-fix, confirmed post-fix |
| Reused package's own existing test suites | VALIDATED (unmodified, read-only) | 33/33 passed |
| Data-source labeling never defaults to LIVE | VALIDATED (reused test, re-run) | `test_live_gateway.py::test_safety_envelope_payload_never_defaults_missing_data_source_to_live` |
| Read-only HTTP surface (non-GET/HEAD rejected) | VALIDATED (reused test, re-run) | `test_live_gateway.py`'s 405 assertions |
| Portability: clean workspace, export bundle, build, launch, health check | VALIDATED | full run recorded above |
| No hardcoded project-home paths in source | VALIDATED | grep check |
| `colcon test` standard pytest hook | BLOCKED | same pre-existing `tests_require`/pytest-runner tooling gap already recorded for `autokart_hp60c`/`autokart_safety` in this environment |
| `mode: "live"` against a real running vehicle bridge / safety node / perception stack together | NOT RUN | no hardware, no multi-node live bring-up performed in this session (each of this library's nodes validated independently) |
| Real browser / built Mission Control frontend | NOT RUN | not available in this environment; underlying HTTP/WebSocket surface is tested via the reused package's own `aiohttp` test-client suite instead |
| Real camera hardware | NOT RUN | `enable_camera_feed` defaults false in both shipped profiles |

## Explicit statement

**Software-only acceptance above does not authorize or imply vehicle
motion, and does not grant this node any actuation authority — it has
none.** This gateway only displays what upstream nodes (especially
`autokart_safety`, node 10) already decided, and the one request channel it
can forward is independently re-validated and re-gated by that node, never
acted on directly by this one.
