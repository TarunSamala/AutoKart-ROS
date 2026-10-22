# Independent validator report — autokart_system_monitor

A distinct, sequential validation pass, run after both of the reviewer's
fixes. It covers an isolated-workspace build and tests, pre-fix
reproductions, a clean exported workspace, a live launch, and a live
cross-node fault injection with `autokart_safety`.

## Environment

- ROS 2 Humble, `colcon`, and `psutil` 5.9.0 are installed.
- No Raspberry Pi and no camera are attached; this is software-only
  validation.
- `kart/ros2_ws` was not used or modified. Its `install/` is stale relative
  to its `autokart_msgs` source (`MissionDebug` and `BenchVcuStatus` are
  missing), so node 10's runs below use a fresh scratch copy of
  `autokart_msgs`. This node does not need `autokart_msgs` at all.

## Runs and actual output

1. **Isolated scratch workspace** (fresh `autokart_msgs` copy plus nodes 10,
   11 and 12): `colcon build` gave `4 packages finished`. Running
   `pytest test/ -q` in this package gave **21 passed**.
2. **Pre-fix reproductions** (on scratch copies; originals untouched):
   - QoS defect: the best-effort publisher test **failed**, and ROS logged
     `incompatible QoS ... RELIABILITY`. Post-fix it passes.
   - Self-heartbeat defect: with the guard disabled, watching `/diagnostics`
     with no other publisher reported `fresh`. Post-fix the topic is
     rejected, and the new test passes.
3. **Clean-workspace portability.** This was re-run after the final code
   change, with only `/opt/ros/humble/setup.bash` sourced:
   - `export_node.sh` produced a tarball containing only
     `src/autokart_system_monitor/` and `README_EXPORT.md`.
   - Extracted into a new directory, `colcon build --packages-select
     autokart_system_monitor` gave `1 package finished`.
   - `pytest test/ -q` from inside the extracted copy gave **21 passed**.
   - `ros2 launch autokart_system_monitor system_monitor.launch.py` →
     `ros2 topic list` showed `/diagnostics`, `/system/health` and
     `/system/startup_status`. `ros2 topic echo /system/health --once` showed
     both default heartbeat topics as `never seen` (nothing else was
     running) and `resources: within limits` (real psutil).
4. **Live cross-node check and fault injection** (an earlier clean build of
   this package together with a clean exported build of `autokart_safety`;
   the QoS fix was already in place, and the later self-heartbeat guard does
   not affect this path):
   - Both nodes were launched. `/system/health` showed
     `/autokart/safety/status: fresh` (OK), `/hp60c/status: never seen`
     (STALE) and `resources: within limits`. `/system/startup_status`
     showed `/autokart/safety/status: seen at least once`.
   - The safety node process was then killed. `/system/health` changed to
     `/autokart/safety/status: stale: no message within timeout` (level 3),
     while `/system/startup_status` stayed at `seen at least once`. This is
     the brief's required scenario — a dependency that stops publishing must
     show up as stale, not silently look fine — demonstrated live, not only
     in a unit test.
5. **Cleanup:** `ps`/`grep` (with bracket patterns, so the search could not
   match itself) confirmed that none of this session's node or launch
   processes remained. A sibling agent's `/autokart_controller` node was
   seen on the graph and was deliberately left alone.

## Acceptance matrix (software-only)

| Capability | Status | Evidence |
|---|---|---|
| Builds standalone, no project dependency | VALIDATED | clean export build, only `/opt/ros/humble` sourced |
| `/diagnostics`, `/system/health`, `/system/startup_status` as standard `DiagnosticArray` | VALIDATED | live `ros2 topic list`/`echo` |
| Works standalone with no dependency running | VALIDATED | integration test + live launch |
| Dependency stops publishing → STALE | VALIDATED | integration test + live kill of `autokart_safety` |
| Startup latch never un-latches | VALIDATED | integration test + live |
| Best-effort publisher detected (QoS defect fix) | VALIDATED | reproduced pre-fix, passes post-fix |
| Own outputs rejected as heartbeats (self-evidence fix) | VALIDATED | reproduced pre-fix, passes post-fix |
| psutil present → real CPU/RAM; missing → WARN/null, no crash | VALIDATED | unit + node-level fault injection; live `within limits` |
| No safety authority / no command publisher | VALIDATED | grep + publisher set (three diagnostic topics only) |
| `colcon test` standard hook | BLOCKED | the same pre-existing `tests_require` tooling gap recorded for `autokart_hp60c`; pytest was run directly instead |
| CPU/RAM on target Raspberry Pi; threshold tuning | NOT RUN | no Pi in this environment |
| Heartbeats from `autokart_hp60c` and other real nodes | NOT RUN | only `autokart_safety` was run alongside; `/hp60c/status` was correctly "never seen" |

## Explicit statement

This node reports; it does not decide. Software-only acceptance here does
not authorize or imply vehicle motion, and nothing in this package touches
any `*_verified` flag.
