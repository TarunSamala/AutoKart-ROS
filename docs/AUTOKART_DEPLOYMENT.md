# AutoKart Deployment — Integration & Launch (Agent 7)

> **CORRECTION (2026-10-18, later):** statements in this document that no HP60C is attached are WRONG or STALE. An HP60C (USB `3482:6723`) is attached and streams RGB+depth at ~11-13 fps via the vendor SDK; the live Vehicle Dashboard was verified against real frames (see `ui_split/VALIDATION_MATRIX.md`). Items below marked NOT TESTED because "no camera" are now testable; those not re-run in `ui_split/VALIDATION_MATRIX.md` remain NOT TESTED. Calibration is still UNVERIFIED and no qualified metric geometry exists.

## Environment reality

This work was done on `Jarvis`, an x86_64 Ubuntu workstation with ROS 2 Humble — **not the Raspberry Pi ARM64 target**. No Raspberry Pi was reachable from this session. Everything below is x86_64-verified; the ARM64/Pi-specific items (TASK G1 OS dependency audit on actual Pi, TASK G6 performance on actual Pi, TASK G7 systemd service on actual Pi) are **NOT TESTED** and tracked as open issues, not fabricated.

## Launch targets (real, tested this session)

| Target | File | Status |
|---|---|---|
| Camera-only / perception-only / replay | `autokart_nodes/autokart_perception/launch/perception_replay.launch.py` | Pre-existing |
| Sim bridge | `autokart_bringup/launch/fast_sim.launch.py`, `full_stack.launch.py` | Pre-existing |
| Cone track / skidpad / gazebo scenarios | `autokart_bringup/launch/{cone_track,skidpad,gazebo_sim}.launch.py` | Pre-existing |
| **Perception + mapping + planner + controller + safety diagnostics + optional gateway** | `autokart_bringup/launch/rc03_rc05_diagnostics.launch.py` | **Updated this session** — added `front_safety_envelope_monitor` (previously missing from this launch entirely; the safety monitor package didn't exist when this launch file was first written) |
| Safety monitor standalone | `autokart_safety_bridge/launch/front_safety_envelope_monitor.launch.py` | Pre-existing (this session's earlier RC-05 work) |

**No launch file combines camera + perception + safety + gateway into a true single hardware-diagnostic command yet** — `rc03_rc05_diagnostics.launch.py` is the closest (now includes the safety monitor), but still needs explicit `camera_library`/`camera_configuration`/`model_path`/`model_manifest` arguments to run `perception_mode:=live`, none of which point at a connected HP60C here, so that specific combination remains structurally ready but **NOT TESTED against real hardware**.

## `rc03_rc05_diagnostics.launch.py` — what changed and why

Added `front_safety_envelope_monitor` (package `autokart_safety_bridge`), gated by a new `launch_safety_monitor` argument (default `true`), loading the same `front_safety_envelope_monitor.yaml` config the standalone launch uses, with a new `safety_data_source` launch argument wired to the node's `data_source` parameter (default `UNVERIFIED` — never silently `LIVE`; see `AUTOKART_SAFETY_ENVELOPE.md` / the RC-06 addendum for why). Added `autokart_safety_bridge` as an `exec_depend` in `autokart_bringup/package.xml`.

**Real verification, not just argument parsing:**

```
$ ros2 launch autokart_bringup rc03_rc05_diagnostics.launch.py \
    perception_mode:=synthetic safety_data_source:=SIMULATED
$ ros2 node list
/autokart_controller
/autokart_mapping
/autokart_perception
/autokart_planner
/front_safety_envelope_monitor        <- now present
$ ros2 param get /front_safety_envelope_monitor data_source
String value is: SIMULATED            <- correctly propagated from the launch argument
$ ros2 param get /front_safety_envelope_monitor vehicle_authority
String value is: OFF                  <- non-negotiable contract intact
```

All 5 nodes started together; `vehicle_authority` held at `OFF`; `data_source` took the launch-time value rather than defaulting to `LIVE`. The `rc03_rc05` launch's own docstring already states "Odometry remains an external prerequisite" — in `perception_mode:=synthetic` with no external odometry supplied, `perception_node` logs `status=DEGRADED reasons=('INVALID_POSE',)` repeatedly; this is pre-existing, documented, correct fail-safe behavior given the missing prerequisite, not a defect introduced here.

## Two environment/packaging defects found and fixed this session (see `AUTOKART_YOLO26S_INTEGRATION.md` for the full writeup)

1. A stale, non-editable `pip`-installed copy of `autokart_perception` in `~/.local/lib/python3.10/site-packages` was shadowing the real source tree for any plain `python3 -c "import autokart_perception"` invocation — reinstalled editable (`pip install --user -e autonomy_stack_phase1/phase_01_perception`).
2. The `control` optional dependency (`kart_sim`, package name `auto-kart-simulator`, living at `kart/`) had never been installed — reinstalled editable (`pip install --user -e kart`).

Both are one-time environment setup steps a real Pi deployment runbook must include; they are not source-code changes.

## Regression, full picture, this session

| Suite | Result |
|---|---|
| `colcon build` (16 ROS2 packages) | **16/16 build** |
| `autonomy_stack_phase1/phase_01_perception/tests/` | **163/163 PASS** |
| `autokart_dashboard_gateway` + `autokart_safety_bridge` (incl. rclpy-based node-contract tests) | **43/43 PASS** |
| `phase_sim` vitest + `tsc` + build | **43/43 PASS**, clean |
| Combined diagnostic launch (`rc03_rc05_diagnostics.launch.py`, synthetic mode) | **Verified live** — all 5 nodes start, contract holds |

## Open issues (explicit, not fabricated as PASS)

| Item | Status |
|---|---|
| Raspberry Pi ARM64 build/run | **NOT TESTED** — no Pi reachable from this session |
| `launch_camera_live` end-to-end with real HP60C via `rc03_rc05_diagnostics.launch.py perception_mode:=live` | **NOT TESTED** — no camera attached |
| systemd startup service (TASK G7) for the Pi | **NOT IMPLEMENTED** — deferred until Pi access exists; writing a service file against an unverified target would risk encoding wrong assumptions (device paths, user, working directory) that only a real Pi session can confirm |
| `ros2 doctor`, TF tree validation on the real combined graph | **NOT TESTED** — requires odometry/TF publishers this launch doesn't supply standalone |
| CPU/RAM/frame-rate measurement under full combined launch | **NOT TESTED** this pass — see `AUTOKART_YOLO26S_INTEGRATION.md` for the one latency number that was measured (YOLO26s inference only, this x86_64 workstation, not the Pi) |
