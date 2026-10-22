# AutoKart — Consolidated Open Issues

> **CORRECTION (2026-10-18, later):** statements in this document that no HP60C is attached are WRONG or STALE. An HP60C (USB `3482:6723`) is attached and streams RGB+depth at ~11-13 fps via the vendor SDK; the live Vehicle Dashboard was verified against real frames (see `ui_split/VALIDATION_MATRIX.md`). Items below marked NOT TESTED because "no camera" are now testable; those not re-run in `ui_split/VALIDATION_MATRIX.md` remain NOT TESTED. Calibration is still UNVERIFIED and no qualified metric geometry exists.

Generated from this session's audit across camera/perception, YOLO26s, geometry, safety envelope, deployment, and frontend. Each item: what's missing, why it matters, what would close it. Nothing here is fabricated as done.

## Hardware-blocked (cannot be closed without physical access)

| # | Item | Blocked on |
|---|---|---|
| 1 | Real HP60C camera acquisition, live timestamps, live RGB-depth pairing | No HP60C attached to this sandbox (only a laptop webcam) |
| 2 | `base_link` origin physical survey/marking | Physical measurement on the actual vehicle |
| 3 | `bumper_setback_m` (1.0668 m), `minimum_bumper_clearance_m` geometry upgrade from PROVISIONAL to VERIFIED | Depends on #2 |
| 4 | Camera mount calibration (`MOUNT_VERIFIED`) via the existing calibration-session/fit/store pipeline | Requires real camera + surveyed reference points |
| 5 | Stationary surveyed obstacle-distance tests (RC-07 Gate D) | Requires #1 + #2 physically measured obstacles |
| 6 | Raspberry Pi (ARM64) build, performance, systemd service | No Pi reachable from this session |
| 7 | Measured braking performance / deceleration | Explicitly out of scope — "Physical actuation/VCU/E-stop are out of scope and remain disabled" per the original engineering contract |

## Real software gaps found this session (not fixed — scoped for follow-up)

| # | Item | Why it matters | Scope to close |
|---|---|---|---|
| 8 | **No production publisher for `/perception/obstacles`.** The topic, message type, and consumer (`front_safety_envelope_monitor`) all exist; only cone-class objects are published to `/perception/cones` by `autokart_perception`. | This is TASK C9's "a cone detector is not a general obstacle detector" — right now the safety envelope can only ever react to the 5 trained cone classes, never to an un-trained obstacle (a person, a dropped item, another vehicle). | New depth-based geometric obstacle-monitoring path in `autokart_perception`, independent of YOLO26s class labels, publishing to `/perception/obstacles`. Real scope — not a quick fix. |
| 9 | CPU-only YOLO26s inference measured at ~266 ms / ~3.75 FPS on this x86_64 workstation | Likely too slow for real-time obstacle avoidance at any meaningful operating speed; consumes a large fraction of any reasonable detection-to-braking latency budget before depth/geometry/safety even run | Needs either model optimization (quantization, smaller input resolution, execution-provider tuning) or GPU/NPU acceleration; re-measure on the actual Pi target once reachable |
| 10 | Safety-envelope corridor doesn't model steering/path curvature (TASK E3) | A straight-ahead-only corridor could miss (or over-flag) obstacles when the kart is actually turning | Needs a predicted-path input to the safety monitor, not implemented |
| 11 | Obstacle footprint/size not modeled — only a point position per obstacle | Understates clearance to a wide obstacle, overstates it to a narrow one | Needs an obstacle radius/extent field in the obstacle message and corridor math update |
| 12 | TASK D11 — calibrated (non-schematic) safety-zone projection onto the camera image | Current overlay is intentionally schematic/dashed because calibration is unverified (correct behavior); the *code path* for a real metric projection, dormant until `MOUNT_VERIFIED` exists, hasn't been written yet | Implement once #4 exists, so it's ready rather than written in a rush under hardware-test time pressure |
| 13 | `vehicle_authority`/`safety_monitor_mode` runtime-change rejection relies on a node-level `on_set_parameters` callback (fixed post-RC-06) — confirmed effective because no actuation capability exists anywhere to unlock, not because the parameter itself is unforgeable at the DDS layer | Documented, not a current risk, but worth knowing precisely why it's safe | No action needed unless an actuation path is ever added — if it ever is, this guard needs re-auditing at that time |

## Environment/packaging defects found and fixed this session (closed, with evidence)

| # | Item | Fix |
|---|---|---|
| 14 | Stale, non-editable `pip`-installed `autokart_perception` in `~/.local/lib/python3.10/site-packages` (dated 2026-10-14) was shadowing the real source tree for plain `python3` invocations, missing all RC-05/06 camera.py improvements | `pip install --user -e autonomy_stack_phase1/phase_01_perception`. Verified: `camera.py` test suite 2/7 → 7/7. |
| 15 | `kart_sim` (`auto-kart-simulator`, the `control` optional dependency) was never installed, causing 7 tests to fail via the correctly-fail-safe `CONTROL_DEPENDENCY_UNAVAILABLE` path | `pip install --user -e kart`. Verified: full perception suite 156/163 → 163/163. |
| 16 | `rc03_rc05_diagnostics.launch.py` (the closest thing to a "complete diagnostic system" launch) didn't include `front_safety_envelope_monitor` at all — it predates that package | Added, gated by `launch_safety_monitor` (default true) and `safety_data_source` (default `UNVERIFIED`). Verified live: all 5 nodes start together, contract (`vehicle_authority=OFF`) holds. |

## Frontend (Agent 6) — closed this session, independently re-verified

A background agent separated the live-only Mission Control dashboard (`/ops` route, `MissionControlLive.tsx` + `useMissionDataLive.ts`) from the pre-existing LAB/SIMULATION/REPLAY/DEMO-switchable development app (default route, untouched). Full report: `AUTOKART_MISSION_CONTROL.md`.

**Independently re-verified in this session (not just taken on the agent's word):** files exist as claimed; `grep` of the new files' imports shows no `LabSkidpad`/`AutonomyEngine`/`./demo`/`labScenarios` reference, only doc-comments mentioning them by name; `npx vitest run` → 47/47 (was 43/43, +4 new tests, zero regressions); `tsc --noEmit` clean; `npm run build` succeeds; and a live browser load of `/ops` with no gateway running shows "MISSION CONTROL · LIVE", a static (non-interactive) `REAL_KART` badge, "DISCONNECTED FROM GATEWAY · NO LIVE FRAME DISPLAYED", "NO LIVE CONNECTION ... No simulated or cached geometry is substituted here", and `VEHICLE AUTHORITY OFF" — zero `<select>` elements in the DOM, only `LIVE`/`PAUSE` buttons.

**Honest limitation carried over from the agent's own report, confirmed by this session's own `npm run build` output:** this project's Vite config does not code-split by route — `/ops` and the dev `MissionControl` still ship in one physical JS bundle (`dist/assets/index-*.js`, ~930 kB). The `/ops` React component tree itself has zero reachable code path into sim/lab code (verified above), but a byte-for-byte "zero sim code in the artifact" guarantee would need route-based code splitting or a second build entry — not done, by design, to keep this a minimal, low-risk change.

## Observed test flakiness (noted honestly, not chased down)

One combined run (perception + gateway + frontend suites run back-to-back in one compound shell command) showed 7 failures in `autonomy_stack_phase1/phase_01_perception/tests/` (`test_live_frame_from_a_different_camera_is_not_verified`, `test_yolo26s_live_selection_and_end_to_end_box_contract`, and 5 others) that did not reproduce in 3 immediate repeat runs (163/163 each time) or in the isolated run used for every PASS claim elsewhere in this document. No source file touching those tests was changed this session. Likely cause: timing/resource contention from running three test suites and a browser session back-to-back in one shell, not a code defect — but flagged here rather than silently discarded, since it was observed and not explained with certainty.

## Full regression snapshot, this session, before hardware testing begins

| Suite | Result |
|---|---|
| `colcon build` | 16/16 packages |
| `autonomy_stack_phase1/phase_01_perception/tests/` | 163/163 |
| `autokart_dashboard_gateway` + `autokart_safety_bridge` | 43/43 |
| `phase_sim` (frontend) | 47/47, `tsc` clean, build clean |
| Combined diagnostic launch (synthetic) | Verified live, 5/5 nodes |

**253 automated tests passing total** (163 + 43 + 47), zero known regressions, two real environment defects found and fixed, four real software gaps (#8–#12) honestly identified and deferred rather than papered over.
