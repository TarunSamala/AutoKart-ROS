# Dashboard / Lab Split — Validation Matrix (2026-10-18)

Verdicts: PASS / FAIL / BLOCKED / NOT TESTED. "I ran" = run by the orchestrator in this session. "Agent" = reported by a subagent with evidence, not independently re-run.

## Separate verdicts (spec §10.9)

| Scope | Verdict |
|---|---|
| **UI separation** (two independent apps) | **PASS** — separate entries, builds, ports, processes, ROS domains; isolation verified both ways on the real ports |
| **Physical RGB+depth path** (HP60C → SDK → perception service → browser) | **PASS for display; NOT qualified for metrics** — real frames shown at ~11–13 fps, calibration UNVERIFIED, zero qualified cones |
| **Lab algorithm** | **PASS for what exists** — Python CORE (`/core`) live and cross-checked; 4 pages honestly "not implemented"; 3 partial |
| **Vehicle autonomy** | **NO-GO** — no independent stop/steering/braking physical proof exists; out of scope |

## Tests

| ID | Result | Evidence |
|---|---|---|
| UI-01 Dashboard-only launch | PASS (I ran) | `run_dashboard.sh start` with lab stopped; `smoke_dashboard.sh` 6/6 |
| UI-02 Lab-only launch | PASS (I ran) | `run_lab.sh start` with dashboard stopped; `smoke_lab.sh` 8/8 |
| UI-03 Both simultaneously | PASS (I ran) | both stacks up on 5180/5181/8765/8767/8768/8775, `status.sh` all `yes` |
| UI-04 Kill Lab → Dashboard | PASS (I ran) | lab stopped, dashboard smoke PASS |
| UI-05 Kill Dashboard → Lab | PASS (I ran) | dashboard stopped (ports closed), lab smoke PASS |
| UI-06 Dashboard deep links | PASS (jsdom: all 10 routes, Agent; I ran `/perception`, `/camera` in a browser; smoke `/overview` 200) | `dashboard.test.tsx` 24/24 |
| UI-07 Lab deep links | PASS (jsdom: every route, Agent; `/home` 200 in smoke; dev-mode `/mission` direct load 200 after my proxy fix) | `labApp.test.tsx` 34/34 |
| UI-08 Button audit | PARTIAL — dashboard has no command buttons (tested); Lab pages are routes or honest disabled/not-implemented states; no exhaustive click-through run |
| CAM-01 SDK startup, changing frame IDs | PASS (I ran) | live perception: sequence advanced #2→#939+, `camera_id HP60C`, `source LIVE`; dashboard frame counter rose at ~10.7–17.6 Hz |
| CAM-02 Depth scale/units | PARTIAL — shows `0.001 m/raw, UNQUALIFIED, VENDOR_ROS_DIVIDE_BY_1000_UNSURVEYED`; scale NOT verified against a survey |
| CAM-03 RGB-depth pairing | PASS for pairing metadata (`SDK_PAIRED_CALLBACK`, matching sequences); failure counting only unit-tested |
| CAM-04 Unplug device | NOT TESTED — no physical unplug. Simulated via missing bridge library → `SDK_ERROR`, server stays up, no synthetic frames (Agent) |
| CAM-05 Reconnect / single SDK owner | PASS for single owner (I ran: one live process; deploy script refuses a 2nd; camera released on stop). Hot-replug NOT TESTED |
| ROS-01 Topic/QoS audit | PARTIAL — node/topic actuation audit PASS (I ran); QoS compatibility matrix not re-run |
| ROS-02 ROS node crash | NOT TESTED against a running graph; jsdom shows ROS_OFFLINE/gateway-down states |
| ROS-03 Gateway unavailable | PASS (jsdom, Agent): perception-down names that hop while ROS renders, and the reverse |
| SRC-01 Synthetic provenance | PASS — `--dev-sim` shows SIMULATED banner and stamps the safety monitor SIMULATED (fixed this session); sim backend → red "NOT A PHYSICAL CAMERA" banner (browser, Agent) |
| SRC-02 Missing calibration | PASS (I ran) — real preview visible, "NO QUALIFIED LIVE GEOMETRY", metric layers withheld |
| LAB-01 Deterministic scenario | PARTIAL — CORE vs CLI equality exists from 2026-10-13 (`autokart_apps/VALIDATION.md`); not re-run here |
| LAB-02 Replay seek | NOT TESTED |
| LAB-03 Skidpad 2R+2L fixture | NOT TESTED in the new Lab (existing fixtures untouched) |
| LAB-04 Braking fixture | NOT TESTED in the new Lab |
| SAFE-01 Actuation audit | PASS (I ran) — no node publishes actuation topics; `/control/motion_command` and `/vcu/state` have 0 publishers; `vehicle_authority=OFF`, `DIAGNOSTIC_ONLY` |
| SAFE-02 Lab fault injection isolation | NOT APPLICABLE — fault injection page is not implemented; Lab ROS runs in domain 42 and its UI cannot reach live services (smoke PASS) |
| SAFE-03 Stale/unknown never CLEAR | PASS (jsdom, Agent); unverified/non-LIVE CLEAR shown as "CLEAR · UNVERIFIED" |
| PERF-01 10-minute soak | NOT TESTED |
| OPS-01 Restart ×5 | PARTIAL — 3 restarts on real ports, smoke PASS each, exactly one camera owner, none left after stop; 5 not run |
| OPS-02 Rollback | NOT TESTED — procedure in `ROLLBACK.md`, backup in `_rollback/ui_split_20261018T150757Z/` |

## Automated totals (I ran)
- Frontend `npx vitest run`: **105/105** (was 47); `tsc -p tsconfig.app.json` clean (note: `tsc -p .` checks nothing because `tsconfig.json` has `files: []`); `build:dashboard` and `build:lab` pass.
- Perception `pytest`: **172/172**.
- Bundle isolation: dashboard bundle contains no `LabSkidpad`/`AutonomyEngine` (two `REAL_KART` label strings only); Lab bundle only mentions live ports in explanatory text.
