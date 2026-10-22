# RC-06 — Final Integration & Regression Acceptance Report

> **CORRECTION (2026-10-18, later):** statements in this document that no HP60C is attached are WRONG or STALE. An HP60C (USB `3482:6723`) is attached and streams RGB+depth at ~11-13 fps via the vendor SDK; the live Vehicle Dashboard was verified against real frames (see `ui_split/VALIDATION_MATRIX.md`). Items below marked NOT TESTED because "no camera" are now testable; those not re-run in `ui_split/VALIDATION_MATRIX.md` remain NOT TESTED. Calibration is still UNVERIFIED and no qualified metric geometry exists.

**Scope:** Front safety envelope monitor (DIAGNOSTIC_ONLY) → `autokart_dashboard_gateway` → Mission Control browser dashboard.
**Date (UTC):** 2026-10-18, 09:50–10:11 run window. Addendum below: 10:20–10:28.
**Auditor:** Claude (this session), acting as integrator/QA for its own RC-05/RC-06 changes.
**Verdict up front:** **CONDITIONAL PASS — software integration only.** Full acceptance is **NOT** declared. Several required checks (marked NOT TESTED below) could not be executed in this sandbox and remain open. No vehicle actuation exists in this stack; that has been independently re-confirmed in this audit, not merely re-asserted.

---

## Addendum (2026-10-18, 10:20–10:28Z): RC-06 audit findings A and B closed

The engineering review of this report raised two findings before hardware validation could begin. Both are now fixed, tested, and re-verified live. This addendum does not change the verdict in §7 — hardware-in-the-loop is still NOT TESTED — it closes the two software findings that review flagged.

### Finding A — runtime `vehicle_authority`/`safety_monitor_mode` changes are now rejected

`front_safety_envelope_monitor` now registers an `on_set_parameters` callback (`node.py`) that rejects any attempt — at startup via a launch override, or later via `ros2 param set` — to change `vehicle_authority` away from `"OFF"` or `safety_monitor_mode` away from `"DIAGNOSTIC_ONLY"`. Live re-verification, same as the original audit's probe:

```
$ ros2 param set /front_safety_envelope_monitor vehicle_authority ARMED
Setting parameter failed: vehicle_authority is a diagnostic-only, non-negotiable constant;
it cannot be changed from 'OFF' at runtime
$ ros2 param get /front_safety_envelope_monitor vehicle_authority
String value is: OFF

$ ros2 param set /front_safety_envelope_monitor safety_monitor_mode ACTIVE
Setting parameter failed: safety_monitor_mode is a diagnostic-only, non-negotiable constant;
it cannot be changed from 'DIAGNOSTIC_ONLY' at runtime
$ ros2 param get /front_safety_envelope_monitor safety_monitor_mode
String value is: DIAGNOSTIC_ONLY

$ ros2 param set /front_safety_envelope_monitor minimum_bumper_clearance_m 1.5
Set parameter successful    # unrelated parameters remain changeable, as intended
```

The effective value is confirmed unchanged after each rejected update (full transcript: `followup_rejection.txt`). A launch-time override is now rejected even earlier than before — at `declare_parameter`, raising `rclpy.exceptions.InvalidParameterValueException` — because the callback is registered before any parameter is declared. `main()` now also catches that exception type for the same FATAL-log-and-exit(1) refusal-to-start behavior.

New test: `autokart_safety_bridge/test/test_node_contract.py` — 10 tests, instantiating the real `rclpy` node (not a text-contract check) to exercise the callback directly: rejects both protected parameters at runtime, confirms the value is unchanged after rejection, confirms unrelated parameters still work, and confirms the same two parameters are also rejected at declare-time via `parameter_overrides`. **10/10 PASS.**

### Finding B — synthetic data no longer self-reports as indistinguishable from a live physical sensor

`SafetyEnvelope.msg` gained a `string data_source` field. The monitor node requires it to be set explicitly to one of `LIVE` / `SIMULATED` / `REPLAY`; anything missing or unrecognized publishes as `UNVERIFIED` — **the field never defaults to `LIVE`.** The dashboard gateway passes it through (`safety_envelope_payload()`), and `RosLivePanel.tsx` now shows a `DATA SOURCE` row and a visible `mc-alert` banner — both in the text panel and as a red label inside the schematic camera overlay itself — whenever `data_source` is not `LIVE`, or does not match the dashboard's own selected upstream (`LIVE` for `REAL_KART`, `SIMULATED` for `SIMULATION`). A live DDS connection is explicitly documented (in the `.msg` file, the gateway, and the TS types) as not proof of a physical sensor — this field is a self-reported, auditable declaration, not a cryptographic guarantee; that limitation is stated plainly rather than implied as solved.

Live re-verification: restarted the monitor with `-p data_source:=SIMULATED` —

```
$ ros2 topic echo /safety/envelope --once | grep data_source
data_source: SIMULATED
```

— correctly propagated end to end, matching what was configured, not silently reported as LIVE.

New/updated tests, all passing:
- `test_live_gateway.py`: `safety_envelope_payload()` never defaults a missing or garbage `data_source` to `LIVE` (2 new tests; existing 2 updated for the new field). **20/20 PASS** (gateway suite total).
- `rosLivePanel.test.tsx`: UNVERIFIED-by-default banner, REAL_KART-dashboard-receives-SIMULATED mismatch banner, and the no-warning case when `data_source` is genuinely `LIVE` on a matching dashboard (3 new tests). **43/43 PASS** (phase_sim suite total).

The RC-06 synthetic test harness used in the original run (§3–4) was itself an instance of this exact problem — it had self-reported `"source": "LIVE"` to produce a demo screenshot. That was wrong and is called out here rather than quietly fixed; a corrected harness would report `SIMULATED`.

### Full regression after both fixes

| Suite | Result |
|---|---|
| `autokart_safety_bridge` (`test_envelope.py` + new `test_node_contract.py`) | **23/23 PASS** |
| `autokart_dashboard_gateway` (`test_core.py` + `test_live_gateway.py`) | **20/20 PASS** |
| `phase_sim` vitest full suite | **43/43 PASS** |
| `tsc --noEmit` | PASS, zero errors |

**86/86 total**, zero regressions in pre-existing tests. File checksums for this addendum's changes: `followup_checksums.txt`.

### What this addendum does not do

RC-07 (real HP60C hardware, real YOLO26s, physical calibration, stationary obstacle-distance tests) was not attempted — this sandbox has no camera. `vehicle_authority: OFF`, `mode: DIAGNOSTIC_ONLY`, `physical_vehicle_motion: PROHIBITED` remain unchanged and are now additionally enforced against runtime tampering. No perception, mapping, planning, or controller algorithm was modified; no VCU/brake/throttle interface was touched; every existing working ROS 2 package besides `autokart_safety_bridge` and `autokart_dashboard_gateway` (both introduced/modified by this same RC-05/RC-06 work, not pre-existing) is untouched.

---

## 0. Method

This sandbox has no physical HP60C camera, no Gazebo/real kart, and no `/mnt/data` evidence log. Where the real sensor/perception stack was unavailable, a synthetic ROS 2 publisher (`rc06_synthetic_sensor.py`, kept outside the delivered packages — see §4.1) stood in for it, publishing on the **real topic names** with the **real message types**, so the **real** `front_safety_envelope_monitor` and `autokart_dashboard_gateway` nodes, and the **real** browser dashboard, could be exercised over **real DDS and a real WebSocket/HTTP connection**. Every number in this report was captured by running that pipeline, not inferred from source reading. Raw evidence files are retained in the session scratchpad (`rc06_evidence/`): node logs, `ros2 topic`/`node`/`param` CLI output, a JSONL status-poll timeline, and 4 screenshots.

---

## 1. Build Verification — **PASS**

```
colcon build --packages-up-to autokart_dashboard_gateway autokart_safety_bridge
Summary: 4 packages finished [6.41s], 0 build failures
```
`autokart_msgs`, `autokart_perception`, `autokart_dashboard_gateway`, `autokart_safety_bridge` all built cleanly against ROS 2 Humble. (The `ament_cmake_pytest` "pytest module not found" CMake warning is a pre-existing sandbox packaging quirk unrelated to this change — tests were run directly with `pytest` instead, see §2.)

---

## 2. Automated Test Results — **PASS** (72/72)

| Suite | Count | Result |
|---|---|---|
| `autokart_safety_bridge/test/test_envelope.py` (classification logic) | 13 | **13/13 PASS** |
| `autokart_dashboard_gateway/test/test_core.py` + `test_live_gateway.py` (incl. 3 new RC-06 tests) | 19 | **19/19 PASS** |
| `phase_sim` vitest full suite (incl. 5 new `rosLivePanel.test.tsx` tests) | 40 | **40/40 PASS** |
| `tsc --noEmit -p phase_sim` | — | **PASS**, zero errors |
| `npm run build` (phase_sim production build) | — | **PASS** |

All pre-existing tests in both the Python and TypeScript suites still pass — **no regression detected** in cone/mapping/planning/replay/lab behavior that this change touched incidentally (`core.py`, `gateway.py`, `schema.ts`, `RosLivePanel.tsx`).

---

## 3. Live Pipeline Integration — **PASS**, with one coverage gap noted (§6)

### 3.1 Real ROS 2 graph was formed and verified

```
$ ros2 node list
/dashboard_gateway
/front_safety_envelope_monitor
/rc06_synthetic_sensor

$ ros2 topic info /safety/envelope -v
Type: autokart_msgs/msg/SafetyEnvelope
Publisher count: 1   (front_safety_envelope_monitor)
Subscription count: 1 (dashboard_gateway)
```

### 3.2 Publish rate — **PASS**

```
$ ros2 topic hz /safety/envelope
average rate: 10.001   min: 0.100s  max: 0.100s  std dev: 0.00007s  (window: 32)
```
Matches the configured `publish_rate_hz: 10.0`.

### 3.3 Message content — **PASS**

```
$ ros2 topic echo /safety/envelope --once
clear_distance_m: 10.0
stopping_distance_m: 2.8
limiting_reason: CLEAR:OBSTACLE_OUTSIDE_BRAKING_ENVELOPE:CALIBRATION_UNVERIFIED
valid: true
```

### 3.4 End-to-end state transitions, live through the gateway HTTP API — **PASS**

Polled `/mission/api/status` every 2s across one full synthetic phase cycle (CLEAR → BRAKING → CRITICAL → STALE → CLEAR). Every transition landed on the expected state with the correct numbers (2 m/s test speed, trigger distance = 1.0 + 2·0.25 + 2²/(2·2.0) + 0.3 = **2.8 m**, matching the published `stopping_distance_m` throughout):

| wall time (UTC) | envelope state | clear_distance_m | stopping_distance_m | odometry |
|---|---|---|---|---|
| 10:03:37.100 | UNKNOWN | — | — | STALE |
| 10:03:39.131 | CLEAR | 10.0 | 2.8 | CURRENT |
| 10:03:43.151 | CLEAR | 10.0 | 2.8 | CURRENT |
| 10:03:45.160 | BRAKING | 2.0 | 2.8 | CURRENT |
| 10:03:47.169 | BRAKING | 2.0 | 2.8 | CURRENT |
| 10:03:49.177 | CRITICAL | 0.4 | 2.8 | CURRENT |
| 10:03:53.194 | CRITICAL | 0.4 | 2.8 | CURRENT |
| 10:03:55.206 | UNKNOWN | — | — | STALE |
| 10:03:59.269 | CLEAR | 10.0 | 2.8 | CURRENT |

Full 12-sample timeline retained in `status_timeline.jsonl`.

### 3.5 Browser verification, against the real running gateway — **PASS**

Vite dev server proxying `/mission` → real gateway on `127.0.0.1:8765`, loaded in the built-in browser, no mocks.

- **FRONT SAFETY ENVELOPE panel**, state CRITICAL, live: `0.40 m from bumper`, `2.80 m · provisional, not a measured braking test`, `/safety/envelope FRESHNESS: CURRENT · 43 ms`, `CALIBRATION STATUS: UNVERIFIED · schematic overlay only`, `VEHICLE AUTHORITY: OFF · DIAGNOSTIC_ONLY`. Screenshot: `screenshots/live_dashboard_CRITICAL_state.jpg`.
- **No-backend fail-safe**: before the pipeline was started, the panel showed `⚪ UNKNOWN` / `MISSING · TOPIC NOT PUBLISHING` — never a fabricated CLEAR. Screenshot: `screenshots/no_backend_fail_safe_UNKNOWN.jpg`.
- **Camera-frame synchronization** — **PASS**: once the harness published a `MissionDebug` "perception" status with a `sample_time_ns` matching the paired RGB/depth image stamps, `/mission/api/camera` reported `"available": true, "source": "LIVE", "valid": true` with 100% valid depth pixels, and the browser camera panel switched from `CAMERA DISCONNECTED` to `RGB CAMERA · LIVE`, `DEPTH · LIVE`. Screenshot: `screenshots/live_dashboard_CLEAR_camera.jpg`.
- **Schematic overlay renders live and tracks state** — confirmed at the DOM level (screenshots of the overlay's thin dashed strokes are low-contrast against the harness's flat synthetic color frames and don't photograph clearly at the dashboard's thumbnail panel size — a cosmetic limitation of the synthetic test frames, not a functional defect):
  ```js
  svg.getAttribute('aria-label') === 'Front safety envelope CRITICAL, geometry unverified, schematic only'
  // 5 children: 1 polygon + 3 <g> bands (opacity 1 on the active CRITICAL band, 0.35 on the others) + 1 "GEOMETRY UNVERIFIED · SCHEMATIC ONLY" text
  ```
  Re-checked against CLEAR and BRAKING states; `aria-label` and the active-band opacity updated correctly each time.
- **Stale camera → disconnected, not fabricated** — during the synthetic STALE phase (harness stops publishing everything, including images), the dashboard correctly showed `CAMERA DISCONNECTED` / `WAITING FOR ROS`. Screenshot: `screenshots/live_dashboard_STALE_camera_disconnected.jpg`.

### 3.6 Topic-dropout / stale-data fail-safe — **PASS**

Killed the `front_safety_envelope_monitor` process directly (not just its `ros2 run` wrapper) and polled `/mission/api/status` every 1s for 9s:

| wall time | channel state | age_ms |
|---|---|---|
| 10:06:12.458 | CURRENT | 85 |
| 10:06:13.468 | **STALE** | 1095 |
| 10:06:21.653 | STALE | 9280 |

The gateway's `safety_envelope` channel transitioned to STALE within ~1s of the publisher dying and **stayed STALE** for the full 9s window — it never reverted to a fabricated CLEAR. Restarting the node recovered the channel to CURRENT within 5ms of the first new message.

### 3.7 Diagnostic-only authority contract — **PASS**, with one finding (not a failure — see analysis)

```
$ ros2 node info /front_safety_envelope_monitor
Publishers:
  /parameter_events, /rosout, /safety/envelope        <- nothing else
Service Clients:  (none)
Action Clients:   (none)

$ ros2 param get /front_safety_envelope_monitor vehicle_authority       -> OFF
$ ros2 param get /front_safety_envelope_monitor safety_monitor_mode     -> DIAGNOSTIC_ONLY
```
Confirms, independent of source reading, that the running node has **no publisher, service client, or action client to any VCU/brake/throttle/drive-inhibit interface** — the "no actuation path" property is structural, not merely a flag.

**Finding (informational, not a blocker):** `ros2 param set /front_safety_envelope_monitor vehicle_authority ARMED` succeeds at the ROS parameter-server level — the node only checks `vehicle_authority`/`safety_monitor_mode` once, at startup (`node.py` `__init__`), and has no `on_set_parameters_callback` rejecting a later change. Re-confirmed after the probe that the node's publisher list was still only `/safety/envelope` — changing the parameter value has **no behavioral effect**, because no actuation code path exists anywhere in the node for it to unlock. The safety property holds by construction (absence of capability), not by the runtime parameter check alone. Recommend adding a parameter-change guard for defense-in-depth in a future change; **not applied here**, per this task's "audit only, no feature work" instruction.

---

## 4. Regression Check — **PASS**

- No existing node, launch file, message, or config outside `autokart_safety_bridge` (new package) and the two edited files in `autokart_dashboard_gateway` (`core.py`, `gateway.py`) was modified.
- `core.py`/`gateway.py` changes are strictly additive: a new `safety_envelope` channel name, a new nested `safety.envelope` field, a new top-level `safety_envelope` status channel. The pre-existing `safety.state` (VCU-derived) field and all other channels are untouched, and their existing tests (16 of the 19 gateway tests) still pass unmodified.
- `schema.ts`/`RosLivePanel.tsx` changes are additive (new optional `safety.envelope` field, new section/overlay component); no existing field, component, or prop signature was changed. Pre-existing component tests (`liveMissionControl.test.tsx`, `mission.test.ts`, etc. — 35 of the 40 vitest tests) still pass unmodified.

### 4.1 Test infrastructure note

`rc06_synthetic_sensor.py` (the HP60C/perception stand-in used for §3) lives in the session scratchpad, **not** in any ROS package, and is not part of this change's deliverable. `.claude/launch.json` was added at the repo root to let the dashboard dev server be previewed; it configures no new application behavior.

---

## 5. NOT TESTED (explicit — acceptance is withheld on these)

| # | Item | Why not tested |
|---|---|---|
| 1 | Real HP60C camera hardware | Not attached to this sandbox. All camera/perception data in §3 came from a synthetic publisher standing in for it. |
| 2 | Real `autokart_perception` YOLO26s pipeline | Not exercised; the harness published directly to the topics that node would normally produce. |
| 3 | Measured physical braking performance / stopping distance | Explicitly out of scope — `AutoKart_HP60C_Camera_Geometry_Production_Spec.md` states "Physical actuation/VCU/E-stop are out of scope and remain disabled." `response_latency_s`, `guaranteed_deceleration_mps2`, and `distance_margin_m` remain the illustrative values from the original design doc, never validated against a real brake test. |
| 4 | Verified camera mount/intrinsics calibration | By design still UNVERIFIED; this audit only confirmed that `calibration_verified=false` propagates correctly end to end (it does — §3.5). |
| 5 | Per-object (orange/red) obstacle markers on the overlay | Not implemented — `SafetyEnvelope.msg` carries one scalar nearest-clearance value, not per-object pixel geometry. Documented limitation from the original RC-06 build. |
| 6 | Long-duration / soak run (the project's own T5 "10-minute run" gate) | Not executed; this audit's live run totaled roughly 6 minutes across two sessions. |
| 7 | Multiple concurrent browser clients, reconnect-under-load, mobile/narrow viewport | Not executed; single desktop-viewport browser session only. |

---

## 6. File Manifest & SHA-256 Checksums

Computed at 2026-10-18T10:10Z, after the final test run, from the working tree (no git repository at the project root to diff against).

```
9a0cab61111d0d1df938d6d721fdde23306ad097950f5adc62b337ed26d4788d  kart/ros2_ws/src/autokart_safety_bridge/CMakeLists.txt
a4c5cc80b355754a3846bd5d861cb23a66d6df160e2e19e207567e3b0b302dbf  kart/ros2_ws/src/autokart_safety_bridge/package.xml
9ed3a288a299c168b9ec1d286a6de88fb17091942769c99bbdb75257641ef589  kart/ros2_ws/src/autokart_safety_bridge/config/front_safety_envelope_monitor.yaml
ad491cb4c0ba4a0c63c93e178964dc443c24732cfc2652c34294da38d1207aad  kart/ros2_ws/src/autokart_safety_bridge/launch/front_safety_envelope_monitor.launch.py
e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855  kart/ros2_ws/src/autokart_safety_bridge/autokart_safety_bridge_ros/__init__.py
719142b9301f69829f41637711544a7f5b57497904968ef07f92e94a42d06d8e  kart/ros2_ws/src/autokart_safety_bridge/autokart_safety_bridge_ros/envelope.py
60bfb0ccf481d1490f847256945f27377e55d577c2921c6a7bd37b57af5db2bb  kart/ros2_ws/src/autokart_safety_bridge/autokart_safety_bridge_ros/node.py
38b747e3f37a07a578a918fceee0ed6cb48d47668b884d476a75480922d8e8a8  kart/ros2_ws/src/autokart_safety_bridge/scripts/front_safety_envelope_monitor
0dbd492ca350053f1b1ed191991a706e4a80492a451862ba76499724d7c5adda  kart/ros2_ws/src/autokart_safety_bridge/test/test_envelope.py
9adafb1195338a154f115c2a61bd2aab04f40f8ee92e0d833dcae6bffa520585  kart/ros2_ws/src/autokart_dashboard_gateway/autokart_dashboard_gateway/core.py
b461464b50c280f55d0e4cb61a1d8d7392e233247feb10aec24eac2e14da354b  kart/ros2_ws/src/autokart_dashboard_gateway/autokart_dashboard_gateway/gateway.py
083ad545a860e5d4e4acb400d67806ed440321f7808758c456b9ac6966606ae8  kart/ros2_ws/src/autokart_dashboard_gateway/test/test_core.py
7d3c295954464152b8c6873165c0caf06bac3b5c2f40585c9d3baf89b2d99d62  kart/ros2_ws/src/autokart_dashboard_gateway/test/test_live_gateway.py
65a4187e720b450f8effb3e1ffe39287e890cebdba58f25ee335a2c7ef4a59d6  phase_sim/src/mission/schema.ts
65adba3f714de3f5d2e0a541bdd3946d630d57a6d3d05685979c53fcd537c497  phase_sim/src/mission/RosLivePanel.tsx
3f56084bb04f5b4968a1725c4f08aca64b1ee2f11294f7f9e874ee457a55660b  phase_sim/src/test/rosLivePanel.test.tsx
e99f34aec35bd29a6e4a2b69fe727b43815e7af1829df345e4ce7f4121ce04f9  .claude/launch.json
```

---

## 7. Overall Verdict

| Area | Verdict |
|---|---|
| Build | PASS |
| Automated unit/component tests (72) | PASS |
| Real ROS 2 topic graph, rate, content | PASS |
| Live state transitions (CLEAR/BRAKING/CRITICAL/UNKNOWN) through the real gateway | PASS |
| Browser rendering against the real running gateway | PASS |
| Camera-frame synchronization | PASS |
| Stale/dropout fail-safe behavior | PASS |
| Diagnostic-only authority (no actuation publisher/client exists) | PASS |
| Regression (pre-existing behavior unchanged) | PASS |
| Hardware-in-the-loop (real HP60C, real perception, real braking) | **NOT TESTED** |

**This report does not declare full RC-06 acceptance.** The software integration (ROS 2 node → dashboard gateway → browser) is verified working, correct, and fail-safe under the conditions actually exercised. Table §5 lists what remains open before this can be called hardware-accepted. Per the parent engineering contract, vehicle authority remains OFF and no further action beyond this diagnostic visualization is authorized by this report.
