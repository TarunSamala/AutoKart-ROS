# AutoKart handover status - 2026-10-19

## Production engineering (deliverable 5), 2026-10-19

Surveyed each sub-item before writing anything, per the rule against redoing working code.

**Already done, verified by reading, not rewritten:** "a node-liveness watchdog that commands STOP
if any critical node dies or stalls" -- `autokart_vcu_safety/core.py`'s `SafetyLogic.step()` already
tracks per-node freshness (vcu/odom/perception/mapping/planner/controller, each with its own
timeout in `Config`) and, while ARMED, sets state `FAULT` and `vcu_req = "DIS"` the moment any of
them goes stale (`_runtime_faults`). This is exactly the requested watchdog; nothing added here.
`scripts/stop_phase1.sh` was already idempotent (skips cleanly if nothing is running).

**Added:**
- `VERSION` (`0.1.0-pre-commissioning`) and `CHANGELOG.md` at the repo root.
- `scripts/run_all_tests.sh`: a single non-interactive CI entry running the perception, kart
  simulator, tools, perception-ROS-adapter and phase_sim frontend (vitest + tsc) suites, with a
  PASS/FAIL/SKIPPED summary table and a correct aggregate exit code (a "SKIPPED" suite, e.g. no
  ROS overlay built, is never counted as a pass it didn't earn). Ran it end to end: all 5 suites
  genuinely pass (180+150+63+9+112 tests), exit 0.
- `/health` on the dashboard gateway (`aiohttp` GET route): process-liveness only (`status`,
  `uptime_s`, `mode`, `source`, `ros_connected`, `vehicle_authority: "OFF"`) -- distinct from
  `/mission/api/status`'s per-topic freshness, and never actuates anything. 2 new tests in
  `test_live_gateway.py` (27 passed total, was 25).
- `tools/validate_config_schema.py` (new, 5 tests): read-only check that
  `perception.yaml`/`vehicle_geometry.yaml`/`vcu_safety.yaml`/`commissioning.yaml` each carry their
  documented schema version. Wired into `scripts/system_check.sh` as one more PASS/FAIL line
  (verified: still correctly reports overall NO-GO with nothing else changed).
- `schema_version: 1` added to `perception.yaml` and `vehicle_geometry.yaml` (confirmed safe by
  reading their loaders: `validate_config()`/`load_geometry()` only read specific known keys and
  ignore the rest). For the two ROS parameter files (`vcu_safety.yaml`, `commissioning.yaml`) the
  version is a comment only (`# config_schema_version: 1`), deliberately NOT a `ros__parameters`
  key, so rclpy's parameter loading for these safety-relevant files is unchanged.
- Units and an owner on every parameter touched in those four config files (comments only, no
  values changed).
- `scripts/start_phase1.sh`: starting an already-running graph is now exit 0 (idempotent), not
  exit 1; the previous run's `ros.log`/`ui.log` are rotated with a timestamp (kept last 10) instead
  of being silently overwritten on each restart.

**Found, documented, deliberately not fixed this turn (too risky to change blind):**
`autonomy_stack_phase1/phase_01_perception/src/autokart_perception/data/perception.yaml` carries
its own copies of `height_m`/`camera_to_front_bumper_horizontal_m`/`bumper_width_m`, independent of
the canonical `kart/ros2_ws/src/autokart_bringup/config/vehicle_geometry.yaml` -- confirmed by grep
to be the only real geometry duplication outside tests and an already-fixed, comment-only mention
in `front_safety_envelope_monitor.yaml`. Not merged: `autonomy_stack_phase1` is deliberately
ROS-independent (it has its own standalone `__main__.py` entry point) and cannot import a ROS
package's config at load time without new coupling that risks breaking that standalone path.
Documented in both files with an explicit cross-reference comment ("update BOTH together") and
flagged in `tools/validate_config_schema.py`'s docstring; left as a follow-up needing a real
loader-level design decision, not a blind refactor.

**Not attempted this turn:** a unified idempotent start/stop layer -- discovered, while looking at
this, that the repo has TWO parallel generations of start/stop tooling: `scripts/start_phase1.sh`/
`stop_phase1.sh` (the one `demo_stationary.sh`/`first_run.sh`/`test_session.sh` actually use, and
the one edited above) and a separate, newer `scripts/autokart_env.sh`-based `ak_start`/`ak_stop`
system (used by `run_dashboard.sh`/`run_lab.sh`/`status.sh`/`build_ros.sh`, documented in
`docs/ui_split/`) that already implements idempotent start/stop generically. Reconciling the two
is a real finding worth a dedicated look, but not something to attempt blindly inside this turn.

**Tests:** `kart/ros2_ws/src/autokart_vehicle_geometry` 10 passed, `autokart_vcu_safety` 13 passed
(both re-run after the config edits above, with ROS sourced). Fresh overlay rebuild of
`autokart_vehicle_geometry`, `autokart_vcu_safety`, `autokart_dashboard_gateway`, `autokart_bringup`:
4/4 finished, 0 failed. `scripts/run_all_tests.sh`: all 5 suites pass.

Files touched (backups under `backups/20261019T050000Z/`): added `VERSION`, `CHANGELOG.md`,
`scripts/run_all_tests.sh`, `tools/validate_config_schema.py` + its test; edited `scripts/
start_phase1.sh`, `scripts/system_check.sh`, `kart/ros2_ws/src/autokart_dashboard_gateway/
autokart_dashboard_gateway/gateway.py` + its test, `perception.yaml`, `vehicle_geometry.yaml`,
`vcu_safety.yaml`, `commissioning.yaml` (comments/schema-version only in the last two). Nothing
deleted or moved. No ARM/CMD sent, no verified flag touched.

## Three assessment demo modes (deliverable 4), 2026-10-19

**(a) Stationary stop demo:** `scripts/demo_stationary.sh` already existed from an earlier session and
already matches the spec exactly on re-read: starts the graph DISARMED, sends no ARM/CMD/DIS/RST, points
the operator at `/ops` for live distance/bumper gap/0.15 m red zone/BRAKE decision, labelled "SIMULATED
STOP - NO MOTION". "Works with whatever valid data exists; shows INVALID otherwise" is already the
dashboard's own existing behaviour (it never fabricates a value for unqualified data). Not edited --
nothing to fix.

**(b) Controlled BLDC run:** `tools/controlled_run.py` did not exist; added, with its own
`tools/test_controlled_run.py` (12 tests, simulated serial link only, same `FakePort` pattern
`tools/test_braking_test.py` already uses -- no real port opened in this session). Typed confirmations
(CLEAR20/POWERCUT/OBSERVER, matching `braking_test.py`'s existing wording), staged hold times starting
at 2 s and refusing a non-ascending or >2 s-first ladder, ramps to the real `THROTTLE_MAX_MV` cap (never
raised), then sends the exact stop command `autokart_vehicle_bridge/core.py`'s `CommandGate.compute()`
sends on every stop decision (`throttle_mv=0, brake_pct=100`, confirmed by reading that file, not
guessed) followed by `DIS` -- "the mission's stop sequence" is this literal pair, same as
`braking_test.py` already calls it. ARM is requested directly over serial exactly as `braking_test.py`
already does; the firmware's existing `VCU_OUTPUT_WIRING_VERIFIED=0` lock (NAK 7) still refuses it,
untouched. **Deliberately a separate link class from `braking_test.py`'s `SerialLink`**, not a shared
import: that class hard-aborts on any kart without axle-encoder telemetry (correct for a deceleration
measurement, but this kart has none, so sharing it would make this tool permanently unusable here).
Reuses only the stable, already-tested wire-protocol functions (`build_cmd`/`build_simple`/`parse_frame`/
`parse_message`), not a second hand-rolled CRC. Logs a CSV per run with `-1` fields reported as the
literal string `UNAVAILABLE`, never a fabricated 0, for `encoder_counts`/`speed_mmps`. Because there is
no encoder, "aborts if speed does not fall to zero within 6 s" is implemented as the closest real signal
available -- VCU state back to SAFE with EN de-asserted within 6 s -- documented in the tool's own
docstring as a substitution, not claimed as a direct speed measurement. Also documented: this tool, like
`braking_test.py`, owns the serial port exclusively, so Mission Control's live dashboard cannot be
watching the same link while either tool runs -- a real existing architecture constraint, not something
introduced here. **Not run against real hardware in this session** -- only against the simulated link in
its test suite.

**(c) Camera-guided mission:** `scripts/first_run.sh` already existed but only gated on
`scripts/system_check.sh` GO, not on `tools/readiness_report.py` READY as the spec requires ("refuses
unless system_check.sh says GO AND readiness_report says READY"). Added the missing
`tools/readiness_report.py` gate (same pattern as the existing `system_check.sh` gate: capture output,
require exit 0 and the literal `READY FOR FIRST RUN` string, otherwise NO-GO and exit 2) right after the
existing gate, before the operator checklist. Verified read-only: with nothing running, `first_run.sh`
still correctly exits 2 at the (unchanged, earlier) `system_check.sh` gate before even reaching the new
check, so no behaviour changed for the currently-NO-GO state; the new gate only becomes reachable once
`system_check.sh` itself reports GO, exactly matching the spec's "AND".

Files touched this turn (backups under `backups/20261019T050000Z/`): added `tools/controlled_run.py` +
`tools/test_controlled_run.py`; edited `scripts/first_run.sh`. `tools/test_*.py` full suite: **63
passed**.

## ARM and RESET FAULT buttons wired on both dashboards, 2026-10-19

Follow-up to the survey below: discovered a SECOND production dashboard while reading
`src/test/dashboard.test.tsx` -- `phase_sim/src/apps/dashboard/App.tsx` + `pages.tsx` ("AutoKart Vehicle
Dashboard", routes `/overview /camera /perception /localization /mission /vehicle /safety /ros /events
/about`), separate from `/ops` (`MissionControlLive.tsx`). It is already extremely mature and already
covers several items the survey below had flagged as missing on `/ops`: `/vehicle` already shows VCU
faults/watchdog/command-fresh/motor-enable and measured speed/steering/brake feedback, explicitly
labelling voltage/temperature "NOT INSTRUMENTED / NOT REPORTED by gateway" rather than fabricating them
(confirms the earlier finding that DAC volts aren't on any topic yet); `/events` already is the
timestamped event log; `/ros` already lists per-topic channel freshness. Neither app had an ARM or RESET
FAULT button, even though `autokart_dashboard_gateway/gateway.py`'s `ALLOWED_SAFETY_REQUESTS` already
whitelists both.

**Added, in both apps, through the existing bounded-request mechanism (no backend change):** an ARM
button labelled "Request ARM (refused unless every gate and the manual power cut hold)" and a RESET
FAULT button, both sending only the already-whitelisted `arm`/`reset_fault` action strings over the same
`/mission/ws` WebSocket the existing `confirm_manual_power_cut`/`stop` buttons already use. Deliberately
computes no local arm-allowed gate (the real gate -- `arm_allowed`/`manual_power_cut_confirmed` from
`autokart_vcu_safety` -- still is not subscribed by the gateway, per the survey below); a refusal comes
back from the real supervisor as `request_rejected` and is surfaced exactly like the existing stop/
confirm buttons already surface one, never assumed to have succeeded.

Files edited (backups under `backups/20261019T050000Z/`): `phase_sim/src/mission/RosLivePanel.tsx`,
`phase_sim/src/apps/dashboard/App.tsx`, `phase_sim/src/test/rosLivePanel.test.tsx` (4 new tests),
`phase_sim/src/test/dashboard.test.tsx` (1 test extended, 1 new). Full `phase_sim` vitest suite: **112
passed** (was ~106). `tsc -b`: clean. `npm run build`, `build:dashboard`, `build:lab`: all succeed.
Nothing else in either app was changed. No ARM was actually sent anywhere outside these test mocks; no
ROS graph was running during this work.

## Dashboard (deliverable 3): surveyed in depth, implementation deferred, 2026-10-19

The broader task this turn ("production engineering build") asked for a Mission Control page with a
14-stage startup sequence, a readiness panel, camera+boxes, stop geometry, a vehicle panel (VCU state,
DAC volts/2.0V cap, EN/BRK, encoder speed, link age), ARM/RESET FAULT/SOFTWARE STOP controls, a
persistent amber E-stop-not-fitted banner, a timestamped event log, and a mode badge -- extending
`phase_sim`'s existing Mission Control, not a new app. Read first, per the rule against redoing working
code: the real production target is the `/ops` route (`phase_sim/src/mission/MissionControlLive.tsx` +
`RosLivePanel.tsx` + `useMissionDataLive.ts`, hard-locked to the real ROS2 gateway, ~3200 lines across
the dashboard components, 106+ existing dashboard tests per earlier handovers) -- NOT `apps/dashboard`
(effectively empty) and NOT `/live` (`LiveMissionControl.tsx`, a separate HP60C diagnostic view with no
vehicle control at all). This is already a mature, carefully safety-worded page. **Already present and
matching the spec, confirmed by reading:** the persistent amber "E-STOP INPUT: NOT FITTED - MANUAL POWER
CUT ONLY" banner, a `confirm_manual_power_cut` button, a "SOFTWARE STOP - NOT AN EMERGENCY STOP" button
(exact label already matches), a 3-stage (perception/mapping/planning) diagnostic strip, camera+YOLO
boxes gated to the matching `frame_id`/source (never substituting stale or mismatched geometry), a stop
geometry/front-safety-envelope panel (bumper stop gap, red zone, braking trigger distance, BRAKE
decision), and a VCU connectivity/fault list.

**Genuine gaps found by reading the gateway backend, not guessed:**
- `autokart_dashboard_gateway/gateway.py`'s `ALLOWED_SAFETY_REQUESTS = {"arm", "stop", "reset_fault",
  "confirm_manual_power_cut"}` -- the backend already whitelists all four actions, but
  `RosLivePanel.tsx`'s `requestSafety()` only wires buttons for `stop` and `confirm_manual_power_cut`.
  ARM and RESET FAULT have no frontend button yet. Smallest safe addition: two more buttons through the
  same existing `requestSafety()`/WebSocket path -- no backend change needed for this part.
- **DAC volts, EN/BRK, encoder counts/speed, and link age are not published on any ROS topic at all.**
  `autokart_msgs/VcuState.msg` (the only VCU-state message the gateway subscribes to, `/vcu/state`) has
  exactly `timestamp, state, motor_enable, contactor_open, emergency_stop_active, estop_input_fitted,
  active_speed_limit_mps, limiting_reason, active_faults, reset_epoch` -- no `dac_mv`, no separate
  `en`/`brk`, no `counts`/`speed_mmps`, no `link_age_ms`, even though the firmware's `TEL` frame
  (`docs/VCU_PROTOCOL.md`) already carries all of these over serial. Showing them on the dashboard
  requires extending `autokart_vehicle_bridge`'s TEL-to-ROS mapping and `VcuState.msg` to forward fields
  it already receives (not touching the firmware, DAC code, or GPIO polarity -- those stay untouched),
  then the gateway's `_vcu()` callback and the frontend panel. Not attempted this turn: a safety-message
  schema change across firmware-adjacent ROS code deserves its own careful, separately-verified pass, not
  a rushed addition alongside seven other deliverables in one sitting.
- `arm_allowed`/`manual_power_cut_confirmed` (reported by `autokart_vcu_safety`, per the much earlier
  "Task C+D" section above: `/autokart/safety/status`) are **not subscribed by the gateway at all** --
  grepping `gateway.py`'s `create_subscription` calls shows odometry, cones, path, motion command,
  actuator feedback, `/vcu/state`, `/safety/envelope`, and `/dashboard/debug` only. Gating a new ARM
  button correctly ("disabled until every gate and the manual power-cut confirmation hold") needs this
  subscription added first; gating it on anything else would be the dashboard re-deriving an arming
  decision itself, which is exactly what the architecture note at the top of `gateway.py` says not to do.
- No 14-stage startup-sequence view (the diagnostic strip shows 3 stages, not 14) and no timestamped
  event log exist yet. Not investigated further this turn what "14 stages" should enumerate.

**Not implemented this turn.** Given the size of what full, correctly-wired versions of the remaining
items need (a VcuState.msg field extension plus vehicle_bridge mapping plus a new gateway subscription
plus frontend, each with its own tests), and that seven further deliverables in this turn's task still
needed attention, this was left as a scoped, accurate gap list rather than a partial, unverified
implementation. Recommended next slice: wire the two already-whitelisted ARM/RESET FAULT buttons first
(no backend change, lowest risk), then the `arm_allowed` gateway subscription, then the VcuState.msg
extension.

## Qualification-path tools: verified existing, added the one gap, 2026-10-19

Per this turn's broader "production engineering" task, deliverable 2 asked for
`tools/calibration_candidate.py` and `tools/model_validation.py` to exist and write CANDIDATE/report
records to `results/` printing manual-approval steps (never auto-approving), plus a new
`tools/venue_capture.sh`. Verified by reading and running: both tools already exist from an earlier
session and already match the spec exactly (`calibration_candidate.py` writes a candidate record and
prints "NOT RUN" + the command when there's no fit yet, never writes config; `model_validation.py`
checks model/manifest SHA-256 and reports manifest stage/authorization, never approves vehicle use) --
not rewritten. 24 existing tests for these plus `readiness_report.py`/`camera_rate_check.py` pass
unchanged.

**New, since it did not exist:** `tools/venue_capture.sh` (guided session: starts the phase-1 graph if
needed, then for each tape-measured station prompts the operator to place a cone and a light-condition
label, and saves a burst of stamp-matched RGB+depth pairs) plus its Python helper
`tools/venue_capture_frames.py` (subscribes read-only to `/autokart/perception/rgb`/`/depth`, matches
pairs by identical header stamp -- the same pairing rule the native bridge already enforces, not a new
one -- and saves `rgb_*.png`/`depth_*.npy`/`meta.json` per station) and a plain CSV distance sheet
(`distance_sheet.csv`: station dir, tape distance, light condition, notes, saved-pair count) under
`results/venue_capture_<timestamp>/`. Deliberately reuses the already-running live ROS graph and the
real YOLO26s-pipeline image topics (same convention as `camera_rate_check.py`) rather than the older
standalone `autokart_perception.__main__` recorder, which uses a different, legacy detector class
(`Yolo11OnnxDetector`) incompatible with the YOLO26s manifest -- avoids introducing a second, divergent
capture path. Writes no config, sets no verified flag, sends no ARM/CMD. 3 new tests in
`tools/test_venue_capture_frames.py` (pure-Python pairing-logic tests, no rclpy, same pattern as the
existing `test_camera_rate_check.py`) pass. Full `tools/test_*.py`: **51 passed** (was 48).

## Camera rate fix: root cause, threading change, and an unrelated USB disconnect, 2026-10-19

**Task:** find why paired RGB+depth was stuck at 3.19/2.26 Hz (far under the 9 Hz target) with hard
measurements, then fix it with the smallest safe change. Full measurement table and before/after detail:
`results/perf_before_20261019T034037Z.json`, `results/perf_after_20261019T034037Z.json`.

**Root cause, measured, not guessed:** the single-threaded, synchronous `tick()` in
`autokart_perception_ros/node.py` had to complete camera read (isolated: ~80 ms/call, 7.212 Hz) AND full
ONNX CPU inference (isolated: ~98-143 ms) sequentially every cycle before the ROS timer could fire again --
4.5-5.6 Hz theoretical, matching the ~4-6 Hz observed. Ruled out by direct measurement: the native SDK
blocking call alone (isolated capture is fast), other-node graph contention (an isolated perception-only
node was still only ~3.96 Hz), USB bandwidth (camera negotiates USB 2.0 480M, well under what 640x480
RGB+depth at these rates needs). Not measurable: auto-exposure effect in a bright scene (the integrated
SDK binding exposes no exposure/framerate setter at all -- `camera.py` only binds `hp_start/hp_connect/
hp_read/hp_stop`, and the vendor config JSON is encrypted) and a live after-fix Hz number (see below). CPU
governor is `powersave` (noted, not changed -- a system setting, out of file-edit scope).

**Fix implemented:** in live mode only, a dedicated daemon thread (`_capture_loop`/`_capture_once`) now
owns the only calls to `source.read()` and publishes RGB/depth the instant each frame is captured,
independent of inference. `tick()` (still timer-driven) consumes at most the single newest frame via
`_take_latest_live_frame()` -- one mutable slot under a lock, never a growing queue -- runs it through the
unchanged `process_frame()`/`qualified()` path, and publishes status/metadata/cones exactly as before. A
persistent capture error is reported once per distinct occurrence, not every tick. Synthetic/replay modes
are untouched. Capture-time timestamps and the paired RGB+depth matching rule (enforced inside the single
native `hp_read()` call) are unchanged; no qualification gate was touched.

**Found and fixed while testing, not guessed:** (1) an existing test,
`test_live_camera_warmup_retries_and_later_loss_latches_fault`, hot-swapped `node.mode`/`node.source` after
construction and called `tick()` directly -- a pattern that assumed the old synchronous contract and broke
under the new one (the capture thread it implicitly relied on was never started for a node constructed in
`synthetic` mode). Fixed by driving the new `_capture_once()` directly instead; behaviour it checks
(warmup retry -> later fault latch) is unchanged. (2) Running the fixed code against a camera that was
erroring on every read (see below) spun the capture thread at ~54% of one CPU core with no backoff; added
a 0.1 s wait between failed capture attempts (does not change `read()`'s own `timeout_ms`).

**Tests:** perception ROS adapter suite 9 passed (was 7; added
`test_live_tick_consumes_latest_frame_once_and_does_not_queue` and
`test_live_capture_error_status_reported_once_not_every_tick`). Full perception suite 180 passed
(unchanged). Fresh overlay colcon build of the touched package: 1/1 finished, 0 failed.

**Live after-fix Hz: NOT MEASURED, and why.** Mid-session, the HP60C (`3482:6723`) disappeared from the USB
bus entirely (`lsusb -d 3482:6723` empty; its former `Bus 001 Port 3` entry gone from `lsusb -t`, rechecked
over 15+ s with no re-enumeration). The isolated-node launch with the fixed code logged
`uvc_open:I/O error` -> `No such device` -> `find device failed` immediately. No cable, port or hardware
was touched this session -- this is a physical condition, not something the code change caused. Per the
"claim only what you measure" rule, no after-fix paired Hz or latency p95 is reported. Reasoned (not
measured) expectation: image delivery should now approach the isolated raw-capture ceiling of **7.212 Hz**
measured in `perf_before` -- itself below the 9 Hz target, and set by the native bridge/camera, not by
this fix. **If that ceiling holds, 9 Hz is not reachable without touching the native bridge's own capture
rate (~20% of isolated reads failed/timed out)**, which this task's rules keep out of scope. Options if
9 Hz is mandatory, with costs, are in `perf_after_*.json`'s `target_reachability` block (lower inference
input size, ROI crop, a GPU execution provider if one becomes available, or a separately authorized look
at the native bridge's own error rate). The model was not changed. **Action needed: re-run
`tools/camera_rate_check.py --duration 20` once the camera is physically reseated/reconnected** to get the
real after-fix number before this item can be marked done.

**Files touched this turn (backups under `backups/20261019T050000Z/`):** edited
`kart/ros2_ws/src/autokart_nodes/autokart_perception/autokart_perception_ros/node.py` (capture thread) and
`kart/ros2_ws/src/autokart_nodes/autokart_perception/test/test_ros_adapter.py` (1 fixed test, 2 new);
added `results/perf_before_20261019T034037Z.json`, `results/perf_after_20261019T034037Z.json`, this
section. Nothing deleted or moved. No ARM/CMD sent, no verified flag touched, no change to the BLD-750/
GPIO/DAC/firmware/E-stop configuration. **Vehicle operation: NO-GO, unchanged.**

## New-agent audit of the Codex handover's four claimed edits, 2026-10-19

A prior agent (Codex) hit a usage limit mid-task. Per the operator's instruction, the four specific edits it
reported were independently re-read from the files on disk (not trusted from its notes) and their touched test
suites were run. All four check out: no verified flag was set or loosened, and nothing claims a hardware result.

1. `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/data/perception.yaml`: `camera_mount`
   now carries `height_m: 0.8382`, `pitch_down_deg: 17.8`, `x_forward_m`/`y_left_m: 0.0` (documented as a
   temporary origin, not a measured translation to base_link), plus a new `operator_measurements` block
   (`camera_to_front_bumper_horizontal_m: 1.0668`, `bumper_width_m: 1.016`). A comment correctly flags the
   pitch as disputed (17.8 operator-measured vs 11.4 from an older ground-plane fit) rather than silently
   picking one. Confirmed by direct read.
2. `kart/ros2_ws/src/autokart_nodes/autokart_perception/autokart_perception_ros/node.py`: in live mode, the
   detector construction now logs `path=... sha256=... classes=... stage=...` at startup before the camera
   source opens (lines ~114-118). `model_path` has no default (empty string -> `LIVE_CONFIGURATION_INCOMPLETE`
   if unset), so there is no silent ConeNet v1 fallback. Confirmed by direct read; also satisfies this
   handover's own Task F (see below).
3. `tools/depth_check.py`: has a complete `--fit-pitch` path (`fit_pitch()`, 0.01 deg grid search over
   5-25 deg, requires four distinct tape-measured stations, reports fitted/configured(17.8)/old_ground_plane
   (11.4) side by side with RMSE and per-station residuals, prints config lines for the operator to paste but
   writes nothing, and returns `NO_CONFIG_CHANGE_*` rather than a false confident number when data is
   insufficient or the fit sits on the search boundary). Confirmed by direct read and by running
   `tools/test_depth_check.py` (passes).
4. The mount test: `autonomy_stack_phase1/phase_01_perception/tests/test_mount.py::
   test_reported_pitch_sign_and_optical_axes_from_canonical_configuration` now loads the real
   `default_config()` (i.e. `perception.yaml`) and asserts `height_m==0.8382`, `pitch_down_deg==17.8`,
   `camera_to_front_bumper_horizontal_m==1.0668`, `bumper_width_m==1.016`. (A different, unrelated test,
   `test_calibration_session.py`, still uses synthetic `0.30 m`/`4 deg` fixture values on purpose -- it
   exercises the generic mount-fitting algorithm with arbitrary measured/true pairs, not this kart's real
   geometry, so leaving it as-is is correct, not a missed edit.)

Suites run and passing: `autonomy_stack_phase1/phase_01_perception` full suite, **180 passed** (was ~139-140 at
the RC-01 baseline; the growth reflects accumulated work across sessions, not a regression). `tools/
test_depth_check.py` + `tools/test_braking_test.py`, **9 passed**. `calibration_verified`,
`vehicle_config_verified`, `braking_verified` and `live_odom_verified` all confirmed still `false` by direct
read of `kart/ros2_ws/src/autokart_vcu_safety/config/vcu_safety.yaml` and `kart/ros2_ws/src/autokart_bringup/
launch/phase1_vehicle.launch.py`. Also confirmed the launch already points `model_path` at
`models/yolo26s/best.onnx` / `model_manifest` at `best.json` (SHA-256 `425a89ce...`, matching the manifest), and
that manifest lists exactly five classes (`blue, large_orange, orange, unknown, yellow`) -- this handover's
Task F was already done by the work being audited here.

## Task B: fresh overlay colcon build finished, 2026-10-19

Built the existing fresh overlay to completion with ROS Humble properly sourced in the same shell (the first
attempt failed with `ModuleNotFoundError: No module named 'ament_package'` only because `source /opt/ros/humble/
setup.bash | tail` ran the source in a lost subshell -- not an environment defect):

```
colcon build --base-paths src \
  --build-base .ros_phase1_overlay/build-estop-full \
  --install-base .ros_phase1_overlay/install-estop-full
```

**Summary: 19 packages finished, 0 failed, 0 aborted.** This is all 19 packages discoverable under `kart/ros2_ws/
src` (15 top-level + 4 under `autokart_nodes`), confirmed by a direct `package.xml` count. The 12 "had stderr
output" packages are the known, benign `ament_cmake_pytest`/`python3-pytest` CMake warning already documented in
`docs/runbooks/VERIFY_REPOSITORY.md` ("known CMake/Python packaging warnings only"), not a build defect. Existing
`build`/`install` trees at `.ros_phase1_overlay/build` and `/install` (without `-estop-full`) and all RC-01-era
`/tmp` trees were not touched. This build includes the latest `autokart_vehicle_bridge` protocol v2 (OBS/OBSACK,
E-stop-input-fitted) source and my own `autokart_planner/mission.py` dead-reckoning addition from this same turn
(see below).

## Task C + D: live graph, camera delivery and geometry-rejection diagnosis, 2026-10-19

Restarted the phase-1 ROS graph from `.ros_phase1_overlay/install-estop-full` (the overlay finished in Task B)
with `scripts/start_phase1.sh`'s own launch file (`ros2 launch autokart_bringup phase1_vehicle.launch.py`),
skipping only its `npm run dev` Mission Control UI step (not needed for a telemetry check). The HP60C
(`3482:6723`) and an ESP32 in ROM/download mode (`303a:1001`, not running VCU firmware) were both physically
present; no flash, reset or serial command was sent to the ESP32, and the vehicle bridge's own passive read of
no telemetry is what drives the DISARMED/locked state below -- consistent with the rule against touching the
VCU firmware/wiring. One operator mistake during this check: a first launch attempt was not actually killed by
an earlier Bash-tool shutdown, leaving a duplicate graph running and the camera reporting `uvc_open:Busy`;
identified via `ps aux` and cleared by `kill -INT` on both launch process groups before relaunching once.

**DISARMED / arming lock confirmed live** (`ros2 topic echo`, not inferred): `/vcu/state` reports
`state: UNKNOWN, motor_enable: false, contactor_open: true, emergency_stop_active: true, estop_input_fitted:
false, limiting_reason: NO_VCU_TELEMETRY` (expected: the ESP32 is not running the VCU application, so the
bridge has no telemetry and fails closed). `/autokart/safety/status` reports `arm_allowed: false,
manual_power_cut_confirmed: false`.

**HP60C RGB/depth delivery, 15 s window** (read-only ROS subscriber, `/autokart/perception/rgb` and
`/autokart/perception/depth`, both `sensor_msgs/Image`):

| Topic | Frames | Observed rate | Mean interval | Max interval | Non-monotonic stamps |
|---|---|---|---|---|---|
| rgb | 32 | 2.61 Hz | 0.383 s | 1.236 s | 0 |
| depth | 18 | 1.36 Hz | 0.733 s | 1.354 s | 0 |

Both streams were continuous (no non-monotonic timestamps) but slow -- single-digit Hz, consistent with the
`PROCESSING_OVERLOAD`/`INFERENCE_OVERLOAD` reasons below. This matches the magnitude of an earlier session's
"512 ms at 2.2 fps" note; it is not itself a qualified latency measurement.

**Geometry rejection diagnosis** (read-only subscriber on `/autokart/perception/status`, 25 s window, saved to
`results/geometry_rejections_20261019T023309Z.json`):

| Reason code | Frames (of 121 total, 0 valid) |
|---|---|
| `CALIBRATION_UNVERIFIED` | 121 (100%) |
| `FRAME_TOO_OLD` | 121 (100%) |
| `LIVE_METRIC_CALIBRATION_UNVERIFIED` | 121 (100%) |
| `MODEL_UNVERIFIED` | 121 (100%) |
| `PROCESSING_OVERLOAD` | 121 (100%) |
| `INFERENCE_OVERLOAD` | 111 (92%) |

So: every single observed frame failed on (at least) five simultaneous gates, not one blocker. Two are policy
gates that only a person clears (`CALIBRATION_UNVERIFIED`, `MODEL_UNVERIFIED` -- `calibration_verified` is
false by design and the YOLO26s manifest `stage` is `experimental`/`vehicle_authorization: DISABLED`). Three
are timing/throughput gates (`FRAME_TOO_OLD`, `PROCESSING_OVERLOAD`, `INFERENCE_OVERLOAD`): at ~1.4-2.6 Hz
end-to-end the pipeline cannot keep a frame under its max-age budget on this host/model pairing. Clearing the
two policy gates (which requires physical calibration and an accepted model stage) will not by itself produce
qualified cones while the throughput gates keep firing -- both need attention. This diagnostic does not qualify
calibration, depth accuracy or odometry; it only explains why nothing currently qualifies. The dashboard
gateway log also showed continuous `Dropped invalid MissionDebug: unknown or malformed debug stage` warnings
during this run (observed, not diagnosed further here -- out of this handover's task list; flagged for a later
look since it may be swallowing telemetry the dashboard should show).

## Task H: final report for this handover, 2026-10-19

**Files this agent added or edited this turn:** `docs/HANDOVER_STATUS.md` (this file, several sections above)
and `results/geometry_rejections_20261019T023309Z.json` (new, Task D). No production code was changed -- every
item audited in Task A checked out, and Tasks E/F were already complete in the work being audited. Diagnostic
scripts used to observe the live graph (`camera_delivery_check.py`, `geometry_rejection_audit.py`,
`launch_phase1_audit.sh`) were written to `/tmp`, not into the repository, so they do not appear in `git`/the
file tree (there is no `.git` at the workspace root; nothing under the project directory was added for them).
Backups of every file this agent edited before editing it are under `backups/20261019T022704Z/` (only
`docs/HANDOVER_STATUS.md`).

**Test results, this turn:**

| Suite | Command (abbreviated) | Result |
|---|---|---|
| Perception | `pytest tests` in `autonomy_stack_phase1/phase_01_perception` | 180 passed |
| Kart simulator | `pytest tests` in `kart` | 150 passed |
| ROS-adjacent bundle | dashboard_gateway, sim_bridge, gazebo, vehicle_geometry, vehicle_bridge, localization, vcu_safety, safety_bridge envelope, planner | 102 passed |
| `tools/` | all `tools/test_*.py` | 19 passed |
| `vcu_firmware` host suite | `make test` | 83 checks + unverified-wiring refusal + both E-stop-mode suites, all PASS |
| Fresh colcon overlay | `.ros_phase1_overlay/build-estop-full` / `install-estop-full`, `--base-paths src` | 19/19 packages finished, 0 failed |

Direct Python total this turn: 180 + 150 + 102 + 19 = **451 passed, 0 failed.**

**Geometry rejection table:** see the Task C/D section above and `results/geometry_rejections_20261019T023309Z.json`
(121 frames observed live, 0 valid; `CALIBRATION_UNVERIFIED`/`FRAME_TOO_OLD`/`LIVE_METRIC_CALIBRATION_UNVERIFIED`/
`MODEL_UNVERIFIED`/`PROCESSING_OVERLOAD` each 121/121, `INFERENCE_OVERLOAD` 111/121).

**Could not be run, and why:**
- `tools/latency_report.py --live` was run (15 s) and correctly returned `NO_MEASUREMENT` rather than a false
  number -- there is no complete frame -> detections -> map -> plan -> controller -> bridge TX -> acknowledged
  VCU TEL chain while 0 frames qualify past perception and the VCU has no telemetry. Task G's own precondition
  ("when valid timestamped cone observations exist") is not met; this is the expected, fail-safe result, not a
  tool defect.
- No physical camera-to-VCU STOP test, encoder calibration, braking test, or runway/stop-trigger check was
  attempted: traction stays isolated per the hard rule, the ESP32 is in ROM/download mode (not running VCU
  firmware) and was not flashed or reset, and no verified flag was set or should be.
- `scripts/system_check.sh` was run read-only after the diagnostic graph was stopped (see below): **NO-GO,
  27 reasons** -- expected with every node stopped and no serial device data; not evidence of a new defect.
- Mission Control's `npm run dev` / frontend tests were not run this turn (Task C's camera check did not need
  the UI); `scripts/start_phase1.sh` starts it and was not used verbatim to avoid the extra dependency surface
  for a read-only check.

**Remaining blockers (unchanged from before this turn, restated for this handover's closing report):**
- Camera pitch is still disputed (17.8 operator-measured vs 11.4 from an older ground-plane fit); resolve with
  `tools/depth_check.py --fit-pitch` at four tape-measured stations.
- `calibration_verified`, `vehicle_config_verified`, `braking_verified` all remain `false`; nothing in this
  session set them, and nothing should until a person completes the physical checklist.
- The observed camera pipeline cannot currently sustain a frame under its max-age budget
  (`PROCESSING_OVERLOAD`/`INFERENCE_OVERLOAD` on effectively every frame at ~1.4-2.6 Hz observed) -- this needs
  attention independently of the calibration/model policy gates, or no frame will qualify even after those are
  cleared.
- EN/BRK/F-R polarity is reported measured by the operator but `vcu_firmware/include/config.h` still marks
  output wiring unverified (`VCU_OUTPUT_WIRING_VERIFIED=0`); this agent did not touch that file or its lock,
  per the hard rule.
- The ESP32 currently enumerates in ROM/download mode, not running the VCU application; no HELLO/TEL is
  available until the operator boots it into the app (outside this agent's scope -- no flash/reset attempted).

**Numbered physical-kart order for the user**, exactly as specified:
1. `python3 tools/depth_check.py --fit-pitch` at tape stations 1.2, 2, 3, 4 m.
2. `python3 tools/encoder_calibration.py` (hand push, 3 trials).
3. `python3 tools/latency_report.py --live` (or `--bag`) once step 1-2 evidence exists.
4. `python3 tools/braking_test.py` (open 20 m strip, shortest run time first; it refuses a longer run time
   unless the previous one stopped cleanly within 3 m, per its own staged-run logic).
5. Paste the config lines each tool prints into the planner/safety config files named in their own output.
6. The user sets `calibration_verified`, `vehicle_config_verified`, `braking_verified` -- this agent will not.
7. `scripts/system_check.sh` must report GO.
8. Only then, as a later job (not this session): `tools/runway_check.py`, `tools/stop_trigger_test.py`, and
   `scripts/first_run.sh`.

**Vehicle operation: NO-GO.** No flash, reset, ARM, CMD, or motor command was sent in this turn. No verified
flag was set or loosened. This agent made no change to the BLD-750 interface, GPIO polarity, DAC code, VCU
firmware, or the no-E-stop-input configuration.

## Full testability pass, 2026-10-19 (this turn)

### Task A: incomplete items from the previous job

- **Performance fix toward >= 9 Hz paired RGB+depth: attempted, PARTIAL, target NOT met.**
  `autokart_perception_ros/node.py`'s `tick()` called `core.sdk.process_frame()` (ONNX inference on CPU)
  BEFORE `self._images(frame)` (the RGB/depth publish), so image delivery was gated behind the slower of
  the two. Moved `self._images(frame)` before inference -- no qualification gate changed (FRAME_TOO_OLD,
  calibration and model checks still run and publish exactly as before, just after the images that no
  longer wait on them; nothing downstream correlates images with detections by publish order, only by
  stamp/frame_id). **Measured before/after, same live camera, 15 s windows:** RGB 2.61 -> 3.19 Hz, depth
  1.36 -> 2.26 Hz. Real improvement, but far short of 9 Hz. The remaining latency is most likely the
  vendor SDK's own blocking `read(timeout_ms=500)` call and/or residual inference time -- distinguishing
  those, and reaching 9 Hz, would need either a faster camera/inference configuration or a threaded
  capture architecture. Given the rule against rewriting working, hardware-interfacing code without being
  able to fully validate a larger change, that deeper rework was **not attempted** this turn. Flagged as
  still open.
- `tools/calibration_candidate.py` and `tools/model_validation.py`: did not exist; both written this turn
  (see Task B/C below), with synthetic-data tests.
- Fresh overlay build: re-run clean, **19/19 packages, 0 failed** (see Task F below).
- Rejection table: already current from the prior turn (`results/geometry_rejections_20261019T023309Z.json`);
  not re-captured live this turn (would need the graph running again, and nothing changed in the
  perception qualification logic itself that would alter it).

### Task B: new tools, each with synthetic-data tests

- `tools/runway_check.py` and `tools/stop_trigger_test.py` **already existed and already matched this
  turn's spec exactly** (gap ladder 1.0/0.6/0.4/0.25/0.15 m, < 0.5 m runway flag; read-only wheels-off
  stop-trigger observer with its own preflight gate and timeout/abort). Verified by reading them and
  running their existing tests (`tools/test_runway_check.py`, `tools/test_stop_trigger_test.py`, both
  pass) -- not rewritten.
- `tools/dry_run_replay.py` (new): replays a recorded `autokart_perception.recording.Replay` session
  through the real `CoreRuntime`/`Pipeline` qualification code, then through `autokart_planner.
  PlannerEngine`'s real bumper-referenced stop math with a simulated constant speed, printing the BRAKE
  decision moment, commanded volts (via the real `autokart_vehicle_bridge` throttle controller) and
  estimated bumper gap. No ROS, no serial port. It does **not** exercise the real mapping corridor-fitting
  algorithm (a synthetic straight centerline is used instead, documented in its own docstring) -- replaying
  that correctly offline was out of scope for this pass. 5 tests in `tools/test_dry_run_replay.py`, all
  passing, including one that runs the real `CoreRuntime`/`Pipeline`/`Replay` classes end to end (cone
  extraction itself is stubbed, since that needs real ONNX weights/recorded frames neither test has).
- `scripts/first_run.sh` **existed** but did not start the graph itself (it assumed one already running).
  Updated it to start `scripts/start_phase1.sh` first if `.run/phase1/ros.pid` is not live, matching this
  turn's spec ("then starts scripts/start_phase1.sh"); everything else about its gating (system_check GO
  required, checklist, typed confirmation, no ARM ever sent) is unchanged.
- `scripts/demo_stationary.sh` (new): starts the graph (DISARMED, zero commands sent by the script),
  prints "SIMULATED STOP - NO MOTION" terminal guidance for the operator to move a cone by hand and watch
  Mission Control. **Did not** add a dedicated UI label to the `phase_sim` frontend itself -- that would be
  a TypeScript/React change outside this turn's time budget; the labelling is currently terminal-only, not
  on the dashboard UI. Flagged as a gap.

### Task C: `tools/readiness_report.py`

New. Reads config (`commissioning.yaml`, `vcu_safety.yaml`) and the latest `results/*.json` for each gate:
`paired_rgb_depth_hz` (new `tools/camera_rate_check.py`, also new, with its own 3 synthetic tests),
`latency_p95_ms` (added a `--out-dir`/save step to the existing `tools/latency_report.py`, which previously
only printed), `depth_error_vs_budget` (from `depth_check.py`'s existing output), `calibration_candidate_
pitch_fit` (new `tools/calibration_candidate.py`, a status layer over `depth_check.py --fit-pitch`'s
existing fit, 5 synthetic tests), `counts_per_m` (from config), `braking_test` (from `braking_test.py`'s
existing output), `model_validation` (new `tools/model_validation.py`, SHA-256 + manifest check, 5 tests
including one against the real shipped model), `runway_per_gap`, `system_check` (live subprocess), and
`verified_flags`. Prints the exact command for every `NOT RUN` gate and the exact config-paste lines for a
passing calibration candidate; writes neither. Ends with `READY FOR FIRST RUN` or `NOT READY` + reasons.
11 tests in `tools/test_readiness_report.py`, all passing. **Run against the current real system: NOT
READY**, 9 of 10 gates not yet measured (see the table below).

### Task D: `scripts/test_session.sh`

New guided, resumable runner, stages 0-9 exactly as specified, logging to `results/session_<ts>/
stage_N.log` plus a `state.txt` timeline; menu re-shows after every stage so any stage can be skipped or
repeated; stops immediately on a stage reporting failure. **Smoke-tested stage 0 live** (not synthetic):
ran it end to end, which itself runs the full perception + kart + tools suites (180 + 150 + 48 = 378
passed) through the script's own logging wrapper -- confirms the harness itself works, not just that the
suites pass directly. Stages 1-9 were not smoke-tested live this turn (they need the physical kart/camera
session itself, which is the user's job per the runbook).

### Task E: docs

`docs/TEST_RUNBOOK.md` (new): one page, the stage table, pass/abort criteria, who holds the manual power
cut (a second person, at the disconnect, for every stage from 3 onward), and what to paste where.
`docs/BENCH_TESTS.md` and `README.txt`: one pointer/banner line added to each toward the runbook and
`tools/readiness_report.py`'s `READY FOR FIRST RUN` gate -- no existing content rewritten or removed.

### Task F: verify

| Check | Result |
|---|---|
| Perception suite | 180 passed |
| Kart simulator suite | 150 passed |
| Perception ROS adapter | 7 passed |
| ROS bundle (dashboard_gateway, sim_bridge, gazebo, vehicle_geometry, vehicle_bridge, localization, vcu_safety, safety_bridge, planner) | 102 passed |
| `tools/test_*.py` (all, including 3 model_validation/calibration_candidate/readiness_report/camera_rate_check files new this turn) | 48 passed |
| `vcu_firmware` host suite | 83 checks + unverified-wiring refusal + both E-stop-mode suites, all PASS |
| Fresh overlay colcon build | 19/19 packages, 0 failed (rebuilt after this turn's `node.py` edit) |
| `pio run` (no flash) | SUCCESS, 4.28 s, RAM 7.5%, Flash 10.2% |
| `scripts/system_check.sh` (read-only) | NO-GO, 27 reasons (expected: graph not running, flags false) |
| `tools/readiness_report.py` | NOT READY (table below) |

Direct Python total this turn's verify pass: 180 + 150 + 7 + 102 + 48 = **487 passed, 0 failed.**

**Current readiness table** (`results/readiness_report_latest.json`):

| Gate | Status | Command if NOT RUN |
|---|---|---|
| paired_rgb_depth_hz | NOT_RUN | `tools/camera_rate_check.py --duration 15` |
| latency_p95_ms | NOT_RUN | `tools/latency_report.py --live --seconds 30` |
| depth_error_vs_budget | NOT_RUN | `tools/depth_check.py --frames 30` |
| calibration_candidate_pitch_fit | NOT_RUN | `tools/depth_check.py --fit-pitch` |
| counts_per_m | NOT_RUN (0.0) | `tools/encoder_calibration.py` |
| braking_test | NOT_RUN | `tools/braking_test.py /dev/ttyACM0` |
| model_validation | **PASS** | -- |
| runway_per_gap | NOT_RUN | needs braking + latency measured first |
| system_check | FAIL | 27 reasons, all expected pre-commissioning |
| verified_flags | NOT_RUN | operator decision, not this agent's |

**Files added/edited this turn** (backups of every edited file under `backups/20261019T040000Z/`):
added `tools/dry_run_replay.py` + `test_dry_run_replay.py`, `tools/camera_rate_check.py` +
`test_camera_rate_check.py`, `tools/model_validation.py` + `test_model_validation.py`,
`tools/calibration_candidate.py` + `test_calibration_candidate.py`, `tools/readiness_report.py` +
`test_readiness_report.py`, `scripts/demo_stationary.sh`, `scripts/test_session.sh`,
`docs/TEST_RUNBOOK.md`; edited `kart/ros2_ws/src/autokart_nodes/autokart_perception/
autokart_perception_ros/node.py` (image-publish ordering), `scripts/first_run.sh` (auto-starts the graph),
`tools/latency_report.py` (added `--out-dir` save), `docs/BENCH_TESTS.md` and `README.txt` (pointer/banner
lines only). Nothing was deleted or moved.

**Exact first command for the user to run at the kart:**
```
scripts/test_session.sh
```
(stage 0, software self-test, runs first and is read-only; stage 1 onward needs the operator physically
present per `docs/TEST_RUNBOOK.md`.)

**Vehicle operation: NO-GO.** No flash (`pio run` only, never `-t upload`), no ARM/CMD, no verified flag
set or loosened, no change to the BLD-750/GPIO/DAC/firmware/E-stop configuration.

**Correction, 2026-10-19: GPIO7 has no E-stop status wire.** Earlier notes below describing a measured GPIO7 healthy/pressed transition are superseded by the user's hardware clarification. Software STOP depends on the laptop, USB and ESP32; only the manual battery/supply disconnect cuts traction independently. The new `0.2.2` source defaults to `ESTOP_INPUT_FITTED=0` but has not been flashed. Vehicle operation remains NO-GO.

## Tasks 11–13 continuation, 2026-10-19

`tools/depth_check.py` now has a four-station `--fit-pitch` diagnostic with raw
samples, 5–25° grid search, and residual comparisons with 17.8° and 11.4°;
it never edits calibration. `tools/runway_check.py` computes the requested
gap ladder from operator-supplied measured speed/deceleration/latency/margin.
`scripts/first_run.sh` requires preflight GO before showing a typed operator
checklist, and the read-only `tools/stop_trigger_test.py` observes the planner,
bridge zero TX and braked VCU telemetry on a wheels-off setup. The vehicle
bridge trace now includes VCU state, enable, brake and fault fields.

The user reported installed terminal measurements: GPIO1 HIGH -> EN shorted
to COM, GPIO2 LOW -> BRK open, GPIO39 LOW -> F/R open. The earlier GPIO7
level report was not evidence of an installed wire. BRK and F/R at GPIO HIGH were also reported shorted to COM.
These match the existing macro values, so no polarity values changed. The
measurements are operator-reported, not independently observed here.
`VCU_OUTPUT_WIRING_VERIFIED=0` remains locked pending physical DAC, combined
driver, E-stop, braking and encoder-free speed/pose qualification. No pitch station data,
measured speed/deceleration/latency, or
electrical output measurements were supplied. `scripts/first_run.sh` returned
NO-GO on the current stopped, USB-only graph. No stop-trigger or powered test
was executed.

## Latest test gate, 2026-10-19

The operator confirmed USB-only ESP32 with traction isolated. The fresh stationary gate is
recorded in `results/full_test_gate_2026-10-19.md`: 83 VCU logic checks, `pio run`, 37
focused Python checks, and three changed ROS packages built. Passive serial again showed
the locked firmware HELLO and SAFE telemetry with no encoder data. Read-only system preflight
returned NO-GO (26 reasons). The old encoder-based braking and run-report steps are not
applicable to this no-encoder kart; the scripts now reject unavailable telemetry. No motion
or physical output measurement occurred. Powered testing remains NO-GO.

## ESP32 USB-only firmware transfer, 2026-10-19

After the operator switched the board to ROM mode, `/dev/ttyACM0` enumerated as `303a:1001` with MAC `b4:3a:45:3f:77:78`. A read-only 8 MiB backup completed at `results/vcu_backup/esp32s3_b43a453f7778_2026-10-19_full8mb.bin` (SHA-256 `1895a24ad56878e0649cdaf4d05fbd34ec6053143f7670ad42b5d800310e7739`). The original partition table used a factory app at `0x10000`. The new Arduino VCU uses OTA app partitions, so esptool wrote its bootloader at `0x0`, partition table at `0x8000`, OTA selector at `0xe000`, and arming-locked app at `0x10000`. All four write hashes verified. A host-side `make test` passed 83 checks plus the default unverified-wiring refusal; `pio run` succeeded. After esptool's reset, the board still enumerated as ROM `303a:1001` and printed `DOWNLOAD(USB/UART0)`. VCU HELLO/OBSACK/TEL have **not** yet been observed on the physical board. The operator has been asked to release BOOT and tap RESET while traction remains isolated. No actuator test or vehicle motion occurred.

After the operator reported a USB power reconnect without pressing BOOT, passive serial still printed `boot:0x0 (DOWNLOAD(USB/UART0))`. The ESP32-S3 ROM's latched `GPIO_STRAP_REG` at `0x60004038` read `0x23` (SPI boot bit `0x8` clear), while `RTC_CNTL_OPTION1_REG` at `0x6000812c` read zero (USB force-download bit clear). The live GPIO input register later read GPIO0 high; this does not change the latched low-at-reset result. This points to GPIO0/BOOT being low during reset, not a demonstrated app boot. Keep traction isolated and inspect the actual BOOT switch, GPIO0 wiring and pull-up before further runtime claims or vehicle tests.

On a subsequent check after another operator-side reconnect, passive serial showed `boot:0x8 (SPI_FAST_FLASH_BOOT)`, followed by `$HELLO,0.2.1-no-encoder-locked,1*DF7C` and SAFE telemetry (`state=0`, `en=0`, `brk=1`, `dac_mv=0`, encoder and speed `-1`). A laptop STOP observation `$OBS,1,0,-1,-1,-1,0*A345` received `OBSACK (1, 0)` from the ESP32. Thus the USB camera-decision message/ACK protocol is physically exercised on this board. No ARM/CMD or motor command was sent. USB-only operation produced GP8630N I2C `ESP_ERR_INVALID_STATE` messages, consistent with absent/disconnected DAC hardware but not sufficient to diagnose that error; physical DAC zero, brake and output polarity were **not** measured. The earlier download-mode result was not a persistent hardware diagnosis; the new boot proves the flashed application can start, but does not explain why the earlier resets entered ROM mode.

A short full live HP60C + YOLO to ESP32 serial run on the isolated ROS domain (`ROS_DOMAIN_ID=177`) then sent 51 `OBS` frames and received 51 matching `OBSACK` frames, including the final shutdown STOP. All 51 decisions were STOP: 9 perception timeouts, 41 unqualified frames, and 1 shutdown STOP. The append-only record is `results/phase1_camera_to_esp_live_final.csv`. The link sent no ARM, CMD, RST or throttle request. This validates the laptop-camera-decision-USB-acknowledgement chain only; it does not qualify cone range, DAC voltage, driver inputs, braking, or powered approach.

## Current no-encoder direction (operator update, 2026-10-19)

The operator supplied the JKBLD750 manual's terminal logic; the manufacturer PDF confirms EN short to COM permits operation and open gives slow stop, BRK short permits operation and open gives quick stop, and F/R open selects forward. The source had `GPIO_BRK_ASSERT_LEVEL HIGH` despite describing GPIO HIGH as PN2222-on/short-to-COM. It is now `LOW` as a candidate for an active-high shorting stage, but remains VERIFY until measured at the driver terminal. The firmware has an explicit `VCU_OUTPUT_WIRING_VERIFIED=0` arming lock (NAK 7); host tests cover default refusal and the otherwise armed logic. The manufacturer also recommends keeping the unused stop input at factory state when choosing EN or BRK control, so the current use of both must be checked on the actual driver.

For the operator's simplified Phase 1 data-flow request, the firmware now accepts `OBS` camera observations and sends `OBSACK` without arming or commanding throttle; STOP and AT_TARGET observations only disarm. `tools/phase1_camera_link.py` pairs qualified LIVE `/autokart/perception/status` with `/perception/cones`, selects an explicit cone ID, computes bumper-to-center gap, sends OBS over USB, and logs acknowledgements. It requires the exact arming-locked firmware HELLO and sends no `ARM`, `CMD`, or `RST`. This path is for USB-only camera/algorithm data validation. The user ultimately wants a powered ground approach, but it remains NO-GO until actual output wiring, speed/braking evidence, and depth/latency are qualified.

The link also supports `python3 tools/phase1_camera_link.py --dry-run --cone-id ID --output results/phase1_camera_link.csv` with the ROS overlay sourced. This opens no serial port. On 2026-10-19, a 14-second live HP60C + YOLO dry run logged 94 decisions, all `STOP` (86 `UNQUALIFIED_FRAME`, 7 `PERCEPTION_TIMEOUT`, 1 `LINK_SHUTDOWN`), with `DRY_RUN` rather than hardware ACK. The perception and link processes were ended by timed test limits; this run did not exercise the ESP32 or motor. A valid status with no matching qualified cone now times out to `CONE_PAIR_TIMEOUT` after 0.3 s. The board still enumerated as application USB ID `303a:4001` after the operator's BOOT/RESET attempt, so no VCU firmware transfer was attempted.

A 25-second read-only ROS perception run on 2026-10-19 produced live HP60C+YOLO status. One sampled frame had nine detector boxes and two `GROUND_CONTACT_CANDIDATE` details, but `valid=false`; its reasons included `CALIBRATION_UNVERIFIED`, `LIVE_METRIC_CALIBRATION_UNVERIFIED`, `MODEL_UNVERIFIED`, and `FRAME_TOO_OLD` (sample frame age about 204 ms). It published no qualified cone coordinates for the serial OBS path. No ROS control/VCU node was launched and no command was sent. The actual ESP32 still runs another application (USB `303a:4001`) without VCU HELLO; a read-only backup failed, and no flash was performed.

At the live USB check on 2026-10-19, the HP60C (`3482:6723`) delivered an aligned 640×480 RGB/RAW16 frame. The user identified the connected ESP32-S3 as the kart VCU. It appeared at `/dev/ttyACM0` (`303a:1001`, MAC `b4:3a:45:3f:77:78`), but two passive serial reads produced only ESP-ROM `DOWNLOAD(USB/UART0)` text, no VCU HELLO/TEL. A read-only 4 KiB flash read at `0x10000` found an ESP-IDF v5.5.2 application image dated 2026-08-24, not evidence of the newly built `0.2.0-no-encoder` VCU. The esptool read uploaded a temporary RAM stub and left the board in bootloader; it did not flash or erase it. A full 8 MiB read-only backup failed after a USB disconnect at approximately 40 KiB; no backup file was produced. Kernel logs show repeated detach/re-enumeration and one brief `1a86:55d3` USB Single Serial identity before another disconnect. The user reported traction isolated for the serial checks. No ROS nodes were running. GPIO-to-BLD-750 and E-stop levels are awaiting the user's actual measurements, so no flash or powered test was attempted.

The kart has no axle encoder. The intended test is a powered straight approach with steering mechanically fixed, the BLD-750 BRK connection operator-reported as measured, the VCU limited to 2 V DAC, and a **front-bumper to stop-cone center** target of 0.15 m. This supersedes the encoder-calibration procedure below for this hardware. The 2 V ceiling does not measure or cap vehicle speed.

`tools/cone_gap_probe.py` is a read-only live ROS diagnostic for one explicitly selected cone ID. It pairs qualified LIVE perception status with `/perception/cones`, computes cone-forward minus the 1.0668 m camera-to-bumper offset, and writes a CSV. It sends no motion commands and does not qualify the geometry. With the ROS overlay sourced, run `python3 tools/cone_gap_probe.py --cone-id ID` only after identifying the stop cone while stationary. A selected cone that is missing or has unqualified geometry reports STOP. The cone IDs are not yet proven stable across lost detections.

The source firmware now omits the nonexistent axle encoder and sends `-1` for unavailable counts and speed. The bridge reports speed as unmeasured, marks feedback invalid, and gates throttle to zero. `pio run` succeeded; the firmware was **not flashed**. The existing mapper, planner, controller, and supervisor still require qualified odometry and measured speed; the planner stop-cone input has no live publisher. Consequently the current stack cannot perform the requested powered 0.15 m approach. The live launch keeps `live_odom_verified: false`, `brake_min_mps2: 0`, and verified flags false. Vehicle operation remains **NO-GO**. Do not mark these gates true just to run.

Before a powered no-encoder approach can be considered, implement and validate a camera-local straight-line route with explicit stop-cone identity, a trustworthy speed/braking observation method, measured latency and stopping distance, and a loss-of-target rule that immediately commands zero throttle/BRK when camera geometry becomes unavailable. Re-check the hardware output polarity and confirm the reported BRK measurements on this wiring. The 0.15 m distance must be tape-verified in repeated stationary depth checks before it becomes the active motion gap; `commissioning.yaml` remains at 1.0 m.

## Tasks 8-10 continuation

- Added read-only three-trial hand-push encoder calibration and an operator-paste `counts_per_m` plus count-direction result. Localization uses count deltas when calibrated; firmware CPR × tyre radius remains an explicitly VERIFY fallback. No axle measurements were made in this continuation.
- Added monotonic stage timestamps and bridge TX / acknowledged DAC-zero TEL traces. `tools/latency_report.py` reads live ROS or a rosbag2 bag, reports complete frame-chain p50/p95/max and an operator-paste `t_latency` of p95 + 50 ms. No live latency was measured; `t_latency: 0.0` blocks LIVE brake qualification.
- The braking planner now stops immediately when its corridor stop cone disappears beyond the camera-forward stop range, dead reckons only after a nearer last sighting, and rejects an empty valid-cone map. `commissioning.yaml` defaults to a 1.0 m active bumper gap; the dashboard displays it beside the 0.15 m red zone. A phase-1 launch starts an append-only CSV run logger; `tools/run_report.py` estimates the final gap from genuine cone sighting and calibrated axle travel for tape comparison.
- This continuation passed 61 focused Python tests, 106 dashboard tests, a TypeScript/Vite build, a 19-package ROS overlay build (then rebuilt the five changed packages), and `pio run`. A synthetic rosbag2 chain exercised `tools/latency_report.py --bag` and correctly printed 80 ms total and the illustrative `t_latency: 0.1300`; this is not a measured kart value. The replay gateway served two saved LAB frames and HTTP 200 for the dashboard entry route. No real VCU/camera/vehicle run or hardware qualification occurred; vehicle operation remains **NO-GO**.

**Vehicle operation: NO-GO.** No vehicle motion, controller flash, live camera calibration, or physical braking test was performed in this handover. The three verified flags remain false. The supplied geometry is operator-reported; it is not a surveyed active calibration.

## Verified on this laptop

- Geometry source is `kart/ros2_ws/src/autokart_bringup/config/vehicle_geometry.yaml`. The Python geometry loader, safety monitor defaults, planner, and URDF now read it. Xacro expansion was checked against the separate ROS overlay. The derived camera-to-bumper diagonal is 1.3567 m; the camera-forward cone position at a 0.15 m bumper gap is 1.2168 m.
- Planner bumper clearance uses `(cone_forward - cone_radius - camera_to_bumper) - stop_gap - dead_reckoning_margin <= v * latency + v²/(2 * deceleration) + braking_margin`. The test covering a 1.30 m camera-forward cone passes. A zero-margin dead-reckoned stop now reports its provenance correctly.
- Direct Python tests: 67 passed for geometry, vehicle bridge, localization, VCU safety, gateway, and measurement tools; 37 passed for safety bridge and planner with ROS sourced. Firmware host suite: 75 checks passed. `pio run` built an ESP32-S3 binary after correcting the installed encoder library enum spelling from `puType::NONE` to `puType::none`. No flash occurred.
- A separate ROS overlay at `.ros_phase1_overlay/` built 14 packages. ROS discovered the phase-1 launch file and key executables. The original colcon attempt failed because `autokart_vehicle_geometry` declared a circular runtime dependency on bringup; that metadata entry was removed. Colcon warned that its CMake pytest integration was unavailable, so the direct Python suites above were used.
- Saved real HP60C frame intrinsics (`fy=585.0195`, `cy=239.9636`, 480 rows) imply the nearest ground point in view is about 0.998 m camera-forward. A cone base at 1.2168 m is 34.56 degrees below horizontal, 16.76 degrees below the optical axis, and nominally within the configured 0.3-4.0 m accepted depth range. This does not establish valid depth on the mounted camera. The 5 m depth-check station exceeds that configured range.
- The read-only `scripts/system_check.sh` returned **NO-GO** with no VCU serial device, no active ROS graph/camera topics, and no loaded verified flags. It did not open the serial port or request motion.

## Fixed in this handover

- Removed duplicated bumper values from the safety monitor YAML and made the URDF load the shared geometry YAML at Xacro expansion.
- Latched a planner STOP decision in the VCU supervisor until explicit reset. The bridge now requires a fresh supervisor ARMING approval before forwarding ARM and drops stale safety approval. Invalid upstream status no longer counts as fresh stage data.
- Added stationary visibility/depth diagnostics, a guarded braking measurement script with seven staged runs, an integration launch/start/stop pair, and a read-only preflight script. Measurement scripts never write verified flags.

## Still unverified or blocked

- Actual HP60C depth accuracy, RAW16 scale/registration, YOLO cone distance, final mount pitch, target-computer timing, and encoder distance scale. The operator-reported pitch is 17.8 degrees; `calibration/kart_hp60c_2026-10/ground_plane_self_check_2026-10-19.json` estimates 11.4 degrees. Resolve on the installed camera before accepting geometry.
- The handoff says EN/BRK/F-R polarity was user-verified, while `vcu_firmware/include/config.h` still marks the PN2222 output polarity as unverified. Preserve this discrepancy as **VERIFY** before any firmware flash or physical test. Build success is not output-polarity evidence.
- The phase-1 graph was not started because no VCU serial device was present. The start/stop scripts and complete live node chain have therefore not been runtime-qualified. The ROS perception model manifest remains experimental. `brake_min_mps2` is 0 in the launch file, and mapping's live odometry qualification is false.

## Physical-kart order

1. Resolve mount pitch, output polarity and encoder scale; complete stationary camera calibration and run `python3 tools/depth_check.py --frames 30`. Review the JSON and measured error. The 5 m point is outside the current accepted depth range.
2. Before any physical braking test, revise tool prompts that assume an installed E-stop status input. The physical battery/supply disconnect must be within reach with a second observer. No powered test is authorized by this handover.
3. After accepting the evidence, paste the conservative deceleration and measured delay into the planner launch parameters. Set `calibration_verified`, `vehicle_config_verified`, and `braking_verified` true only through the operator's qualification process. Also resolve `live_odom_verified` using measured odometry evidence.
4. Run `scripts/start_phase1.sh` and `scripts/system_check.sh` with the kart held stationary. Require the E-stop input stage amber NOT FITTED, all other required stages ready, VCU SAFE, a fresh `confirm_manual_power_cut` request, and a final preflight GO before requesting ARM.
5. A future approved short cone-lane run would retain the 1.0 m commissioning gap and manual power cut within reach. Tighten toward 0.15 m only when measured depth, latency, odometry, and braking error permit it.

## Deliberate file inventory

Added: `tools/camera_visibility.py`, `tools/depth_check.py`, `tools/test_depth_check.py`, `tools/test_braking_test.py`, `kart/ros2_ws/src/autokart_bringup/launch/phase1_vehicle.launch.py`, `scripts/start_phase1.sh`, `scripts/stop_phase1.sh`, `scripts/system_check.sh`, and this file.

Edited: `tools/braking_test.py`, `docs/BENCH_TESTS.md`, `kart/ros2_ws/src/autokart_nodes/autokart_planner/autokart_planner/mission.py`, `kart/ros2_ws/src/autokart_vehicle_geometry/autokart_vehicle_geometry/geometry.py`, `kart/ros2_ws/src/autokart_vehicle_geometry/package.xml`, `kart/ros2_ws/src/autokart_vehicle_geometry/test/test_vehicle_geometry.py`, `kart/ros2_ws/src/autokart_description/urdf/autokart.urdf.xacro`, `kart/ros2_ws/src/autokart_safety_bridge/config/front_safety_envelope_monitor.yaml`, `kart/ros2_ws/src/autokart_vehicle_bridge/autokart_vehicle_bridge/core.py`, `kart/ros2_ws/src/autokart_vehicle_bridge/autokart_vehicle_bridge/node.py`, `kart/ros2_ws/src/autokart_vehicle_bridge/test/test_core_and_protocol.py`, `kart/ros2_ws/src/autokart_vcu_safety/autokart_vcu_safety/core.py`, `kart/ros2_ws/src/autokart_vcu_safety/autokart_vcu_safety/node.py`, `kart/ros2_ws/src/autokart_vcu_safety/test/test_safety_core.py`, and `vcu_firmware/src/main.cpp`. Generated build products remain under `.ros_phase1_overlay/` and `vcu_firmware/.pio/`; existing build/install trees and audit/evidence folders were not moved or deleted.
