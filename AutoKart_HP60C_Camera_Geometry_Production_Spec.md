# AutoKart HP60C Camera + Dynamic Geometry — Engineering Implementation Contract

**Scope:** Existing `/home/mark1/Desktop/autonomous_kart` only. Camera, perception, metric geometry, motion estimation, camera calibration UI, dynamic centerline and diagnostic dashboard. **Physical actuation/VCU/E-stop are out of scope and remain disabled.**

**Repository caution:** `~/Desktop/autonomous_kart` has previously been observed NOT to be a Git repository itself; some contained directories have their own `.git`. Inspect actual Git roots before creating branches/worktrees. Do not `git init` the parent, move directories, or overwrite user changes.

**Verdict today:** PROTOTYPE TEST REQUIRED. Production-quality implementation is possible as a goal; the measured calibration, dynamic physical test and system qualification are still missing.

## 0. Evidence baseline (15 Oct 2026)

Read `/mnt/data/AutoKart_Engineering_Work_Log.md` if attached to the working environment, or the equivalent existing project report. Relevant current repository locations:

- `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/camera.py` — HP60C acquisition.
- `.../native/hp60c_bridge.cpp` — manufacturer camera bridge.
- `.../src/autokart_perception/detector.py` — YOLO26s ONNX adapter.
- `.../src/autokart_perception/depth.py` — depth localization.
- `.../src/autokart_perception/mount.py` — extrinsic coordinate conversion.
- `.../src/autokart_perception/pipeline.py`, `sdk.py`, `application.py` — shared geometry.
- `.../src/autokart_perception/live_mission.py` — live server and frame snapshot.
- `.../src/autokart_perception/data/perception.yaml` — canonical configuration.
- `phase_sim/src/live/LiveMissionControl.tsx` — dashboard.
- `autokart_apps/run_live.sh` — launcher.

The current live detector is YOLO26s; previous work demonstrated synthetic Delaunay/Gabriel/gates/centerline but **zero qualified LIVE metric positions**, because camera-to-base-link calibration and RAW16 depth verification were not completed. Earlier work identified `CONE_FOOT_CLIPPED`; it now emits provisional body samples instead of discarding all clipped detections. At least 139 Python and 32 UI tests were claimed by Codex; verify actual current tests. Existing 20-second YOLO26s evidence: inference p50/p95 127.8/139.3 ms; frame age p50/p95 286.3/354.1 ms. `max_frame_age_ms=150` conflicts with this performance: neither the setting nor the measured performance may be disguised. Bench cone geometry: three rows beginning 0.84m forward spaced 0.30m, lane width 0.58m. Operational width constraint 2.0–3.8m must remain unchanged; use a separate bench profile.

## 1. Definition of done (deliver a real application, not a mockup)

1. One click/tab opens a **Camera Calibration** window in the existing Mission Control app, with live synchronized RGB + depth, metric reference captures, measurement entry, calibration estimation, quality reports, save/load and rollback.
2. With the real HP60C connected, live frame-specific YOLO26s detections and calibrated metric cone positions propagate into the existing geometry SDK. Never inject synthetic observations into LIVE.
3. When the real camera is translated/rotated, **local** cone coordinates, Delaunay, Gabriel, gates, centerline and lookahead update on each valid frame. For a **persistent world map**, require an independently validated camera motion estimate; clear/freeze and label the map if tracking is lost.
4. The dashboard shows source=LIVE/REPLAY/SIMULATED; calibration status; transform revision; frame age; stage validity; per-cone source/uncertainty/rejection; and real latency. Unsafe outputs are clearly INVALID, not merely hidden.
5. All changes use the existing SDK, no alternate geometry implementation, no vehicle commands, no hidden config overrides, no silent version upgrades, no fake passes.
6. Reproduce with test commands, evidence JSON, before/after metrics, changelog and rollback. Failure means explicit NO-GO.

## 2. Coordinate-system contract — critical separation

- `camera_color_optical`: X right, Y down, Z forward, with actual HP60C color intrinsics and verified RGB-depth registration. Do not assume the raw depth frame shares the same intrinsics unless proven.
- `base_link`: X forward, Y left, Z up; origin and axis references must be declared and physically marked.
- `odom` or `map`: a time-varying world-fixed frame available only when actual pose estimation is qualified.
- **Static camera mount**: `T_base_link_camera_optical = {x_m, y_m, z_m, roll_deg, pitch_down_deg, yaw_deg}` and calibration metadata. Nominal initial z=0.300m, pitch down=4°, x/y/roll/yaw UNKNOWN until measured. Distinguish `pitch_down_deg` convention from signed Euler convention.
- **Dynamic camera motion**: `T_odom_base_link(t)` or local incremental `delta_T(t)`; never store this in mount calibration fields. Moving the camera does not move physical world cones. Do not accumulate cones across timestamps without pose estimate and data association.
- Every downstream observation includes `frame_id`, `camera_sequence`, monotonic capture timestamp `timestamp_ns`, `source`, `transform_revision`, position units, covariance/uncertainty, `qualified` and `reasons`.

## 3. Calibration window (actual behavior and persistence)

Build within existing dashboard; route/modal `Camera Calibration`, not a separate webapp. Responsive, editable forms with units, undo, Save Draft, Validate, Activate, Rollback and Export Report. Access limited to authorized diagnostic operators; calibration changes logged. Show current vs proposed values and risk warnings. Changes must never auto-authorize motion.

**Panel A — Device and live capture:** Camera identity/serial (if exposed), resolution, firmware/SDK/driver version, registered mode, intrinsics `fx,fy,cx,cy`, distortion coefficients if available, raw depth unit, depth validity, paired capture IDs, frame freshness and stage latency; show sample pixels and reprojection overlay. Save image/depth/metadata atomically per calibration capture.

**Panel B — Fixed mount survey:** Inputs `x_forward_m`, `y_left_m`, `z_height_m`, `roll_deg`, `pitch_down_deg`, `yaw_deg`. Start with operator-provided z=0.300m and pitch down=4° marked UNVERIFIED, do not secretly treat x/y/roll/yaw=0 as measurements. Provide textual instructions for locating optical center and base_link origin, measuring using tape/inclinometer, and recording instrument accuracy. Offer mounting-offset solver only with adequate non-collinear independently measured references.

**Panel C — Depth verification:** Capture targets at 0.5, 1, 2, 3m and left/center/right if all ranges are valid; check supported sensor distance/field of view. Record target ID, measured base-frame x/y/z, corresponding pixel or region, observed depth median/MAD/valid fraction and residual. Estimate scale/offset only if supported by evidence; preserve nominal RAW16 *0.001 conversion as unverified until compared with references. Show errors and outliers, reject clipped/ambiguous points from ground-contact calibration.

**Panel D — RGB-depth alignment:** Inspect edge-alignment overlays, reject pixel pairing offset or incorrect resolution/ROI transforms. Align via vendor intrinsics/extrinsics or SDK registered stream, and verify observed residual. Registration flag is not a substitute for physical alignment test.

**Panel E — Calibrate/validate:** Produce versioned JSON/YAML config tied to camera identity, image mode, intrinsics/depth revision, mount revision, independent reference captures, operator and timestamp, SHA-256, fit/held-out residuals, error statistics and reviewer. Status flow: `UNVERIFIED → DRAFT → BENCH_VERIFIED → ACTIVE_DIAGNOSTIC`; rejected/expired entries cannot become active. Require independent held-out points; allow manual offsets only as DRAFT until verified. Validate with explicit p50/p95/max metric x/y error; have QA propose target/error budget grounded in bench geometry and sensor uncertainty before acceptance. Save atomically; restart/switchover at frame boundary; rollback last known verified config after failed validation.

**Panel F — Calibration health:** Per-frame checks for calibration identity, camera resolution, intrinsics change, mounting movement, alignment, depth variance, pose drift, timestamps and unit mismatch. Degrade/clear qualified path on fault; allow amber provisional diagnostic map.

## 4. Data and processing architecture

`HP60C RGB + depth (same frame IDs/timestamps)` → `frame validator` → `YOLO26s existing ONNX` → `depth quality sampling inside cone ROI` → `class + bounding-box + depth association` → `optical XYZ + uncertainty` → `static mount transform into base_link` → `qualification gate` → `existing Delaunay → Gabriel → boundary inference → width checks → gate ordering → dynamic centerline → Pure Pursuit diagnostic candidate` → `atomic JSON+frame snapshot` → `Mission Control`.

- LIVE branch only accepts real frame source. SIM/REPLAY explicitly labeled and never reclassified LIVE.
- Reject stale/out-of-order/unsynchronized frames, empty/invalid intrinsics, invalid RAW16 values, low-depth-coverage regions, inconsistent depth surfaces, excessive positional covariance, improbable cone dimensions, wrong/missing class data, and unverified calibration.
- Clipped cones may provide a `PROVISIONAL_BODY_SAMPLE` (and uncertainty) but cannot impersonate ground contact.
- Unknown/orange-class cones must not silently become blue/yellow track boundaries. Class confusion should block or downgrade planning as applicable.
- Correct geometry can legitimately return NO_PATH for indoor width 0.58m while operational limits are 2.0–3.8m. Use explicit BENCH_DIAGNOSTIC profile only in bench tests, no auto-selection based on detecting a narrow scene.
- Never force graph edges/gates when the true geometry is insufficient.

## 5. Moving camera support

**Level 1 — Mandatory before moving-camera release:** frame-by-frame local metric geometry from current qualified observations; no cross-frame map. Camera movement changes relative coordinates naturally. Transform correct camera optical XYZ to base_link. Show current timestamp and camera orientation, not an assumed absolute world trajectory. Use bounded temporal display smoothing only for graphics, never hide invalid frames or silently smooth planner geometry.

**Level 2 — Persistent map (separate release gate):** integrate a real RGB-D visual odometry / fiducial or surveyed external pose source; provide x, y, heading and covariance over time. Test against a measured track and camera path with rotations and translations. Associate cones using predicted location, class, gate and uncertainty. Do not double-count old observations, loop-close without evidence, or drift unbounded. On odometry confidence loss reset to current-frame local mode and invalidate accumulated world map. If no reliable pose source is available, document LOCAL_ONLY rather than fabricating a world map.

State machine: `NO_CAMERA`, `BOOTSTRAPPING`, `UNCALIBRATED`, `CALIBRATING`, `LOCAL_READY`, `TRACKING_LOCAL`, `TRACKING_WORLD`, `DEGRADED`, `STALE`, `FAULT`. Define entry, allowed layers, data validity, timeout and recovery for each.

## 6. Existing dashboard contract

One persistent camera/depth pair + large metric bird's-eye panel, and a **Calibration** tab/window. Bird's-eye layers: color-coded verified cone ground positions, amber provisional samples, uncertainty ellipses, Delaunay edges, Gabriel edges, colored boundaries, gates, ordered centerline, lookahead and local camera origin. Select source LIVE/REPLAY/SIM with mutually exclusive clear badges; show explicitly `NO_VALID_GEOMETRY` reasons, including camera calibrations, invalid pose, width/insufficient pairs and timing.

Use atomic single-frame snapshots and matching visual assets. No independent independently polling data sources that can mix frame IDs; reject old browser responses; bounded one-frame mailbox; no blanking between valid frames. Show capture rate, inference FPS, geometry FPS, dashboard FPS, capture-to-display latency p50/p95, frame age, raw-valid-depth %, diagnostic candidates and qualified positions. No hardcoded FPS or simulated positions labeled real.

## 7. Agent ownership (avoid integration conflicts)

- Lead/integrator: read-only audit, interface contract, task board, branch strategy, final merge, acceptance review. No broad rewrite.
- Developer 1 (camera/calibration): `camera.py`, native bridge if required, `mount.py`, calibration models/API/config and unit tests.
- Developer 2 (metric perception): `depth.py`, `pipeline.py`, detector compatibility and unit tests; coordinate with developer 1 on schema only.
- Developer 3 (dynamic geometry/motion): SDK adapter/tracker/geometry validation and tests; preserve Delaunay/Gabriel implementations unless a reproduced defect is found.
- Developer 4 (frontend): existing `LiveMissionControl.tsx`, calibration UI and tests; consume frozen API contract.
- Independent QA: synthetic/replay/live evidence, test failures, timing, calibration regression and acceptance report; reject weak evidence.

Use isolated branches/worktrees. No two agents edit same file concurrently. One integrator must own any shared schema/API changes; freeze interfaces before concurrent editing. Agents only claim tests they actually run. Report branch/commit, changed files and rollback.

## 8. Test plan and gates

**T0 — Baseline:** record current `git status`, test counts, running camera service, model checksum and script invocation, preserve rollback. Do not delete uncommitted changes; do not launch duplicate camera owner.

**T1 — Synthetic geometry:** given 3+ ordered color-consistent cone pairs, validate expected Delaunay/Gabriel/gates/centerline, correct signed local-frame transforms, and rejection on invalid widths/ambiguous classes. No phantom cones; regression snapshots bitwise/semantically stable.

**T2 — Stationary surveyed calibration:** real camera, physically measured positions, at least 0.5,1,2,3m where depth valid, left/center/right, record p50/p95/max x/y error; also depth-to-ground ROI residual and optical-to-base frame consistency. Show real corresponding IDs. Proposed initial diagnostic target: **p95 horizontal position error ≤ 0.10 m within 0.5–3 m**, but this is a provisional engineering target, not vendor spec or safety sufficiency; mark unmet or revise with written justification based on actual track requirements.

**T3 — Physical camera movement:** move camera +0.30 m forward, +0.20 m left, rotate +10° and -10°; independently measure the movements; compare predicted observation changes against the tape/angle truth. At least 30 repeated matched observations per motion and report p95 transform residuals and dropout. Without externally measured/estimated pose, evaluate per-frame local geometry only and do not claim persistent world tracking.

**T4 — Dynamic track:** stationary track with known widths within selected profile; hand-move camera; show stable ordered gates and centerline, no false path during partial visibility; record path centerline deviations from surveyed centerline and invalid reasons. Functional test should include cone entry/exit, occlusion, class confusion and narrow BENCH_DIAGNOSTIC profile.

**T5 — End-to-end 10-minute run:** correct RGB/depth frame pairing, bounded memory, no duplicate camera processes, no JavaScript errors, p50/p95 of inference and capture-to-display, valid vs invalid geometry counts. Prior measured 150ms freshness threshold is not achievable with current p95 age ~354ms; **never raise freshness limits silently**. If performance cannot meet a justified deadline, fail the release gate and supply profiling data.

**T6 — Fault injection:** stale/missing/out-of-order RGB-depth, camera unplug/replug, malformed model output, zero cones, invalid intrinsics, depth-scale error, bad mount calibration, pose tracking loss, duplicate frame IDs, browser pause/reconnect. Qualified geometry clears or invalidates deterministically and never authorizes motion.

**Exit gates:** T1 green, T2 measured and within approved budget, T3 measured or clearly LOCAL_ONLY, T4 live geometry evidenced, T5 latency/freshness criteria met, T6 faults fail closed, all regression tests green, QA independent signoff. If any mandatory gate fails: **NOT PRODUCTION QUALIFIED**.

## 9. Execution and reporting

**Order:** (1) baseline and contract; (2) calibration UI/backend plus real depth/pose qualification; (3) local geometry; (4) moving camera local; (5) persistent map optional; (6) performance/stability; (7) QA release. Every stage ends with executable commands, artifacts, screenshot/video evidence, test results and remaining blockers. Continue to the next unblocked task, but never conceal incomplete physical measurements. Ask operator only for unavoidable physical tape/angle measurements.

**Final output:** one concise report with `PASS / FAIL / BLOCKED`, root causes, exact changed files, executable launch command, calibration values + evidence, measured x/y residuals, FPS/latency, moving-camera path evidence, regression tests, rollback and one next human physical measurement. Do not assert vehicle production readiness. All vehicle authority remains OFF.
