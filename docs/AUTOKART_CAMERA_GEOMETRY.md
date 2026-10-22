# AutoKart Camera Geometry — Audit (Agent 4)

> **CORRECTION (2026-10-18, later):** statements in this document that no HP60C is attached are WRONG or STALE. An HP60C (USB `3482:6723`) is attached and streams RGB+depth at ~11-13 fps via the vendor SDK; the live Vehicle Dashboard was verified against real frames (see `ui_split/VALIDATION_MATRIX.md`). Items below marked NOT TESTED because "no camera" are now testable; those not re-run in `ui_split/VALIDATION_MATRIX.md` remain NOT TESTED. Calibration is still UNVERIFIED and no qualified metric geometry exists.

**Method:** read and tested the actual existing implementation (`mount.py`, `depth.py`) against the master execution prompt's required transform chain. **Finding: the required pixel → camera 3D → vehicle 3D → ground-contact pipeline already exists and is correctly implemented.** This is not a gap to fill; it's an existing, tested, honestly-labeled algorithm, preserved unmodified per the "do not rewrite working algorithms" instruction.

## TASK D5 — pixel → camera 3D → vehicle 3D, verified present

`depth.py:localize_diagnostic()` implements exactly the required chain:

1. **Pixel → camera-optical 3D** (`depth.py:92`): `optical = [(u-cx)*Z/fx, (v-cy)*Z/fy, Z, 1]` — the standard pinhole unprojection, matching the formula in the master prompt precisely.
2. **Camera-optical → vehicle** (`depth.py:105,112`): `xyz = camera_to_kart @ optical`, where `camera_to_kart` is the full 4×4 rigid transform from `mount.py:mount_from_config()` — **rotation built from pitch/roll/yaw** (not just the pitch angle) composed with an explicit optical→kart axis-convention matrix (`[[0,0,1],[-1,0,0],[0,-1,0]]`: optical right/down/forward → kart forward/left/up), **plus translation** `[x_forward_m, y_left_m, height_m]`. This is `R,t`, not a scalar subtraction — satisfies TASK D5's "R and t represent the verified optical-camera-to-vehicle transform, not just the pitch angle."
3. **Ground-plane intersection** (`depth.py:122-138`): separately casts the cone-foot pixel ray through the same transform and intersects it with the vehicle-frame ground plane (`z=0`), then cross-checks that ground candidate against the depth-sampled surface point (`ground_contact_offset_m`, rejected if they disagree by more than `max_ground_contact_offset_m`). This is a real, tested consistency check between two independent position estimates, not a single unverified projection.

## TASK D4 — pitch / sightline-angle / translation kept separate (verified)

`mount.py`'s pitch parameter (`pitch_down_deg`) feeds only the rotation matrix; it is never conflated with a derived sightline angle, and translation (`x_forward_m`, `y_left_m`, `height_m`) is a separate, independent set of config values. No code path anywhere computes or substitutes a "camera-to-point-A sightline angle" for the mount pitch — confirming this session's earlier correction (49.14 in geometry-derived diagonal vs. the 52 in reported, physical-pitch-vs-sightline-angle) was already architecturally consistent with how the code is structured, not something the code itself conflates.

## TASK D6 — bumper-relative clearance

`camera_to_kart`'s translation is `T_base_link←camera_optical` (matching the terminology in `AutoKart_HP60C_Camera_Geometry_Production_Spec.md`), so `localize_diagnostic()`'s output (`cone.xy`) is **already in base_link-frame coordinates**, not raw camera-ray distance. The front safety envelope monitor (`autokart_safety_bridge`, this session's earlier RC-05/06 work) correctly consumes base_link-frame `x` and subtracts a `bumper_setback_m` parameter from it — this is subtracting two values in the *same* frame, not "subtracting 1.0668 m from an arbitrary camera-ray distance," which is what TASK D6 explicitly warns against.

**What remains genuinely unresolved (not a code defect — a physical-measurement gap):** `base_link`'s origin has never been physically marked or surveyed (the production spec doc says so explicitly: "origin and axis references must be declared and physically marked"), so `bumper_setback_m` cannot be upgraded from PROVISIONAL to VERIFIED by writing more code. This was already labeled `UNVERIFIED`/`PROVISIONAL` everywhere it appears (config comments, the `/safety/envelope` message's `limiting_reason` suffix, the dashboard's calibration banner) before this audit, and stays that way. No change made.

## TASK D3 — inconsistent diagonal measurement

Already recorded (RC-05 of this session): 52 in / 1.3208 m reported-measured value preserved and distinguished from the 49.14 in / 1.248 m geometry-derived value, in `kart/ros2_ws/src/autokart_safety_bridge/config/front_safety_envelope_monitor.yaml`'s comments. Neither was silently replaced.

## TASK D7–D10 — guided calibration utility, residuals, MOUNT_VERIFIED gating

Already exists and is tested: `calibration.py`, `calibration_fit.py`, `calibration_session.py`, `calibration_store.py` (covered by `test_calibration*.py`, 27/27 passing alongside `test_mount.py`/`test_depth.py`/`test_geometry.py` in this audit). `mount_from_config()` itself never returns anything stronger than `MOUNT_CONFIGURED_UNVERIFIED` from config alone (`mount.py:20,41`) — it cannot self-upgrade to verified; that requires the separate calibration-session/fit/store path with actual held-out residual checks, matching TASK D10's requirement exactly. Not independently re-verified against new physical survey data in this pass (no hardware) — see Open Issues.

## TASK D11 — metric safety-zone image projection

**Not implemented.** The dashboard overlay built in this session's earlier RC-06 work is explicitly schematic/non-metric (dashed, fixed-fraction bands, labeled "GEOMETRY UNVERIFIED"), by design, because calibration is not verified. A true calibrated projection (apply intrinsics+extrinsics to the bumper/1m-boundary points, clip offscreen geometry) would require `MOUNT_VERIFIED` status, which does not exist without physical survey data. Implementing the projection *code path* now (dormant until a verified calibration exists) is a reasonable next increment — not done in this pass; tracked as an open item.

## Regression

`test_mount.py` + `test_depth.py` + `test_geometry.py` + `test_calibration.py` + `test_calibration_fit.py` + `test_calibration_session.py`: **27/27 PASS**, part of the full `autonomy_stack_phase1/phase_01_perception` 163/163 passing suite (see `AUTOKART_YOLO26S_INTEGRATION.md`).

## Open issues

| Item | Status |
|---|---|
| `base_link` origin physical survey/marking | **NOT TESTED** — physical measurement, no hardware access |
| `bumper_setback_m` upgrade from PROVISIONAL to VERIFIED | **BLOCKED** on the above |
| Calibration residuals against *new* physical survey points | **NOT TESTED** — no camera/surveyed obstacles available |
| TASK D11 calibrated (non-schematic) safety-zone image projection code path | **NOT IMPLEMENTED** — reasonable next increment, deferred |
