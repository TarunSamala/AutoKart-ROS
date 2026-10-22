# AutoKart HP60C Calibration — Procedure and Current State (2026-10-19)

**Status: UNVERIFIED.** The software is ready; the physical survey has not been done. Nothing below qualifies metric geometry until `calibration_session evaluate` passes on held-out stations that a human measured.

## What was built / found this session
| Item | Where | Notes |
|---|---|---|
| Kart mount config (reported values, UNVERIFIED) | `calibration/kart_hp60c_2026-10/perception_kart_unverified.yaml` | Replaces the bench default (0.300 m, 4°) which was wrong for this kart. Does not qualify anything. |
| Ground-plane self-check | `autokart_perception.calibration_groundcheck`, tests `tests/test_calibration_groundcheck.py` (8) | Fits the floor in real depth; compares to reported mount. Evidence only — hard-coded `qualified:false`. Pitch/roll are depth-scale independent; height is not. |
| Real self-check result | `calibration/kart_hp60c_2026-10/ground_plane_self_check_2026-10-19.json` + `scene_at_selfcheck.jpg` | 20 live frames, plane found in every analysed frame. |
| Survey plan + blank stations file | `survey_plan.json`, `stations_template.json` | 8 stations (4 fit + 4 validate) × 3 cones, every measured value `null`. |

## Real measurements so far (live HP60C, vendor SDK intrinsics)
- Intrinsics (SDK, not independently verified): fx 585.27, fy 585.02, cx 335.98, cy 239.96, 640×480.
- Ground plane in depth: **height 0.820 m** (reported 0.8382 m, Δ −1.8 cm), **pitch 11.4° down** (reported 17.8°, **Δ −6.4°**), **roll 1.5°**. Frame-to-frame std 0.25 cm / 0.08° / 0.09°.
- **Unresolved conflict:** pitch disagrees by 6.4° (tolerance 2°). Either the 17.8° reference was taken differently (e.g. bracket vs optical axis), or the camera moved since. The scene in `scene_at_selfcheck.jpg` is a garage floor with cones, so the plane is very likely the floor — but a flat plane is not proof. **Measure with an inclinometer on the optical axis before the survey.** The height agreement (within 2 cm) mildly suggests the 0.001 m/raw depth scale is about right (implied factor 1.022), but that is confounded with the tape measurement.
- Visibility (same under both pitches): ground is not visible nearer than ~1.05 m in front of the camera, so 0.5 m and 1.0 m grid stations are unmeasurable. Measurable stations are 1.5, 2.0, 2.5, 3.0 m forward of the origin = 0.43, 0.93, 1.43, 1.93 m ahead of the reported bumper — bracketing the 1 m clearance boundary (origin x ≈ 2.07 m). This alternative grid needs a named human approval (`approved_grid.approved_by`, `reason`).

## Survey procedure (needs you at the kart; ~1–2 h)
Work in `calibration/kart_hp60c_2026-10/`. Copy the template; never edit the original.

1. **Define the frame.** Mark the ground point directly below the optical centre (plumb line). X forward along the kart centreline, Y left. Mark the front-most bumper point on the ground; measure its X (reported 1.0668 m). Write the definition in `base_link_definition`.
2. **Measure the mount** with named instruments: optical-centre height, X/Y offsets, pitch (inclinometer on the optical axis, not the bracket), roll, yaw. Enter in `mount_measurement` with realistic `uncertainty` (`translation_m`, `angle_deg`).
3. **Re-run the self-check** with the measured values: `python3 -m autokart_perception.calibration_groundcheck --mount <json> --frames 20 --out <file>`. Expected: `CONSISTENT_WITH_REPORTED`. If not, resolve before continuing.
4. **Approve the grid:** fill `approved_grid.approved_by` and `reason` (needed because 0.5/1.0 m are out of view).
5. **Place cones and measure each foot** with a tape/laser from the marked origin, filling `x_m`, `y_m`, `sigma_m` for every cone in all 8 stations. Fit stations and validate stations must be *different physical placements*. Include a station near x ≈ 2.07 m.
6. **Choose the tolerance before looking at results** (≤ 0.15 m enforced; the HP60C spec proposes p95 ≤ 0.10 m over 0.5–3 m as a provisional engineering target).
7. **Capture** (camera must be free — stop `scripts/run_dashboard.sh`):
   `python3 -m autokart_perception.calibration_session capture --stations <stations.json> --output <capture_dir> --camera-library <lib> --camera-configuration <cfg> --model models/yolo26s/best.onnx --manifest models/yolo26s/best.json --config perception_kart_unverified.yaml`
8. **Evaluate:** `python3 -m autokart_perception.calibration_session evaluate --capture <capture_dir> --stations <stations.json> --tolerance-m <chosen> --artifact <new_artifact.json>`
   PASS is decided only on held-out validation stations; the artifact is written only on PASS and refuses to overwrite.
9. Load the artifact (`calibration_artifact` parameter / `--config`) and confirm the dashboard shows the calibration state change. Depth scale also needs its own independent verification (artifact `depth_scale_factor`).

## What stays blocked until then
`MOUNT_VERIFIED` and `DEPTH_VERIFIED` cannot be produced by any software here: `mount_from_config` never returns more than `MOUNT_CONFIGURED_UNVERIFIED`; the self-check always emits `qualified:false`; the stations loader rejects any file with an unmeasured value (verified in `docs/ui_split/VALIDATION_MATRIX.md` follow-up). Consequently the dashboard bird's-eye keeps showing "NO QUALIFIED LIVE GEOMETRY" for live data, and `bumper_setback_m` in the safety monitor stays PROVISIONAL.

## Known issue found along the way
Live frame processing takes ~280 ms but `max_frame_age_ms` is 150, so the pipeline `sdk_capture` hop can read STALE under the kart config. This pre-dates this work (documented in the HP60C spec); the limit has not been raised, per the no-silent-loosening rule.
