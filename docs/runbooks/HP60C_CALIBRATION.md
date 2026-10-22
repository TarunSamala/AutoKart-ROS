# HP60C guided calibration

LIVE cones stay `DIAGNOSTIC_ONLY` until a calibration artifact passes held-out validation. Nothing here moves the vehicle. Stop any running perception node first: the HP60C has exactly one owner.

```bash
cd autonomy_stack_phase1/phase_01_perception
export PYTHONPATH=src
A=$PWD/..
LIB=$PWD/native/build-x86_64-linux-gnu/libhp60c_bridge.so
CFG=$A/vendor/linux_ros/linux/configurationfiles/hp60c_v2_00_20230704_configEncrypt.json
```

1. **Measure the mount.** Mark the base_link origin. With a tape and inclinometer, measure the optical-centre `x_forward_m, y_left_m, height_m, pitch_down_deg, roll_deg, yaw_left_deg` and their uncertainty. Write them to `mount.json`.
2. **Plan the field of view.** The camera reports its intrinsics; the current HP60C gives `585.27 585.02 335.98 239.96`.

   ```bash
   python -m autokart_perception.calibration_session plan --intrinsics 585.27 585.02 335.98 239.96 --mount mount.json
   ```

   Out-of-view cells are listed with measurable alternatives. At 0.30 m height and 4° pitch, the 0.5 m row is not visible; the proposed substitute is 0.65 m.
3. **Write the template.** Add `--approved-distances` only if a reviewer approves a substitute.

   ```bash
   python -m autokart_perception.calibration_session template --intrinsics ... --mount mount.json \
     --output stations.json --approved-distances 0.65 1 2 3
   ```

   Fill in every `null`:
   - operator, instrument and base_link definition
   - the mount measurement and its uncertainty
   - `approved_grid.approved_by` and `approved_grid.reason`
   - each cone's tape-measured `x_m`, `y_m` and `sigma_m`

   `fit` and `validate` stations must be **separate placements**: move the cones and measure again. The tool refuses unmeasured fields.
4. **Capture.**

   ```bash
   python -m autokart_perception.calibration_session capture --stations stations.json --output captures/<date> \
     --camera-library $LIB --camera-configuration $CFG \
     --model ../../models/yolo26s/best.onnx --manifest ../../models/yolo26s/best.json
   ```

   For each station, place the cones as written and press Enter. The preview shows RGB and depth when a display exists. Each station needs 5 accepted frames. These are rejected and counted:
   - stale or unpaired frames, and invalid depth
   - a clipped cone foot
   - no detection near a surveyed cone, or an ambiguous match
   - a changed camera configuration

   The output contains recordings per station, `fit_annotations.json`, `validate_annotations.json` and `capture_manifest.json` (device identity, stations hash, rejections). Boxes come from the detector; check them with the existing annotator (`evaluation.make_template`) if needed.
5. **Evaluate.** Fix the tolerance before testing; it must be ≤ 0.15 m.

   ```bash
   python -m autokart_perception.calibration_session evaluate --capture captures/<date> \
     --stations stations.json --tolerance-m 0.10 --artifact calibration/hp60c_<date>.json
   ```

   `calibration_report.json` gives:
   - fitted depth scale, height and pitch, with 1σ uncertainty
   - held-out position and X/Y error by range
   - rejections and PASS/FAIL

   PASS needs every held-out cone within tolerance, LIVE-only recordings, the full (approved) grid on validation stations and no sync mismatches. The artifact is written only on PASS. Editing `stations.json` after capture is refused.
6. **Use the artifact.** Launch with `calibration_artifact:=<path>`. The dashboard CAMERA CALIBRATION block then shows `VERIFIED` and the measured error. Frames from a camera with different intrinsics show `CALIBRATION_CAMERA_MISMATCH`.

Limits:
- **Hash:** the artifact hash detects accidental edits, not forgery.
- **Fixed parameters:** x/y/roll/yaw stay as measured; only depth scale, height and pitch are fitted.
- **Range:** accuracy outside the surveyed grid is unknown.
- **Model:** LIVE cones also require the model to be qualified (`MODEL_UNVERIFIED` blocks them today).
- **Scope:** the artifact says nothing about odometry, the VCU or braking.
