# AutoKart — HP60C Camera Calibration & Metric Cone Localization

**Document ID:** AK-CAM-CAL-MASTER-2026-10-23  
**Revision:** 1.0 | **Date:** 23 October 2026  
**Scope:** YDLIDAR HP60C depth/RGB → cone ground-contact points → vehicle-frame coordinates → ROS 2 mapping/planning and offline simulator/replay.  
**Disposition:** **REFERENCE GRID DOCUMENTED / LIVE METRIC CALIBRATION NOT VERIFIED / VEHICLE AUTONOMY NO-GO.**

> **Evidence rule:** `REPORTED_BY_OPERATOR` ≠ `MEASURED_AND_VALIDATED`; `CALCULATED_FROM_REPORTED` ≠ physically measured; `DESIGN_TARGET` ≠ pass. Values that are not evidenced must remain `null`/`UNVERIFIED`. Do not create a fake calibration artifact to make the live gate pass.

## 1. Relevant hardware and architecture

| Element | Known configuration | Evidence / caveat |
|---|---|---|
| Primary perception camera | YDLIDAR **HP60C** RGB and RAW16 depth | Reported/tested on Raspberry Pi; vendor USB ID historically `3482:6723`. |
| Host | Raspberry Pi 4B / Ubuntu 22.04 / ROS 2 Humble | Historical deployment; verify running driver/version. |
| Physical image format | Previously reported paired RGB + RAW16 depth, about **640×480** | Confirm exact streams, registration, intrinsics, timestamps and units from active camera driver. |
| Camera detection | YOLO26s and other stored cone models | Detector confidence is distinct from geometric calibration accuracy. Select **one** active model/decoder and record its hash. |
| Intrinsics `fx,fy,cx,cy` | **UNKNOWN IN THIS DOCUMENT** | Must read from actual device/CameraInfo at the active resolution. |
| Depth unit scale | Historical candidate `raw × 0.001` metres | Not independently validated. Must measure; do not assume factory code is a site calibration. |
| Camera height relative to ground | **UNKNOWN** | Measure optical-centre height, not body top. |
| Camera translation relative to kart base | **UNKNOWN** | Survey X/Y/Z with a defined `base_link` origin. |
| Camera yaw, roll, pitch | **UNRESOLVED** | Alignment observations are not accepted extrinsics. |
| Lens-depth registration | **NOT VERIFIED FOR CURRENT RUNTIME** | Verify depth pixel association to RGB bounding boxes before projection. |

The camera should have exactly **one physical device owner** (e.g. `autokart_hp60c`). Other ROS 2 nodes consume timestamped topics; do not have a simulator/dashboard open the HP60C independently while the production driver owns it.

## 2. Actual reported nine-cone survey (19 October 2026)

### 2.1 Survey frame

The original nine-cone grid is expressed in an **operator-declared floor frame**, with `+X = right`, `-X = left`, `+Y = forward`, and nominal centreline `X=0`. This is **not** the standard ROS `base_link` sign convention; explicitly transform between frames. `Y=0` is the operator-designated floor tape datum, **not yet established** as either the vertical projection of the optical centre or the tilted optical-ray/floor intersection. The offset must be measured before estimating extrinsics.

| ID | Station | Position | X right (m) | Y forward (m) | Provenance |
|---|---|---|---:|---:|---|
| FIT-1-L | FIT-1 | Left | -0.60 | 1.75 | Operator tape report |
| FIT-1-C | FIT-1 | Centre | 0.00 | 1.66 | Operator tape report |
| FIT-1-R | FIT-1 | Right | +0.60 | 1.75 | Operator tape report |
| FIT-2-L | FIT-2 | Left | -0.70 | 2.25 | Operator tape report |
| FIT-2-C | FIT-2 | Centre | 0.00 | 2.15 | Operator tape report |
| FIT-2-R | FIT-2 | Right | +0.70 | 2.25 | Operator tape report |
| FIT-3-L | FIT-3 | Left | -0.80 | 2.80 | Operator tape report |
| FIT-3-C | FIT-3 | Centre | 0.00 | 2.70 | Operator tape report |
| FIT-3-R | FIT-3 | Right | +0.80 | 2.80 | Operator tape report |

**Do not force these into perfectly level rows.** Each side-cone pair lies **9–10 cm farther forward** than the centre cone. The side spans are **1.20, 1.40, 1.60 m**. Side-cone approximate planar ranges are **1.850, 2.356, 2.912 m**; these are derived from the supplied tape coordinates and are not sensor readings. FIT targets can fit the model but **cannot independently validate** it.

### 2.2 Camera pitch and mount disagreement

- Operator-reported camera mounting pitch: **18° downward** (19 October report).
- Earlier reported depth-based fit suggestion: **about 11.4°** (not independently verified).
- Earlier iPhone Level observation: **0° on a top surface**, which may not be aligned with the optical axis.
- **Accepted actual optical pitch: UNKNOWN.** Roll is unknown; yaw-to-kart offset is unknown. Camera mounting height is unknown.
- Repeat measurements on a defined physical reference face, record phone orientation/calibration and verify with floor-point residuals. Do not pick 18°, 11.4° or 0° just to satisfy software gates.
- The original report provisionally assumed ±1 cm per tape coordinate and ±1° for angle uncertainty; **those uncertainty bounds have not been demonstrated**.

### 2.3 Potential cone-base offset

The geometry target is the **centre of the cone footprint where it meets the floor**, not the nearest visible face of a cone. A previously discussed **~15 cm cone-base width** was an *example*, not a confirmed current measurement. Differences between near edge and footprint centre can reach several centimetres. Measure and record actual base dimensions/shape, convention and pose; do not apply a guessed fixed 7.5 cm correction.

## 3. Calibration math and transforms

### 3.1 Pixel-to-optical-coordinate projection

For a **registered depth image whose pixel gives optical-axis Z depth**, a valid detection point `(u,v)` and calibrated camera intrinsics:

```text
z = raw_depth * verified_depth_scale_m_per_unit
x_opt = (u - cx) * z / fx
 y_opt = (v - cy) * z / fy
z_opt = z
P_camera_optical = [x_opt, y_opt, z_opt, 1]
```

The leading space on the `y_opt` line is cosmetic. Camera optical convention is commonly `+X right, +Y down, +Z forward`; confirm driver frame definitions. **Some depth products are radial range rather than optical-axis Z; the active HP60C representation must be confirmed before this formula is applied.** Invalid/zero/saturated depth must be rejected, never interpreted as a zero-distance obstacle or zero-error sample. A bounding-box centre is not necessarily the floor-contact pixel.

### 3.2 Depth sampling and ground-contact extraction

1. Accept only synchronized or explicitly registered RGB/depth frames. Validate intrinsics match the image resolution, and log actual frame stamps and age.
2. Run the selected YOLO detector; retain class, confidence, bounding box, model version/hash.
3. Select a **geometry-consistent cone-foot candidate**, using the lower body/footprint neighbourhood or a validated ground-plane intersection; reject pixels on walls or behind the cone.
4. Estimate robust depth from a small supported region (e.g., median of valid samples), with support count, spread and uncertainty. This sampling rule is a **proposal** until it passes data review.
5. Convert detected 3-D camera points through a **measured** static transform `T_base_link_camera_optical` and, if required, the observed ground plane.
6. Convert to the measured cone footprint centre, accounting for base geometry and viewpoint; keep any offset correction traceable and data-derived.
7. Tag the cone with `FRAME_ID`, acquisition timestamp, RGB/depth pairing IDs, pixel/depth statistics, XY in `base_link`, covariance, `calibration_id`, and `validity_reason`.

ROS `base_link` convention: `+X forward`, `+Y left`, `+Z up`. The nine-cone survey uses `+X right`, `+Y forward`; without additional offsets and rotations, the rough axis relationship is `base_x ≈ survey_Y`, `base_y ≈ -survey_X`, but this is **NOT** a complete physical transform and must not replace measured extrinsics.

### 3.3 Error function / fitting protocol

Use the nine **individual** surveyed floor points, not idealised rows. Fit physically meaningful variables such as depth scale/correction and camera pose (height, translation, pitch, roll, yaw); constrain model complexity to prevent nine-point overfitting. Compare sensor-estimated footprint centre `(X_cam,Y_cam)` against tape-measured `(X_tape,Y_tape)` in **the same frame**:

```text
ex = X_cam - X_tape
ey = Y_cam - Y_tape
e_xy = sqrt(ex**2 + ey**2)
```

Report per-cone `ex`, `ey`, `e_xy`, missing/rejected observations, median/mean/max, bias by left/centre/right and error vs range. Fit only using `FIT-*`, freeze the model/hash **before** observing `VAL-*` performance.

## 4. Independent validation — required, not yet done

Obtain **three independent validation stations**, with three cones per station if possible, at **different, tape-measured** points from the nine FIT points. Earlier suggested station ranges were approximately **1.7 m, 2.2 m and 2.75 m**, but their exact positions were **never accepted as measured ground truth**. Record `VAL-1-L/C/R`, `VAL-2-L/C/R`, `VAL-3-L/C/R` only after new tape measurements.

| Ground-plane XY error | Proposed classification | Acceptance disposition |
|---|---|---|
| `e_xy ≤ 0.10 m` | Target achieved for that point | Preferred result |
| `0.10 m < e_xy ≤ 0.15 m` | Within maximum tolerance | Conditional review needed |
| `e_xy > 0.15 m` | Outside maximum tolerance | FAIL |
| No valid detection/depth | Availability failure | Do not record zero error |

**Approval policy from 19 October report:** every usable independent validation point must be **≤0.15 m**, target **≤0.10 m**, publish worst case and all exclusions. Meeting this geometry target **alone** does not approve braking or autonomous motion.

**Evidence package** for each fit/validation run:

```text
camera_calibration_runs/<run_id>/
  metadata.json             # host, camera serial, firmware, acquisition config, timestamp
  camera_info.yaml          # fx, fy, cx, cy, distortion, resolution
  survey_points.csv         # ID, X, Y, tape datum, uncertainty, cone-base convention
  rgb/                     # raw paired RGB observations
  depth_raw16/             # actual raw depth, format/scale stated
  rgb_depth_pairing.csv     # timestamps, frame indices, offsets
  detections.csv           # model, box, confidence, class
  projected_cones.csv      # footprint XY, covariance, validity, reason
  fit_parameters.yaml      # fitted parameters, units, bounds, model version
  independent_validation.csv
  error_metrics.json       # per-cone, median, mean, max, missing-data rates
  qa_report.md             # who measured, what failed, approver/decision
  sha256sums.txt
```

## 5. Known experimental limitation from October 12

A preliminary HP60C report recorded approximately **1,110 received frames over two runs** (~**12.4 Hz per run**). In nominal **90 → 80 → 90 → 100 cm** position sequences, one test run had a **−104 raw-unit** shift between initial and returning 90 cm medians, while another had **+2.5 raw units**. Transport timestamps were monotonic and sampled central 40×40 regions met the logger's pixel-validity rule. **Interpretation:** frame reception and nonzero pixels are not sufficient proof of stable metric depth; the root cause was not demonstrated. Do not convert these raw deltas into metres without a verified scale.

## 6. Proposed calibration config format (NOT an approved artifact)

```yaml
schema_version: 1
sensor: HP60C
source: LIVE_PHYSICAL
status: NOT_VERIFIED
calibration_id: null
intrinsics:
  width: 640                # historical stream; verify active setting
  height: 480
  fx: null
  fy: null
  cx: null
  cy: null
  distortion: null
raw_depth:
  scale_m_per_unit: null    # historical 0.001 is an unverified candidate
  is_optical_axis_z: null
  aligned_to_rgb: null
extrinsics_to_base_link:
  x_m: null
  y_m: null
  z_m: null
  roll_rad: null
  pitch_rad: null
  yaw_rad: null
survey:
  convention: X_RIGHT_Y_FORWARD
  tape_y_zero_defined: false
  measured_cone_footprint_centre: false
  fit_points_count: 9
  independent_val_points_count: 0
validation:
  target_xy_m: 0.10
  max_xy_m: 0.15
  max_observed_xy_m: null
  total_latency_p95_ms: null
  reviewer: null
  approval_timestamp: null
provenance:
  raw_data_manifest_sha256: null
  model_sha256: null
  runtime_driver_git_sha: null
```

### File/logic requirements

- Reject `LIVE_PHYSICAL` map qualification if essential intrinsics/extrinsics/depth conversion or calibration status is invalid, absent, stale, or inconsistent with active camera identity/resolution.
- Keep `REPLAY` and `SYNTHETIC` independent of physical-live qualification. Simulation may use a declared synthetic camera model; it must **not** be called measured/live.
- Version and hash every calibration artifact; never silently overwrite old coefficients or modify safety gates to accept missing values.
- Qualify cones independently from the mission type: skidpad uses loop geometry; braking corridor uses boundary/obstacle/stop geometry.
- Validate stability under vibration, turn, mixed lighting, occlusion, camera remount and temperature drift; a stationary bench pass is not dynamic validation.

## 7. ROS 2 and simulator integration contract

```text
HP60C camera driver (only opener)
  -> stamped RGB + RAW16/depth + CameraInfo
  -> autokart_perception (active YOLO model + depth association)
  -> validated 3-D footprint points in base_link
  -> autokart_mapping / autokart_mission / autokart_planner
  -> dashboard: actual overlay + bird's-eye cones + source/age/uncertainty
```

### Minimum structured cone observation

```yaml
header: {stamp: '<sensor_acquisition_time>', frame_id: base_link}
observation_id: null
model_id: null
class_label: null
class_confidence: null
rgb_frame_id: null
depth_frame_id: null
depth_support_pixel_count: null
x_m: null
y_m: null
covariance_xy: null
calibration_id: null
source: LIVE_PHYSICAL  # or REPLAY, SYNTHETIC
valid: false
reason: CALIBRATION_UNVERIFIED
```

Do not assume these exact YAML keys match the current ROS `.msg` definitions. Audit the repository and adapt to the **existing** message schema. The simulator should load the **same calibration-derived transforms and detection logic** when replaying camera captures; synthetic scenes are allowed separate explicitly synthetic ground truth.

## 8. Unresolved tasks and release gates

- [ ] Confirm the exact physical camera model/serial, resolution, depth encoding, CameraInfo, synchronized pairing and active firmware/driver.
- [ ] Define floor tape datum unambiguously; resurvey cone footprint centres and tape uncertainty.
- [ ] Physically measure camera optical-centre height and `base_link` translation.
- [ ] Resolve 18° vs 11.4° vs 0° pitch observations and measure yaw/roll.
- [ ] Measure cone base geometry and footpoint extraction bias.
- [ ] Derive/freeze scale/extrinsics from **nine fit points** without using validation targets.
- [ ] Survey **new** VAL-1/2/3 targets, collect raw paired frames and publish per-cone validation errors.
- [ ] Characterise sensor/decode/detector/fusion latency p50/p95/p99 in actual Pi workload.
- [ ] Test target visibility, false cone detections, dropout and motion effects.
- [ ] Reviewer signs calibration artifact and deploys it only with matching camera/resolution/model identifiers.
- [ ] Live perception remains **unverified/no-go** until requirements pass; vehicle release needs separate dynamics, brake and hard-stop evidence.

## 9. Original sources / provenance

- `AutoKart_HP60C_Camera_Calibration_Engineering_Report_2026-10-19.pdf` / `.docx`, report `AK-CAM-CAL-019`, 4 pages (nine measured cones, geometry and independent-validation policy).
- `AutoKart_Research_Paper_Preliminary_2026-10-13.docx` (October 12 depth repeatability evidence).
- `AutoKart_Project_Detail_Complete.md`, `AUTOKART_12_NODE_MASTER_ARCHITECTURE.md`, and recent camera calibration discussions.

**Final engineering statement:** The nine-cone grid is suitable for a fit, not proof of an accurate or approved live perception sensor. No fabricated calibration residuals, mount data or independent validation is included.
