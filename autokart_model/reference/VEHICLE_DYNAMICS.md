# AutoKart — Vehicle Geometry, Dynamics & Actuation Calibration Master

**Document ID:** AK-VEH-DYN-MASTER-2026-10-23  
**Revision:** 1.0 | **Date:** 23 October 2026  
**Purpose:** Exact traceable mechanical/vehicle parameters; drivetrain encoder scaling; bicycle/Pure Pursuit simulation; steering, BLDC, brake and odometry calibration; ROS/ESP32 integration.  
**Release status:** **PROVISIONAL VEHICLE MODEL; HARDWARE CALIBRATION AND BRAKING NOT SIGNED OFF; AUTONOMOUS MOTION NO-GO.**

> This is an engineering baseline for the existing AutoKart repository and a configurable simulator, not a claim that physically tested control gains or stopping capability exist. Never promote a historical or simulated value to a `MEASURED_PASS` field without traceable evidence.

## 1. Present system ownership (latest 22 October baseline)

```text
Raspberry Pi 4B / ROS 2 Humble
  ├─ HP60C live/replay cone detection, mapping, mission, planner
  ├─ MPU-6500 IMU and wheel–IMU odometry fusion
  ├─ Pure Pursuit + speed/stop candidate
  └─ software safety authority -> bounded target requests
            |  USB serial / agreed ROS bridge; target profile 50 Hz
            v
ESP32-S3 VCU / MicroPython
  ├─ AS5047P 14-bit SPI encoder at 14T BLDC sprocket
  ├─ AS5600 steering angle I²C (0x36)
  ├─ AS5600 brake angle I²C (0x36; channel separation required)
  ├─ BLDC/BLD-750 via GP8630N DAC and PN2222 logic
  ├─ NEMA34/R86mini steering
  ├─ NEMA23/TB6600 brake
  ├─ local command timeout and actuator fault handling
  └─ feedback/validity/events -> Pi

Independent hardwired E-stop + traction inhibit / safe braking path
  └─ REQUIRED, but physical qualification remains unverified.
```

**Important sensor correction:** the AS5047P uses **SPI**, not historical ABI; both AS5600 devices use **I²C**, not historical PWM. Two AS5600s share fixed address **0x36** and require separate buses or mux channels (a later validation plan allocated brake to a **PCA9548A channel 1**). Audit the real VCU harness before accepting channel/pin assignments. Not all earlier MicroPython/ESP-IDF proposals describe the same installed firmware.

## 2. Mechanical geometry and conflicting revisions

These are **reported dimensions**, not an as-built signed survey. Values from the October 16 updated-measurements discussion are preferred as **provisional**, but newer physical test evidence is required before live use.

| Parameter | Provisional value | Prior/conflict | Disposition |
|---|---:|---|---|
| Wheelbase (front axle centre to rear axle centre) | **1117.6 mm / 1.1176 m** | Older simulation constant **1.05 m** was **not measured** | `REPORTED; RE-MEASURE` |
| Front track centre-to-centre | **1066.8 mm** reported spreadsheet | Mechanical report **38 in = 965.2 mm** centre-to-centre; **43 in = 1092.2 mm** outside-to-outside | `CONFLICT: ±101.6 mm between centre reports` |
| Rear track centre-to-centre | **1181.1 mm / 1.1811 m** | Historical separate rear-axle length **1026 mm** (not track/wheelbase) | `REPORTED; RE-MEASURE` |
| Chassis frame length | **1422.4 mm** | Not full vehicle swept length | `REPORTED` |
| Chassis frame width | **660.4 mm** | Not full tyre outer-to-outer width | `REPORTED` |
| Ground clearance | **38.1 mm** | Unloaded/loaded context not established | `REPORTED` |
| Front tyre nominal size | **10 × 4.5 - 5**, SM-68 | Earlier diameter **240 mm**; size notation yields nominal **254 mm**, 114.3 mm width | `CONFLICT / VERIFY LOADED` |
| Rear tyre nominal size | **11 × 7.10 - 5**, BKT SM 68 | Earlier **295 mm** diameter; tyre notation gives nominal **279.4 mm** | `CONFLICT / MEASURE LOADED CIRCUMFERENCE` |
| Tie rod | **290 mm** later report | Earlier steering rod figure **611 mm** may describe different part/reference | `DATUM AMBIGUOUS` |
| Total mass (kart + driver) | **~130 kg** previously reported | Battery, driver, payload, weight distribution might change | `REPORTED; WEIGH` |
| Centre of gravity height and position | **UNKNOWN** | Never infer from chassis geometry | `MEASURE` |
| Axle/tire contact patches, front/rear overhang | **UNKNOWN** | Critical for obstacle clearance | `MEASURE` |
| Steering angle range of actual road wheels | **UNKNOWN** | Historical software ±20° applied to a different configuration | `MEASURE` |
| Front/rear tire cornering and friction coefficients | **UNKNOWN** | No skidpad/surface tests | `MEASURE` |

**Critical:** `rear axle length 1026 mm` is not wheelbase. The **1117.6 mm** wheelbase is a newer **operator report**, not an independently audited physical measurement. Do not use front track **1066.8 mm** as a verified collision width until the 965.2 mm conflict is resolved.

## 3. BLDC drivetrain and AS5047P encoder calibration

### 3.1 Latest documented physical mount

Latest October 22 validation plan states the magnet is **centred on the 14-tooth BLDC sprocket**, facing a stationary perpendicular AS5047P PCB. Rear axle carries a **44-tooth** sprocket. Chain length reported **72.5 cm** (definition, link count, pitch and tension not yet verified). Therefore, **if** the magnet truly rotates with the 14T sprocket and there is no additional reduction/slip:

```text
N_motor = 14 teeth
N_axle  = 44 teeth
R = motor/encoder revolutions per axle revolution = 44/14 = 3.1428571429
Q = AS5047P absolute SPI angle positions per encoder revolution = 16384
```

This **replaces the direct-on-rear-axle assumption for the reported mounting**, pending a measured full-revolution test. AS5047P has single-turn absolute angle; derive cumulative motor rotation only by correctly unwrapping successive valid samples, tracking direction and reset history.

### 3.2 Tyre conversions: two conflicting nominal diameters

| Quantity | Later tyre-nameplate nominal `D=0.2794 m` | Older manually reported `D=0.295 m` |
|---|---:|---:|
| Nominal rear circumference `πD` | **0.87776099 m** | **0.92676983 m** |
| Ideal travel per **motor**/14T revolution `πD/R` | **0.27928759 m** | **0.29488131 m** |
| Ideal SPI unwrapped counts per axle revolution `16384×R` | **51492.5714** | **51492.5714** |
| Ideal SPI unwrapped counts per metre `16384×R/(πD)` | **58663.55** | **55561.34** |
| Valid ground-speed accuracy | **NOT VERIFIED** | **NOT VERIFIED** |

**Adopt neither nominal circumference for validated odometry.** Measure **loaded effective rolling circumference** `C_eff` with operating tyre pressure/load and repeat three runs. The 14T/44T ratio itself must be independently cross-checked by hand rotation.

### 3.3 Theoretical speed/RPM checks (use only for bench sanity)

For `C_eff ≈ π × 0.2794 m` and `R=44/14`:

| Vehicle speed | Rear axle RPM | 14T/motor/AS5047P RPM |
|---:|---:|---:|
| 2 km/h | 37.98 | 119.35 |
| 5 km/h | 94.94 | 298.38 |
| 10 km/h | 189.88 | 596.76 |
| 15 km/h | 284.82 | 895.13 |
| 20 km/h | 379.75 | 1193.51 |

Actual gear ratio, motor output, tachometer speed, chain dynamics and tyre slip can differ. **The simulation goal of 15–20 km/h is not a physically authorized speed.**

### 3.4 Encoder math (encoder on 14T sprocket)

```python
Q = 16384
R = 44.0 / 14.0    # provisional: verify actual sprockets and mount
raw_delta = ((raw_now - raw_prev + Q//2) % Q) - Q//2
# Reject invalid/parity/magnet frames, ambiguous >0.5-turn steps or long gaps.
encoder_total_counts += direction_sign * raw_delta
axle_revolutions = encoder_total_counts / (Q * R)
distance_signed_m = axle_revolutions * C_eff_m  # C_eff MUST be physically measured
axle_rpm = (delta_encoder_counts / (Q * R)) / delta_time_s * 60.0
wheel_speed_mps = (delta_encoder_counts / (Q * R)) / delta_time_s * C_eff_m
```

AS5047P SPI must validate parity, pipelined reads, magnet magnitude/diagnostics, sensor freshness and unwrap continuity; a 14-bit angular **resolution is not physical distance accuracy**. No cumulative odometry after a reset unless session continuity is explicitly tracked. Driving-wheel angular speed is **not** a reliable ground-speed measure under slip; Pi wheel–IMU fusion must model uncertainty.

### 3.5 Physical calibration test

1. Confirm exact 14T magnet/sensor mounting and rigid relationship to the rotating motor sprocket; check power **OFF** first.
2. Rotate 14T one revolution manually: require approximately one unwrapped encoder turn / **16,384** ideal increments and healthy magnet diagnostics.
3. Rotate the 44T axle sprocket exactly one revolution: require ~**3.142857** 14T turns, in the correct direction (check chain coupling).
4. Mark a rear tyre at contact patch; under normal load/pressure roll straight for ≥**five rear-tyre revolutions**, repeat **three times**, and calculate `C_eff = measured_distance / wheel_revolutions`.
5. Check tachometer RPM against encoder RPM and measured **10 m** ground distance against integrated encoder distance. Prior engineering *targets* were within **2%** for each; not yet approved safety requirements.
6. Record signs, chain slack, backlash, wrap/dropout behavior and correct units before installing as a VCU calibration.

**Latest test evidence:** AS5047P **SPI angle/magnet diagnostics passed in isolation**; PWM variant did not. Complete mounted ratio/loaded distance tests remain **PARTIAL/UNVERIFIED**.

## 4. Steering geometry, gear ratio and calibration

### 4.1 Steering gearing chronology / correction

| Record | Motor driving gear : steering shaft driven gear | Ideal shaft reduction | Status |
|---|---|---:|---|
| Older RC/steering configs | **17T : 51T** | 3:1 | `HISTORICAL, NOT CURRENT` |
| Earlier Oct 16 measurement discussion | **17T : 34T** | 2:1 | `SUPERSEDED BY LATER CORRECTION` |
| Later explicit Oct 16 correction | **34T : 34T** | **1:1** | `LATEST REPORTED; PHYSICAL VERIFICATION REQUIRED` |

A direct meshed 34T:34T pair reverses shaft rotation direction. **Do not** equate motor/steering-shaft angle with front-wheel steer: the linkage/Ackermann geometry must be calibrated. If R86mini microstepping remains **3,200 pulses per motor revolution**, 1:1 ideal shaft conversion is **8.8889 pulses per shaft degree**, and 90° shaft ≈800 pulses. These are **not** validated road-wheel commands or torque proof.

### 4.2 Actual road-wheel angle from AS5600

- Steering AS5600 12-bit raw absolute angle: `0..4095`, nominal quantisation `360/4096 = 0.087890625°`; actual error/noise and magnet alignment still to test.
- Wheel-command convention is not yet fully frozen: a historical VCU note proposed `+right`, while standard ROS `base_link` and Pure Pursuit formulas use **positive steering to left**. Choose **one explicitly defined sign conversion** at the VCU boundary and test physical directions; never silently flip signs.
- Record `AS5600 raw/unwrapped`, `motor step count`, steering-shaft angle, actual **left road-wheel** angle and **right road-wheel** angle at multiple measured setpoints, including both sweep directions.
- Suggested unloaded characterization points: `[-45,-30,-15,0,+15,+30,+45]` degrees **of steering shaft**, stopping early before real mechanical limits. These numbers do **not** authorize ±45° road-wheel steering.
- Calibrate a *monotonic lookup* from sensor reading to road-wheel/virtual bicycle steer (or geometric Ackermann mapping). For Ackermann, left and right road-wheel angles are generally different; derive the curvature-consistent equivalent angle from the measured linkage geometry.
- Measure zero offset, left/right sign, backlash, centre return, deadband, max achievable angle and max rate under load. Monitor missed steps and sensor faults; issued pulses are not proof of achieved steering position.

**Current evidence (22 October validation note):** unloaded steering UI commands/centre calibration worked, but steering **stuttered/slipped under load**. This test is marked **FAIL**, not calibrated for live use. Proposed initial targets of **≤1° repeatability** and **≤2° steady-state wheel-angle error** remain engineering targets, not passing results.

## 5. Brake actuator, effect and measurement

- NEMA23 **JK57HS112-3004-03** with **TB6600** is the historical mechanical brake actuator.
- Brake AS5600 #2, if installed at the actual braking linkage, supplies 12-bit I²C angle (0x36); latest validation plan describes **PCA9548A channel 1** for bus separation. Verify real mechanical installation and channel arrangement.
- Calibrate physically measured `released_angle`, at least two intermediate angles, `applied_angle`, linkage/cable travel, wheel resistance or brake force. Use angle-to-travel lookup if nonlinear; **percent travel ≠ brake force ≠ deceleration**.
- Repeat apply/release cycles **≥20**, measure response latency, binding, return-to-release, current, heat and power-loss behavior. A stepper cannot be assumed spring-applied/fail-safe on power removal.
- Stop test must physically measure entry speed, STOP-trigger latency, brake-onset latency, stopping distance/time and minimum repeated deceleration for each tested surface/temperature/load.
- **Latest evidence:** NEMA23/driver bench rotation worked. Mounted brake travel/force/power-loss behavior/vehicle deceleration are **BLOCKED/UNVERIFIED**. No live autonomous braking release.

### 5.1 Stopping-envelope model (conditional, not a validated stop distance)

```text
d_required_m = v_mps * T_total_latency_s + v_mps**2 / (2*a_brake_min_mps2) + clearance_margin_m
```

`T_total_latency` must include acquisition/frame age, inference/fusion/planner timing, Pi→VCU transport, local command acceptance, traction decay, brake response and uncertainty. `a_brake_min` must be the **conservatively measured worst suitable braking performance**, not theoretical tyre friction. `clearance_margin` must include front bumper distance, perception uncertainty, actuation variation, body swept footprint and surveying error. **All three safety-critical values are currently unknown.**

For illustration only, **350 ms LoRa heartbeat timeout** contributes **0.194 m** of unbraked travel at 2 km/h, **0.972 m** at 10 km/h and **1.944 m** at 20 km/h, **before** brake response and full stopping distance. A historical **15 cm** red-zone visual threshold is not a valid complete stopping clearance.

## 6. BLDC speed calibration, DAC, direction and safe states

- Drivetrain: **BLD-750** + BLDC motor; **GP8630N** I²C DAC historically `0x58`, targeting BLD speed-voltage (`SV`) control. Verify actual module, voltage span/jumper, wiring and power-off level.
- PN2222 transistors historically translate ESP32 GPIO control to BLD `EN`, `BRK`, `F/R` terminal contacts; **multiple firmware revisions use different GPIOs**. The October 22 validation plan tests GPIO **13/14/15** as one bench mapping; an older workbook maps **1/2/39**. **Neither is a guaranteed active pinout.** Measure physical terminal open/short-to-COM truth table for every boot/stop/direction state.
- **Latest evidence: GP8630N requested 5.0 V produced ~3.3 V measured**, and CW/CCW speeds differed strongly. The DAC/BLDC response test is recorded **FAIL**. Do not paper over this electrical fault with a software scaling coefficient or treat a DAC command as a measured output.
- Only after traction-isolated truth-table validation: measure requested/meter-read SV voltage vs reference motor RPM in both directions, 0 to rated approved input, current/heat, acceleration ramp and coast-down, command dropouts and driver alarms.
- Require safe outputs during reset, brownout, USB loss, watchdog trip, stale command and manual emergency stop. Neutral throttle/`target_speed=0` is not sufficient evidence of effective braking.

### 6.1 Longitudinal model (simulation until measured)

The physics backend may represent a simple longitudinal plant:

```text
m * dv/dt = F_traction(v, DAC_voltage, battery_voltage) - F_brake(brake_travel, v)
            - C_rr*m*g - 0.5*rho*C_d*A*v*abs(v) - m*g*sin(road_grade)
```

**UNKNOWN / need identification:** motor torque-speed map, effective drive ratio losses, traction limits, `C_rr`, `C_d*A`, longitudinal tire grip, `F_brake` vs travel/temperature and slopes. A first-order response `dv/dt = (v_target-v)/tau` is acceptable **only as an explicitly synthetic fallback** for four-stage simulation, never as an experimentally identified model.

## 7. Kinematic vehicle model for mapping, control and sim

### 7.1 Bicycle model (baseline)

Let `x,y` be rear-axle midpoint position in a consistent right-handed world frame, `ψ` yaw/heading CCW positive, `v` signed longitudinal speed, `L=wheelbase`, and `δ` equivalent **road-wheel** steering angle positive to the **left** in the conventional mathematical/ROS frame:

```text
x_dot = v*cos(psi)
y_dot = v*sin(psi)
psi_dot = v*tan(delta) / L
```

Initial **provisional** `L=1.1176 m` is an operator-reported value; do not use simulator constants (e.g. 1.05 m) as physical evidence. Use actual measured left/right Ackermann angles or curvature-consistent `δ` as model input. At low lateral acceleration this is a kinematic approximation; tire scrub, slip and yaw dynamics can cause large skidpad error. Integrate using measured timestamps (`dt`), angle wrap handling and correctly rotated frames; replay with deterministic fixed-step option.

### 7.2 Pure Pursuit geometry

For a lookahead point in `base_link` with **X forward/Y left**, selected lookahead distance `L_d` and bearing `α=atan2(y_lh,x_lh)`:

```text
L_d = clamp(L0 + k_v*abs(v), Ld_min, Ld_max)
kappa_cmd = 2*sin(alpha) / L_d
delta_cmd = atan(L * kappa_cmd)
```

`L0,k_v,Ld_min,Ld_max` are **NOT PHYSICALLY TUNED**. Clamp steering to physically tested road-wheel bounds **and rate limits**. The controller supplies a **target steering angle** to VCU; it does not send stepper pulses. If the path endpoint or obstacle stop is near, longitudinal planner must reduce speed and explicitly request brake/stop, not merely set target speed to 0 and call it stopped.

### 7.3 Lateral-speed feasibility

```text
kappa = curvature_of_smoothed_path
v_curve = sqrt(a_lat_verified / max(abs(kappa), epsilon))
v_allow = min(v_mission, v_curve, v_visibility, v_stopping_envelope)
```

**Do not populate `a_lat_verified` with a guessed friction coefficient.** If track geometry, body envelope, braking or sensor-range validation is unavailable, qualified live speed cannot be computed; the simulator may use named **SYNTHETIC** limits for algorithm exercises and display them as such. Swept-volume clearance must use tyres/body/front bumper, not kart centreline only.

## 8. IMU + wheel odometry and frame calibration

- Pi-hosted **MPU-6500** supplies timestamped angular velocity and acceleration; gyro bias, axis orientation, gravity compensation, acceleration scale and sensor mount are not yet calibrated.
- ESP32 AS5047P provides 14T motor-sprocket rotation; convert to axle travel using verified ratio and loaded circumference; AS5600 provides road-wheel/shaft angle only after linkage mapping.
- Pi fuses wheel-distance increments with IMU yaw-rate (`odom` → `base_link`), with timestamp/clock-offset compensation, covariance and zero-motion handling. Without GPS/landmark correction, **dead-reckoned odometry drifts**, particularly during skidpad slip.
- Simulation must publish frame-consistent `base_link`, `odom`, `map`, `camera_optical`; assign a single TF owner. Separate `ground_truth` from noisy observations and from fused odometry.

## 9. Nominal integration rates are design targets

| Signal / task | Latest VCU master proposal | Older RC-06 proposal | Live qualification |
|---|---:|---:|---|
| AS5047P SPI sampling | **200 Hz** | 200 Hz | Not benchmarked as installed |
| Steering AS5600 I²C | **100 Hz** | 200 Hz | Compare code to actual scheduler |
| Brake AS5600 I²C | **100 Hz** | Up to 100 Hz | Sensor/install verify |
| Steering controller | **50 Hz** | 200 Hz | Determine real measured control rate |
| BLDC PI | **50 Hz** | 100 Hz | DAC response fault unresolved |
| Pi → VCU commands | **50 Hz** | 50 Hz | Protocol/timing validation required |
| VCU → Pi fast telemetry | **50 Hz** | 100 Hz | Revision conflict: freeze one actual contract |
| Diagnostics | **10 Hz** | 10 Hz | Proposed |
| MPU-6500 | **~200 Hz proposed** | 200 Hz | Calibrate and measure |

Older UART/browser/LoRa timeouts include 1200/350 ms; the newer local Pi command **100 ms expiry** is a **design target**, not an as-built safety certification. Do not silently assume any of these rates for real evidence plots; each telemetry record needs a measured timestamp, measured period, sequence, validity flag and source label.

## 10. The simulation must distinguish four data levels

| Provenance | Allowed usage | UI label |
|---|---|---|
| `SYNTHETIC` | Ideal/perturbed simulated dynamics and simulated camera | SIMULATED — not real calibration |
| `REPLAY` | Recorded CSV camera/encoder/track logs; deterministic comparison | RECORDED REPLAY — no actuation |
| `LIVE_PHYSICAL` | Actual camera/IMU/encoders with measured quality status | LIVE — show validity, faults, calibrated/unverified |
| `MEASURED_PASS` | Independently verified parameter with run IDs, instrument and reviewer | VERIFIED PARAMETER — scoped evidence only |

Existing user simulator data may be under `~/Desktop/sim/`: `my_track_layout.json`, `MY_TRACK_SIM.py`, `MY_TRACK_built_1Hz.csv`, `MY_TRACK_built_10Hz.csv`, `MY_TRACK_shared_1Hz.csv`, `MY_TRACK_shared_10Hz.csv`, `MY_TRACK_SCENARIO_MATRIX.csv`. **These were named by the operator but are not provided inside this report or inspected in the current runtime.** They must be parsed/validated by the local integration agent before mapping track units, stage definitions, column names or reference frames. User STEP model reportedly `~/Desktop/go kart v39.step`; STL assemblies reportedly `~/Desktop/kart stl/{Body,Chassis,Engine,Rear Axle,Steering}`. Do not infer physical measurements from filenames.

### Sim hardware comparison requirements

1. Render 3-D kart using assembled STEP/STL model with correct local part transforms, pivots and collision footprint; physical model dimensions take priority after validated CAD import.
2. Maintain **separate** ground truth, simulated sensor perception, estimated odometry, path, control target and actuator response traces.
3. Use `L`, track, tire radius, ratio and steering mapping via versioned config, never hard-coded inside a renderer; unresolved values carry provisional labels.
4. For all four simulator UI stages, do not substitute perfect tracks/ideal observations where the selected stage purports to test the **real** YOLO/depth/controller ROS nodes. Test each existing ROS node via adapters without inventing replacement algorithms.
5. Scenario tests: direction, yaw frame/sign, straight corridor, circle of radius `R_turn=L/tan(delta)` for nonzero δ, 2-right/2-left skidpad winding + gate, dropout, cone ambiguity, limited braking, steering stutter, failure handling.
6. Log vehicle state/control at requested rates with actual `dt` and subsystem provenance; plot cross-track error, heading error, speed error, steering error, yaw-rate residual, stopping-distance margin and frame age.
7. Never route simulation/replay target commands to a physically armed VCU. Separate namespaces, actuator permission, dry-run gate and transport endpoints.

## 11. Editable provisional dynamics configuration (NOT physically approved)

```yaml
schema_version: 1
model_name: AutoKart_provisional_2026_10_23
source: REPORTED_AND_DERIVED
calibration_status: NOT_VERIFIED
geometry:
  wheelbase_m: 1.1176                 # operator-reported, needs remeasure
  front_track_m: null               # conflict 1.0668 vs 0.9652
  rear_track_m: 1.1811              # operator-reported
  chassis_length_m: 1.4224
  chassis_width_m: 0.6604
  ground_clearance_m: 0.0381
  body_front_overhang_m: null
  body_rear_overhang_m: null
  cog_height_m: null
  mass_kg: 130.0                    # prior operator estimate, weigh final vehicle
rear_tyre:
  nominal_diameter_m: 0.2794        # latest nominal size, not rolling calibration
  alternative_old_diameter_m: 0.295
  rolling_circumference_m: null
  pressure_kpa: null
encoder:
  model: AS5047P
  interface: SPI
  resolution_positions_per_turn: 16384
  physical_mount: BLDC_14T_SPROCKET_REPORTED
  drive_teeth: 14
  axle_teeth: 44
  encoder_revs_per_axle_rev: null   # verify 44/14 by physical rotation
  positive_forward_sign: null
steering:
  gear_motor_teeth: 34               # latest correction; inspect installed pair
  gear_shaft_teeth: 34
  shaft_gear_ratio_measured: null
  driver_steps_per_motor_rev: null # historical 3200; verify DIP
  left_wheel_angle_lookup: null
  right_wheel_angle_lookup: null
  virtual_steer_lookup: null
  max_road_wheel_steer_rad: null
  max_road_wheel_steer_rate_rad_s: null
  feedback_zero_raw: null
brake:
  released_as5600_raw: null
  applied_as5600_raw: null
  travel_to_force_curve: null
  verified_min_deceleration_mps2: null
  response_latency_p95_s: null
  physical_fail_safe_verified: false
bldc:
  requested_to_measured_voltage_curve: null
  voltage_to_rpm_forward_curve: null
  voltage_to_rpm_reverse_curve: null
  en_brk_fr_terminal_truth_table_verified: false
control:
  pure_pursuit_L0_m: null
  pure_pursuit_k_v_s: null
  lookahead_min_m: null
  lookahead_max_m: null
  lateral_accel_max_verified_mps2: null
  target_speed_limit_verified_mps: null
  longitudinal_pi_kp: null
  longitudinal_pi_ki: null
  total_stop_latency_p95_s: null
  stop_margin_m: null
validation:
  measured_test_run_ids: []
  reviewer: null
  release_authorized: false
```

**Implementation rule:** real/actuation paths must reject `null` safety-critical values. A `SIM_SYNTHETIC.yaml` override can supply **explicitly marked** nonphysical parameter choices for offline comparisons without modifying physical config or claiming a real calibration.

## 12. Validation test matrix and latest known status (22 October)

| Test / gate | Last recorded status | Evidence required to advance |
|---|---|---|
| Rear AS5047P SPI read and magnet diagnostics | **PARTIAL** | Full mount ratio, load/distance and speed verification |
| Steering center, backlash and road-wheel mapping | **FAIL** | Loaded stutter/slip eliminated; actual wheel-angle sweep and repeatability |
| Brake travel, physical stopping and power-loss behavior | **BLOCKED** | Installed sensor/force feedback, repeated stop distances and fail-safe proof |
| BLD-750 PN2222 polarity + DAC response | **FAIL** | Resolve 5V requested→3.3V measured, direction mismatch; real terminal truth tables |
| Independent hardwired traction cut/STOP | **BLOCKED** | Separate physically tested inhibit/safe braking chain |
| Pi–VCU hardware timing/ROS control path | **NOT VALIDATED AS A COMPLETE VEHICLE** | Test production build, fault injection, sensor freshness, loop jitter and commands |
| Camera 9-cone calibration | **GRID READY FOR FIT ONLY** | Independent held-out metric depth validation; see `CAMERA_CALIBRATION.md` |
| Autonomous skidpad and obstacle-braking release | **NO-GO** | All vehicle, perception, safety and measured stopping gates |

## 13. Test data row schema (for actual dynamics identification)

A valid real vehicle record must have at least:

```csv
run_id,utc_time,source,boot_id,seq,monotonic_time_us,encoder_raw,encoder_count_unwrapped,encoder_spi_valid,steering_as5600_raw,steering_valid,steer_left_wheel_deg,steer_right_wheel_deg,brake_as5600_raw,brake_valid,motor_voltage_requested_v,motor_voltage_measured_v,axle_rpm,reference_rpm,wheel_speed_mps,external_speed_mps,gyro_z_rad_s,accel_x_mps2,command_target_speed_mps,command_steer_rad,brake_request,stop_event,ground_distance_m,vehicle_config_hash,calibration_hash
```

For brake runs additionally log stop trigger/stamp, actuator motion onset, road wheel retardation, brake onset, total stop distance and observer. For steering runs log command step counts, actual shaft angle, left/right wheel angles and both sweep directions. Reject runs with missing clocks, missing provenance, sensors falsely marked valid, slipped tire assumptions or active safety defects. The simulator can replay incomplete rows, but the UI must label derived values and absent fields explicitly.

## 14. Sources and hierarchy

- `Pasted text(20261016-090611).txt`: vehicle measurement review (wheelbase 1117.6 mm, track conflicts, updated tyre markings) and later **34T:34T 1:1 steering gear correction**.
- `AUTOKART_VCU_MASTER_ENGINEERING_SPEC.md`, dated 22 October 2026: user-confirmed SPI/I²C hardware interfaces and modern VCU control proposal.
- `AUTOKART_EIGHT_REMAINING_VALIDATION_TESTS.md`, dated 22 October 2026: reported **14T sprocket-mounted AS5047P**, 44T axle ratio, validation results (steering FAIL, DAC/BLDC FAIL, braking BLOCKED).
- `AutoKart_Project_Detail_Complete.md` and `AUTOKART_12_NODE_MASTER_ARCHITECTURE.md`: historic software ownership and algorithm constraints.
- `CAMERA_CALIBRATION.md`: independent calibration and live sensor qualification.

**Final engineering statement:** This document is enough to parameterise a visibly **provisional** simulator and plan physical measurement/validation. It is **not** enough to authorize live vehicle actuation, claim validated steering or braking, or replace missing hardwired stop hardware.
