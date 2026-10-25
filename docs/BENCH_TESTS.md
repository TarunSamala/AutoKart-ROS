# VCU + mission commissioning checklist

**See also `docs/TEST_RUNBOOK.md`** for the one-page stage table (pass/abort criteria, who holds the
manual power cut, what to paste where) and `brake_test/run.sh` for the integrated,
guided, resumable perception-to-VCU braking runner
through all of it. `tools/readiness_report.py` is the single final gate: it prints `READY FOR FIRST RUN`
only once every measured gate (camera rate, depth fit, latency, braking, model/calibration status,
runway, `system_check.sh`, and the three verified flags) is PASS -- never before.

## Current hardware boundary (2026-10-19)

The ESP32-S3 is presently **USB-only with traction isolated**. The locked `0.2.1-no-encoder-locked`
firmware has been flashed; HELLO, SAFE telemetry, and OBS/OBSACK were observed over USB. The
GP8630N DAC and BLD-750 EN/BRK/F-R stage are disconnected for this session. **No E-stop status wire is installed at GPIO7.**
Software STOP depends on the laptop, USB link and ESP32 and is not an emergency stop. The physical battery/supply disconnect is the only emergency stop; it is not visible to firmware.
The updated `0.2.2` source is not flashed in this task. `ESTOP_INPUT_FITTED=0` is its default; telemetry explicitly marks the input NOT FITTED.
There is **no axle encoder hardware**: telemetry reports `counts=-1` and `speed_mmps=-1`.
Sections (g1), (h), and (i) below are historical encoder-based commissioning procedures and
are **NOT EXECUTABLE on this kart**. Do not set `counts_per_m`, infer speed from the 2 V DAC cap,
or use the CPR × tyre-radius fallback without an installed, verified encoder. A powered run
requires an independently measured speed/pose and braking method plus physical wiring and camera
qualification. The current 1.0 m commissioning gap is only a configured target; 0.15 m is the
requested final red zone, not a tested stopping result. `scripts/system_check.sh` must return
GO before any motion request.

## No-encoder commissioning aids

- `python3 tools/depth_check.py --fit-pitch --distances 1.2 2 3 4` collects raw
  HP60C depth/pixel samples and compares a 5–25° fitted pitch with the configured
  17.8° and old 11.4° estimate. It saves per-station residuals. A candidate
  `camera_pitch_down_deg:` line is printed only when four stations fit within the
  diagnostic error budgets; it is **not** a qualified calibration and is never
  written automatically. Tape from the camera ground projection to cone base.
- `python3 tools/runway_check.py --v-run SPEED --a-verified DECEL --t-latency
  SECONDS --margin METRES` prints runway for 1.0, 0.6, 0.4, 0.25 and 0.15 m
  bumper gaps. Inputs must come from independent physical measurements; 2 V
  DAC alone supplies no ground-speed value.
- `scripts/first_run.sh` requires a GO from `scripts/system_check.sh` on an
  already running graph, prints the operator checklist and active live gap,
  and waits for typed review. It sends no ARM or throttle request. On the
  current USB-only setup it correctly refuses to proceed.
- `python3 tools/stop_trigger_test.py` is a read-only wheels-off ROS observer.
  It requires preflight GO, then an independently armed run. It checks a
  qualified live stop-cone trigger, matching zero bridge TX, and VCU telemetry
  with zero DAC request, enable off and brake asserted within the timeout.
  Its PASS is **telemetry only**; the operator must separately measure SV 0 V
  and the BLD-750 BRK terminal state. It cannot run on the USB-only setup.

**Do this with the kart's drive wheels OFF THE GROUND and the physical power cut within reach**, except section
(h) which is explicitly the first powered ground run. Every other step is a bench check, not a driving test.
Stop and investigate at the first sign of anything other than the described behaviour.

This exercises `vcu_firmware/` (flashed as the arming-locked USB-only build), `kart/ros2_ws/src/autokart_vehicle_bridge`,
`kart/ros2_ws/src/autokart_localization`, and `kart/ros2_ws/src/autokart_vcu_safety`. The operator reported all
four installed GPIO/terminal polarity states; see `vcu_firmware/include/config.h`. This session has not witnessed
the measurements or tested the DAC and combined driver behavior. Fill in every remaining blank with direct evidence.

**Current source is arming-locked:** `VCU_OUTPUT_WIRING_VERIFIED=0` makes every ARM return NAK 7. The reported
polarity values match the current macros, but DAC SV, combined EN/BRK behavior, physical power cut action and
braking/speed must still be physically qualified before any unlocking decision. The JKBLD750 manual says EN shorted = run / open = slow
stop; BRK shorted = run / open = quick stop; F/R open = forward. Its manual recommends leaving the *other*
stop input in its factory state when using one stop method. The firmware currently controls both EN and BRK,
so verify that arrangement on this exact driver before treating its stop behaviour as qualified.

## Before you start
- Confirm the GP8630N module's allowed VCC is 3.3 V (check the module's silkscreen/datasheet; some breakout
  boards are 5 V-only on the logic side even though the output stage goes to 10 V). Result: ______
- Confirm wiring: ESP32 GPIO8 = SDA, GPIO9 = SCL to the GP8630N; GP8630N OUT -> BLD-750 SV; GP8630N GND -> BLD-750 COM.
- Have a multimeter on the SV line (GP8630N OUT to BLD-750 COM) for every step below.
- The locked firmware is already flashed. Record its HELLO version; do not upload another build as part of this checklist.

## (a) HELLO and dac_ok
1. `python3 tools/vcu_console.py /dev/ttyACM0`.
2. A `HELLO` line appears (firmware version, protocol version) within 5 s of reset. Result: ______
3. Telemetry reports `dac_ok=1` (bit4 of `flags`, see `docs/VCU_PROTOCOL.md`). If 0, stop: `dac_init()` failed —
   check the I2C wiring and the GP8630N address (0x58). Result: ______
4. `arm`: refused with `dac_ok=1` if some other precondition fails (wiring lock, link). Still refused if you
   unplug the GP8630N and retry (`dac_ok` drops to 0, ARM NAKs "DAC"). Result: ______

## (b) Unplug-USB fault and recovery
1. `arm`, `thr 0.3` (motor spinning freely, wheels off the ground).
2. Unplug the USB cable. Within 300 ms (`CMD_TIMEOUT_MS`) the VCU must fault (`LINK_TIMEOUT`) on its own —
   confirm SV returns to 0 V within that window with the multimeter. Measured time to 0 V: ______ ms
3. Reconnect, reopen `vcu_console.py`. VCU is in FAULT (not silently re-armed), needs `rst`. Result: ______

## (c) EN / BRK / F-R polarity — measure, do not infer
`config.h` ships its **original, unverified** polarity macros. A bench fact that EN and BRK ran the motor when
hand-jumpered directly to COM tells you about the BLD-750's inputs, nothing about which ESP32 GPIO level the
PN2222 stage needs to reproduce that state. **Do not guess; measure each one.**
1. Motor wheel free to spin by hand, wheels off the ground. `arm`, `thr 0.3`. Motor spins only while `en=1` in
   telemetry. Measured BLD-750 EN pin voltage (relative to COM) in this state: ______ V. Matches
   `GPIO_EN_ENABLE_LEVEL` through the PN2222? ______
2. `dis`. Motor free to spin again (EN released), `brk=1` in telemetry (default-asserted in SAFE, per
   `BRAKE_LINE_IN_SAFE`). Measured BLD-750 BRK pin voltage in this state: ______ V.
3. Try to spin the wheel by hand while disarmed. Resists (brake engaged) or spins freely? ______. If it spins
   freely while `brk=1`, the polarity is **backwards**: stop, fix `GPIO_BRK_ASSERT_LEVEL`, re-flash, redo this
   whole section.
4. `arm`, `thr 0.3`, then `brake 100`. `brk=1`, axle resists the same way as step 3, `dac_mv` drops to 0
   immediately (braking forces throttle to zero). Measured delay between brake command and SV at 0 V: ______ ms.
5. `GPIO_FR_FORWARD_LEVEL`: wheel free, `arm`, `thr 0.3`, confirm the turn direction matches "forward" for the
   kart. Observed: ______ (matches: Y/N).
6. GPIO7 is not connected. In the updated source, TEL bit5 and bit0 both stay clear and the dashboard says NOT FITTED. No GPIO7 voltage or E-stop actuation result may be claimed.
7. Treat this section PASSED only once every measurement matches what the macros claim. Any mismatch: fix that
   macro, re-flash, redo the whole section.
   **Section result (PASS / FAIL, date, initials): ______________________**

## (d) Physical power cut
The manual battery/supply disconnect must be reachable and independently remove traction power. Verify its physical operation under a separately approved safe bench procedure; the ESP32 cannot observe it and no `ESTOP` telemetry fault is expected. Result: ______

Optional monitored upgrade: fit a normally closed switch from GPIO7 to 3.3 V with an external pull-down. A wire break then reads LOW. Set `ESTOP_INPUT_FITTED=1`, verify HIGH healthy / LOW pressed, and repeat the fitted-input fault and 3 ms filter tests. This status circuit does not replace the independent power cut.

## (e) Measure SV with a multimeter at 0, 1, 2 V
Wheels off the ground, meter on SV:
1. `arm`, `thr 0.0`: SV ~0 V. Measured: ______ V
2. `thr 1.0`: SV close to 1.0 V. Measured: ______ V
3. `thr 2.0`: SV close to 2.0 V, never exceeding `THROTTLE_MAX_MV` (2000 mV, the absolute design ceiling,
   enforced in firmware, the bridge, and nowhere silently raised). Measured: ______ V
4. A throttle command above the cap from the console faults the firmware (`CMD_RANGE`) rather than exceeding
   2 V. Result: ______
5. `rst`, `dis`.
6. DAC accuracy (commanded vs measured, steps 1-3): ______________________

## (f) Throttle ramp test — find the alarm voltage, don't just avoid it
`python3 tools/throttle_ramp_test.py /dev/ttyACM0` — types `YES`, steps 0 -> 2.0 V in 0.1 V holds, asks about
the alarm LED at each step, returns to 0 V at the first `y`/Ctrl-C/fault.
1. Voltage where the motor first turned: ______ V
2. Voltage where the alarm LED first appeared, if any: ______ V
3. A bench fact says the BLD-750 faulted near 0.75 V previously. Reproduced here? ______. If yes, or any alarm
   below 2.0 V appears, investigate (do not just note it and move on):
   - Driver current-limit setting: ______________________
   - Acceleration / soft-start trimmer settings: ______________________
   - Supply voltage sag under load (measure supply at the alarm voltage): ______________________
   - Grounding of SV and COM (continuity, shared ground with ESP32 logic ground?): ______________________
   - Noise on SV (scope if available): ______________________
   - Conclusion / fix applied: ______________________
4. Only once the ramp reaches 2.0 V with no alarm (or the cause above is understood and fixed) is 2.0 V safe to run at.
   **Section result (PASS / FAIL, date, initials): ______________________**

## (g) Depth and mounted camera check - stationary
With the final camera mount, first resolve the reported 17.8 degree pitch against the saved ground-plane self-check estimate of 11.4 degrees in `calibration/kart_hp60c_2026-10/ground_plane_self_check_2026-10-19.json`. The operator-reported height and pitch are configured but remain unverified.

Run `python3 tools/camera_visibility.py` to print the nominal geometry from a saved live HP60C SDK frame (`fy=585.0195`, `cy=239.9636`, 480 rows). The computed nearest visible ground point is about 0.998 m camera-forward. At the 0.15 m target gap, the cone base is 1.2168 m camera-forward, 34.56 degrees below horizontal and 16.76 degrees below the optical axis, nominally in view. The configured accepted depth range is **0.3 to 4.0 m**; the stop-point slant range is 1.478 m. This is a geometric prediction, not proof of valid depth on the mounted kart.

Place one cone at tape-measured camera-forward distances 1.2, 2, 3, 4, and 5 m. With the vehicle stationary and the ROS camera owner stopped, run `python3 tools/depth_check.py --frames 30`. Select the cone body in each RGB image. The tool saves `results/depth_check_*.json`, reports median projected visible-surface distance, error in cm, and diagnostic pass/fail against 5 cm at 1.2 m or 5 percent beyond. **The 5 m station exceeds the current 4 m depth acceptance limit; it must be reported OUTSIDE_CONFIGURED_DEPTH_RANGE, not passed.** The comparison includes surface-to-base parallax and uses an unqualified RAW16 scale, so it does not itself set `calibration_verified`. Review the largest measured error, finish the surveyed mount/depth qualification, and include braking and latency uncertainty when choosing the stop gap.

## (g1) Rear-axle distance scale - hand push, traction isolated
Stop the ROS bridge so the calibration tool has exclusive serial ownership. Keep traction isolated and the VCU SAFE. Run `python3 tools/encoder_calibration.py --port /dev/ttyACM0 --trials 3`; enter each tape-measured forward push distance, capture before counts, push by hand, and capture after counts. Review each counts/m value and the mean. Paste the printed `counts_per_m:` and `encoder_forward_sign:` lines into `kart/ros2_ws/src/autokart_bringup/config/commissioning.yaml` under `autokart_localization.ros__parameters`. The tool writes neither config nor serial commands. Until measured, `counts_per_m: 0.0` leaves odometry on the firmware's `AXLE_CPR × TYRE_RADIUS_M` conversion, both marked VERIFY. Result: ______ counts/m; direction: ______; tape comparison: ______.

## (g2) End-to-end stop latency - stationary evidence
Record `/autokart/{perception,mapping,planning,controller}/status` and `/autokart/latency/trace` in a rosbag2 bag, or run `python3 tools/latency_report.py --live --seconds 30` while the ROS graph is active. For a recorded bag use `python3 tools/latency_report.py --bag BAG_DIRECTORY`. The tool requires a single host monotonic clock, one frame stamp shared across all four stages, a zero bridge TX tied to that frame, and VCU telemetry acknowledging that TX with `dac_mv=0`; incomplete chains produce no latency estimate. Review per-frame stage times, p50, p95 and max; paste its `t_latency:` line (p95 plus 50 ms, in seconds) into `commissioning.yaml` under `autokart_planner.ros__parameters`. No value is written automatically. With `t_latency: 0.0`, LIVE braking qualification remains unavailable. Result: ______ s; sample count: ______.

## (h) Braking test - controlled physical measurement
Only after sections (a)-(f) and the operator's physical checks pass, keep the ROS vehicle bridge stopped and run `python3 tools/braking_test.py /dev/ttyACM0`. Its operator prompt now names the physical battery/supply disconnect as the emergency stop. The tool still requires encoder telemetry, which this kart lacks, and therefore cannot measure braking here. No braking test is authorized by this document or this software change.

The tool saves `results/braking_*.json` with encoder-derived speed traces, speed at cut, stopping time/distance, and per-run mean deceleration. It prints `v_run` at 2 V and a `brake_min_mps2:` line equal to 80 percent of the smallest final-run mean deceleration. Review that line and paste it into the planner launch parameters only after accepting the physical evidence; the tool never writes verified flags. Use the separately measured `t_latency` from (g2) for perception-to-confirmed-DAC-zero timing. Do not infer it from a firmware build or host test.

## (i) First powered ground run — blocked pending physical qualification
Only after (a)-(h) all PASS, `calibration_verified`, `vehicle_config_verified` and `braking_verified` are all
set `true` by a person who reviewed the evidence above, and `autokart_vcu_safety`'s operator
`confirm_manual_power_cut` request has been sent for that ARM:
1. Lay out a short, straight lane with a single stop cone roughly 3-4 m ahead — well inside the sensor range vs
   stopping-distance margin validated in (g)/(h), and short enough that a miss is still a low-speed, low-energy
   event.
2. Physical power cut within reach, a second person watching the kart, nobody standing in front of or behind the lane.
3. Request `arm` from Mission Control (forwarded through `/autokart/safety/request` to `autokart_vcu_safety`,
   which decides) and confirm the E-stop stage is amber NOT FITTED, the manual power cut stage is READY, and all other required startup stages are READY before the request is accepted.
4. For the first short run, keep `stop_gap_from_bumper_m: 1.0` in `kart/ros2_ws/src/autokart_bringup/config/commissioning.yaml`, plus the separate braking margin. Record the actual front-bumper-to-cone gap: ______ m. Tighten toward 0.15 m only after measured depth and braking errors permit it.
5. Request `stop`. Confirm the VCU actually stops (not just the command) before touching anything. Each phase-1 launch starts a read-only CSV in `results/runs/` with time, cone forward distance, bumper gap estimate, speed, actual VCU `dac_mv`, decision, state, and axle counts. Run `python3 tools/run_report.py results/runs/RUN_FILE.csv` after stopping; it uses the last genuine cone sighting and calibrated encoder travel to print a final front-bumper gap estimate. Compare this estimate with a tape measurement. It refuses to estimate while `counts_per_m` is unset. Do not use a good estimate alone to qualify braking or camera geometry.
6. Repeat 2-3 times before changing ANY configuration. Do not raise speed, tighten the margin, or change the
   mission after a single run — repeatability is the evidence, one good run is not.
   **Section result (PASS / FAIL, date, initials, number of repeats): ______________________**

## Gate
`calibration_verified` (`autokart_perception`), `vehicle_config_verified` and `braking_verified`
(`autokart_vcu_safety`) all stay `false` until a person who ran this checklist sets them after reviewing the
actual measurements above. Nothing in this repository sets them automatically; `autokart_vcu_safety/core.py`
refuses to allow ARM while any of them is `false`. **Nothing in this repository, or in passing its tests,
authorizes vehicle motion.**
