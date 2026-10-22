# AutoKart

ROS 2 stack + ESP32-S3 VCU firmware + Mission Control dashboard for the straight-corridor mission:
**drive between the cones at a throttle of at most 2 V and stop 15 cm before the cone ahead.**

**Status: nothing here has run on the real kart.** The pure logic (firmware state machine, protocol, bridge gate,
safety supervisor, mapping, planner, controller, gateway) is unit-tested on a PC (`pi/tests/run_all.sh`).
The ROS 2 node wrappers, the ONNX inference, the serial link and the browser UI have not been run against ROS or hardware.
Vehicle operation stays NO-GO until the hardware safety and stopping tests pass. See `../plan.md` for the full
integration, wiring and acceptance plan this layout implements.

## Layout
```
pi/                           everything that runs on the laptop/Pi (ROS 2, dashboard, tools, scripts, tests)
  ros2_ws/src/                 autokart_msgs  autokart_perception  autokart_mapping  autokart_planner
                                autokart_controller  autokart_vehicle_bridge  autokart_safety  autokart_bringup
  dashboard/                   frontend/index.html (Mission Control)   ros_gateway/ (ROS -> WebSocket)
  tools/                       vcu_console.py  braking_test.py  throttle_ramp_test.py  sim_gateway.py (SIMULATED UI data)
  scripts/                     start_autokart.sh  stop_autokart.sh  system_check.sh
  tests/                       pure-logic unit tests (pytest + the VCU firmware's own host tests)
esp32_firmware/                PlatformIO project (BLDC only): src/ include/ test/ platformio.ini
docs/                          PROTOCOL.md (shared laptop <-> VCU serial protocol)
```

## Scope decisions you made
- **Speed limit:** throttle DAC has an absolute design ceiling of **2 V** (`THROTTLE_V_MAX` / `THROTTLE_MAX_MV` in the
  controller and bridge code, `THROTTLE_MAX_MV` in the firmware), enforced three times: controller (clamp), bridge
  command gate, VCU firmware (a command above the cap while armed latches a FAULT, never silently clamped). The motor
  does not move below roughly its measured start voltage, so the commanded **run voltage is 0 V or in
  `[throttle_run_v, throttle_v_max]`**, never in between (`THROTTLE_RUN_MV` in firmware, `throttle_run_v` in
  `autokart.yaml`). Both are currently 2 V, so speed control is effectively on/off — see `SpeedController` in
  `autokart_controller/core.py`.
- **VCU scope:** the firmware parses every field (`throttle, steer, brake`) but **actuates only the BLD-750**
  (enable, brake line, forward, throttle DAC). Steering and the stepper brake are not driven. Pure Pursuit is computed and shown
  on the dashboard as "OBSERVE ONLY".
- **Stop rule:** stop with the front of the kart **15 cm before the cone ahead** (`stop_gap_m`). The "center cone" is any cone within
  `stop_lateral_tol_m` of the centerline (or set `stop_class`). The planner only ever triggers an early stop (a
  measured-too-optimistic deceleration eats into the margin, not the 15 cm itself) — see `d_margin_m` below.
- **Stop sequence:** on any stop (fault, disarm, brake command) the firmware writes the DAC to 0 V *before*
  de-asserting EN / asserting BRK (see `loop()` in `main.cpp`), matching the shutdown order the BLD-750 manual expects.
  Final stop uses the BLD-750 BRK line (`brake_pct >= 50`). Speed control runs on the laptop (feed-forward + PI -> volts).
- **Overspeed:** if measured wheel speed ever exceeds `overspeed_mps` while armed, `autokart_safety` faults and
  disarms (stop) regardless of what the controller was asking for — see `autokart_safety/core.py`.

## ⚠️ EN/BRK/F-R/E-stop polarity is UNVERIFIED — do not power the motor before the bench checks
`esp32_firmware/include/config.h` keeps its **original, unconfirmed** `GPIO_EN_ENABLE_LEVEL` / `GPIO_BRK_ASSERT_LEVEL`
/ `GPIO_FR_FORWARD_LEVEL` / `ESTOP_OK_LEVEL` values — none of them have been inferred or changed from a guess about
bench behaviour, and **none of them have been measured through the actual PN2222 transistor interface**. The bench
fact that EN and BRK ran the motor when hand-jumpered directly to COM says something about the BLD-750's inputs, but
nothing about which GPIO level this firmware's PN2222 stage needs to reproduce that state — that depends on the
PN2222 wiring (NPN/PNP, pull-up/pull-down), which has not been measured. **Do not assume either polarity.** Follow
`docs/BENCH_TESTS.md` step (c) — wheels off the ground, motor free to spin by hand, multimeter on the GPIO and on
the BLD-750 pin — and confirm every one of these levels before trusting them, let alone driving the kart. Only after
that checklist is signed off should these macros be treated as known-good.

## Before the first powered test (in this order)
See `docs/BENCH_TESTS.md` for the full step-by-step checklist. Summary:
1. **DAC driver:** `esp32_firmware/src/dac_port.cpp` implements the GP8630N over I2C (ported from the MicroPython
   bench test). It reports `dac_ok = false` — and the VCU refuses to ARM, or faults `DAC_FAIL` if already armed —
   on any failed I2C step, since the GP8630N has no read-back to verify against.
2. **Polarity:** review every `VERIFY` in `esp32_firmware/include/config.h` (EN/BRK/F-R levels, E-stop status level, `AXLE_CPR`,
   `TYRE_RADIUS_M`). Reset state (GPIO low via pull-downs) must mean "drive inhibited" — see the polarity warning above.
3. **Flash** (`pio run -t upload`), then `python3 pi/tools/vcu_console.py /dev/ttyACM0` with the **wheels off the ground**:
   `arm` must be refused without the DAC; with it, check `thr 0.3` (never above the 2 V `THROTTLE_MAX_MV` cap in
   `config.h`), `brake 100`, pull the USB cable (must fault within 300 ms), press the physical E-stop (must fault
   and cut drive).
4. **Throttle ramp test:** `python3 pi/tools/throttle_ramp_test.py /dev/ttyACM0` — steps 0 to 2 V in 0.1 V holds,
   watching for the driver alarm LED the bench run saw near 0.75 V. Investigate, don't just dodge, any alarm.
5. **Measure and fill** the `MEASURE` values in `autokart.yaml`: wheelbase, bumper offset, camera mount + intrinsics, steering zero/scale.
6. **Braking test:** `python3 pi/tools/braking_test.py /dev/ttyACM0 --volts 1.0 --runs 3` on an open area, then copy
   `a_verified_mps2`, `t_latency_s`, `volts_per_mps` into the config.
7. Set `vehicle_config_verified`, `braking_verified` (safety) and `calibration_verified` (perception) to `true` **only after** the above.
   Until then the dashboard lists the blocking items and ARM stays disabled. **These flags are the operator's call —
   nothing in this repo sets them automatically.**

## Run
```
pi/scripts/system_check.sh
pi/scripts/start_autokart.sh --build          # builds pi/ros2_ws with colcon, starts ROS + gateway
# open http://localhost:8080/index.html
```
The HP60C SDK owner must already be publishing `/hp60c/color` (rgb8) and `/hp60c/depth` (16UC1, aligned to colour).
Nothing in this repo opens the camera. Without ROS, `python3 pi/tools/sim_gateway.py` serves SIMULATED data to the same dashboard.

## Tests
```
pi/tests/run_all.sh     # 75 VCU firmware-logic checks (host-compiled C++) + 50 pytest cases
```

## How the 15 cm is (and is not) guaranteed
The kart starts braking when `available <= v^2/(2a) + v*t_latency + margin`. With the shipped placeholders
(`a = 0.5 m/s^2`, margin 10 cm) it stops **early** on purpose (a logic simulation shows ~65 cm). Once `a` and the latency are measured,
lower `d_margin_m` toward 0.02 m. If the configured deceleration is better than the real one, the kart overshoots, so measure first.
The camera may lose the cone at close range; mapping keeps the cone and the planner continues from wheel odometry for the last stretch.

## Safety design in one paragraph
Dashboard buttons only send requests. `autokart_safety` is the only node that can arm: it needs every startup stage READY
(real data freshness / flags, never "process exists"), verified config, E-stop confirmed by the operator, and a SAFE VCU.
While armed it faults on stale camera/detections/map/plan/odometry/controller, VCU heartbeat loss, E-stop, overspeed
(`overspeed_mps`, tune after measuring speed at 2 V) and a stop that does not slow the kart. A fault disarms the VCU, which
inhibits drive and asserts the brake line. The VCU does the same by itself after a 300 ms command timeout.
None of this replaces the independent hardware E-stop.

## Known gaps
- No IMU in this build: yaw is a bicycle-model estimate from the measured steering angle (status `ENCODER_STEER_MODEL`).
- Only forward driving; direction is set once at boot.
- AS5600 PWM decoding constants and the AS5047P counts-per-rev need bench verification.
- YOLO26 output handling supports the end-to-end `[1,N,6]` head and the raw `[1,4+nc,N]` head; check `class_names` order against your model.
