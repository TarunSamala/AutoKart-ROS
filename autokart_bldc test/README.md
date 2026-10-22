# AutoKart BLDC experimental test harness

Isolated experimental application. Does **not** modify, replace, or get
imported by the production ROS 2 stack (`kart/ros2_ws/`), `vcu_firmware/`,
`autonomy_stack_phase1/`, or any of their configuration/safety files. It
*reuses* existing, already-verified-as-far-as-software-goes modules as
libraries:

- `autokart_perception` (installed package, v0.2.0) for the HP60C camera
  bridge, the pinned ConeNet v1 detector, depth validation, metric cone
  localization, Delaunay/Gabriel graph, boundary/gate/centerline/curvature
  geometry, and the perception safety assessment (`vehicle_authorized`).
- `autokart_vehicle_bridge.protocol` (from `kart/ros2_ws/src/autokart_vehicle_bridge`)
  for CRC16 validation and HELLO/telemetry decoding.

The target is the **Raspberry Pi Pico (RP2040)** on the USB by-id path in
`config/test_config.yaml`, running firmware that identifies as
`0.1.0-rp2040-pico-bench-unwired` with protocol revision 2. This is a
bench-only, unwired port. A Pico 2 (RP2350) or another Pico firmware requires
a separate identity and wiring review. See `docs/BENCH_TESTS.md` for the
physical prerequisites. There is no axle encoder, and
`calibration_verified` / `vehicle_config_verified` / `braking_verified` in
`kart/ros2_ws/src/autokart_vcu_safety/config/vcu_safety.yaml` were all
`false`. **Nothing in this app changes any of that.** The experiment opens
the Pico serial port for reading only; it never transmits ARM, CMD, OBS,
DIS, or RST. The production VCU bridge remains the sole command writer.

## What this app actually does

```
HP60C camera -> ConeNet v1 detector -> metric cone localization -> bird's-eye
graph -> boundary/gate/centerline/curvature -> corridor summary -> obstacle
check (cone blocking the path + required stopping clearance) -> steering
angle PREVIEW + BLDC throttle PREVIEW -> mission state machine -> passive
Pico HELLO/telemetry monitor + CSV log + browser dashboard
```

Throttle output is hard-disabled. The app exits if
`throttle.motor_output_enabled` is set true; there is no serial write path.
The Pico firmware also retains `VCU_OUTPUT_WIRING_VERIFIED=0` and rejects ARM.
Steering, DAC, brake, and STOP values shown here are software previews only.

## Running it

```bash
./start_test.sh --mode live            # camera + passive Pico serial monitor
./start_test.sh --mode camera-only     # camera/perception, no Pico port open
```

Dashboard: `http://127.0.0.1:8780/` (see `config/test_config.yaml: dashboard`).
Dashboard controls: CAMERA START/STOP, PERCEPTION START, NAVIGATION PREVIEW
ON/OFF, STOP REQUEST, RESET FAULT. STOP REQUEST records a software stop
decision; it does not send a physical stop command or confirm braking.

Software-only checks (no camera or VCU required to pass; camera/VCU checks
run too and are reported honestly if hardware happens to be present/absent):

```bash
python3 tests/test_software_only.py
```

Per-run CSV logs land in `results/run_<timestamp>.csv`.

## Folder structure

```
config/test_config.yaml      - all tunables; every "measured" stopping value is an unfilled placeholder
camera/hp60c_bridge.py       - thin wrapper around autokart_perception.camera.HP60C
perception/cone_detector.py  - wraps autokart_perception.live_mission.live_detector
perception/cone_geometry.py  - wraps autokart_perception.pipeline.Pipeline
planning/corridor_planner.py - formats the pipeline's existing centerline/boundaries
planning/obstacle_checker.py - cone-in-path check + BrakeCapability-based stopping clearance
control/steering_controller.py - curvature -> steering angle PREVIEW (never actuated)
control/throttle_preview.py  - DAC voltage PREVIEW with ramp/clamp (disabled by default)
control/stop_controller.py   - INITIALIZING..FAULT mission state machine
vcu/serial_interface.py      - passive Pico HELLO/telemetry monitor; no writes
dashboard/server.py          - stdlib http.server dashboard, no new web framework dependency
app.py                       - orchestrator
start_test.sh                - launcher
tests/test_software_only.py  - the checks behind TEST_REPORT.md
results/                     - CSV run logs (gitignored content, kept as evidence)
```

## Honesty rules this app follows

- A result is never reported as a passed physical test unless it came from
  actual hardware telemetry/measurement, per `TEST_REPORT.md`'s TESTED /
  NOT TESTED / PASS / FAIL columns.
- `STOPPED` in the state machine means a software stop decision, not physical
  stopping. No command is sent by this app.
- The obstacle checker reports `required_clearance_m: null` and
  `clearance_qualified: false` whenever `stopping.measured` is `false` in the
  config, rather than printing a stopping distance computed from an unmeasured
  deceleration.
