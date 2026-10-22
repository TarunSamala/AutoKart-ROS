# AutoKart Pico BLDC experiment: verification report

Updated 2026-10-21 (Asia/Kolkata). Target: Raspberry Pi Pico **RP2040**.
The experiment is read-only on the VCU serial link and cannot move the kart.
Vehicle motion: **NO-GO**.

## Exact commands

```bash
cd "/home/mark1/Desktop/autonomous_kart/autokart_bldc test"
python3 -m unittest discover -s tests -p 'test_pico_monitor.py' -v
python3 tests/test_software_only.py
./start_test.sh --mode live --frames 5
./start_test.sh --mode camera-only
```

The unbounded live command is `./start_test.sh --mode live`; stop it with
Ctrl-C. Dashboard: `http://127.0.0.1:8780/` while the app runs. It only
previews STOP and actuator commands.

## Results from this update

| Check | Status | Evidence |
|---|---|---|
| RP2040 Pico USB identity | TESTED / PASS | `/dev/serial/by-id/usb-Arduino_RaspberryPi_Pico_A63064E62F6E204B-if00` resolved to `/dev/ttyACM0`. Passive probe received `HELLO,0.1.0-rp2040-pico-bench-unwired,2` and telemetry. |
| VCU command link | TESTED / FAIL | Pico telemetry reported `link_age_ms=148474` during the software check, above the 300 ms command-link limit. The harness intentionally sends no CMD heartbeat. |
| Pico monitor software safety | TESTED / PASS | Two tests passed: matching identity accepted, ESP32 identity rejected; the fake serial endpoint recorded zero writes in both cases. |
| HP60C camera frame | TESTED / PASS | Camera probe returned live 640×480 RGB and depth arrays, with intrinsics `fx=585.27`, `fy=585.02`, `cx=335.98`, `cy=239.96`. This does not validate metric depth accuracy. |
| Detector and geometry initialization | TESTED / PASS | `detector_load` and `geometry_engine_init` passed in `tests/test_software_only.py`. |
| Live cone detection and distance | NOT CONFIRMED in this update | The short run processed live frames but logged no per-cone boxes or tape-measured distances; detector load and camera frame availability alone do not establish detection accuracy. |
| Cone distance calibration | NOT TESTED | No tape-measure comparison or depth error study was run in this update. |
| Corridor/path from live cones | TESTED / FAIL | Five-frame live run returned `NO_PATH` with `STALE` or `AMBIGUOUS` safety status; no path qualified. Evidence: `results/run_20261021T090041Z.csv`. |
| Steering preview | TESTED / PASS, SYNTHETIC | A curvature sample of 0.2 m⁻¹ produced 0.226 rad. No live qualified corridor produced a steering request. |
| DAC preview | TESTED / PASS | Disabled output produced 0 mV in the software check and all five live CSV rows. No voltage was measured at the GP8630N. |
| Obstacle logic | TESTED / PASS, SYNTHETIC | Synthetic cone at 2 m marked blocked; stopping clearance remained unqualified because measured braking data is absent. |
| Software STOP decision | TESTED / PASS | All five live rows had `decision=0`, `target_speed_mps=0`, and `dac_mv=0`. This was a software preview, not a physical stop. |
| Pico STOP command and acknowledgement | NOT TESTED | The read-only harness sent no DIS/OBS/CMD. No Pico acknowledgement or braking action was established. |
| Dashboard read endpoints | TESTED / PASS | During a second live run, `/`, `/api/status`, and `/snapshot.jpg` each returned HTTP 200. Status showed expected Pico HELLO, `vcu_command_transmitted=false`, `motor_output_enabled=false`, `decision=0`, and `NO_PATH`. Dashboard button actions were not rechecked. |
| Physical steering, BLDC, brake, obstacle stop | NOT TESTED | No actuator, traction, or vehicle motion test was performed. |

## Evidence and limits

- `python3 tests/test_software_only.py`: software checks and live camera/Pico
  passive probes passed; VCU **command-link freshness was false**.
- `python3 -m unittest discover -s tests -p 'test_pico_monitor.py' -v`:
  2 tests passed.
- `./start_test.sh --mode live --frames 5`: completed, wrote
  `results/run_20261021T090041Z.csv`; serial port and dashboard port closed
  after exit.
- A second live run served the dashboard and wrote
  `results/run_20261021T090147Z.csv`; it was stopped with Ctrl-C. The Pico
  serial port and dashboard port were confirmed closed afterward.
- Earlier CSV files in `results/` belong to earlier experiment runs and do
  not establish Pico command or physical stopping behavior.

The RP2040 port in `vcu_firmware/include/config.h` uses unverified placeholder
output pins and keeps `VCU_OUTPUT_WIRING_VERIFIED=0`. Production firmware
was not changed or flashed here. The ROS safety flags for calibration,
vehicle configuration, and braking remain outside this experiment.

## Before any powered test

Verify the installed Pico variant and flashed firmware, as-built pin map,
GP8630N DAC wiring and voltage, BLD-750 EN/BRK/F/R polarity, steering/brake
actuator behavior, independent physical power cutoff, and a measured stopping
method. Establish metric camera calibration, speed and braking distance
measurements, and end-to-end latency. Use the existing commissioning gates in
`docs/BENCH_TESTS.md`; a connected Pico and a software STOP decision do not
satisfy those gates. The requested 2.0 V ceiling is not a qualified crawl speed.
