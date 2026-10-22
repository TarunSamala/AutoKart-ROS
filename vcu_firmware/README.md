# AutoKart VCU firmware — USB-only arming-locked build

**Status: NO-GO for vehicle motion. E-STOP INPUT NOT FITTED - MANUAL POWER CUT ONLY.** Version `0.2.1-no-encoder-locked` was flashed to the
ESP32-S3 and its USB HELLO, SAFE telemetry, and OBS/OBSACK were observed with traction isolated.
It has not been run against the real BLD-750/GP8630N wiring. The default arming lock remains enabled.
The new `0.2.2` source has not been flashed. Software STOP depends on the laptop, USB link and ESP32; the physical battery/supply power cut is the only emergency stop. Keep it within reach. The three ROS verified flags remain false.
See `../docs/HANDOVER_STATUS.md` for the evidence and physical limits.

## Why a new directory

Which firmware image, if any, is actually installed on the board is **unknown** (`docs/architecture/REPOSITORY_MAP.md`
classifies the existing `AutoKart_RC_Dynamics_ESP32S3_v1/` candidates `UNKNOWN_DO_NOT_TOUCH`; `bench/` holds
MicroPython bench scripts, not a deployable VCU). This directory is new and separate so none of those candidates
are modified or reinterpreted as canonical. It is PlatformIO + Arduino-ESP32 (pioarduino core, Arduino-ESP32 3.x /
IDF 5), chosen because the project's one previously *compiled* ESP32 toolchain evidence
(`docs/runbooks/VERIFY_REPOSITORY.md`) found no `idf.py` on this host; PlatformIO installs its own toolchain via
pip without a separate ESP-IDF system install. Record here if that changes.

## Scope

- **Actuates only the BLD-750** (enable, brake line, forward, throttle DAC via the GP8630N). Steering and the
  stepper brake are parsed from the wire protocol but never driven — `F/R` is set once at boot and never changed.
- **No Wi-Fi, no web server, no second command source.** USB serial only, one ASCII protocol
  (`../docs/VCU_PROTOCOL.md`), one 20 Hz command heartbeat.
- `OBS`/`OBSACK` carries Phase 1 camera decisions without requesting throttle. STOP and AT_TARGET observations
  can only disarm. `VCU_OUTPUT_WIRING_VERIFIED=0` refuses every ARM until actual wiring measurements are accepted.
- Fail-safe: any I2C failure, command-link timeout (300 ms), or over-cap command while armed
  latches a FAULT — throttle to 0 V, drive inhibited, brake asserted. A fitted optional E-stop input also faults when LOW. A FAULT needs an explicit `RST` after its
  cause is gone.

## Layout

```
include/   config.h (every pin + polarity macro, each marked VERIFY) · dac_port.h · protocol.h · vcu_logic.h
src/       main.cpp (Arduino entry point) · dac_port.cpp (GP8630N over I2C) · protocol.cpp · vcu_logic.cpp
test/      test_vcu_logic.cpp — host-compiled C++ unit tests (no Arduino, no hardware)
```

## Host test (software only — does not touch hardware)

```
make test
```
Covers: CRC16-CCITT-FALSE, frame parse/corruption, the SAFE/ARMED/FAULT state machine, ARM refusal (no link, no
DAC, nonzero throttle, E-stop not OK), throttle ramp/cap/instant-drop, over-cap `CMD_RANGE` fault, stale/duplicate
sequence rejection, link-timeout fault, E-stop fault, disarm, and DAC-failure fault. **It does not prove anything
about electrical behaviour, boot states, sensor timing, brake force, or an independent E-stop.**

## ⚠️ Polarity reported; powered outputs remain unqualified
The operator reported terminal measurements through the installed interface:
GPIO1 HIGH shorts EN to COM, GPIO2 LOW opens BRK for quick stop, GPIO39 LOW
leaves F/R open for forward. GPIO7 has **no E-stop wire** on this kart.
The macros match those values. This chat did not witness the measurements, and
no DAC SV voltage, combined EN/BRK behavior, manual physical power cut action or
braking response has been qualified. `VCU_OUTPUT_WIRING_VERIFIED=0` still
refuses ARM. Follow `../docs/BENCH_TESTS.md` with traction isolated for output
measurements before considering any powered test.

## Pins (Rev A workbook, VERIFY against the physical board)

| Signal | GPIO | Source |
|---|---|---|
| BLD-750 EN | 1 (via PN2222) | `AutoKart_S3_Pin_and_Power_Budget_RevA.xlsx` |
| BLD-750 BRK | 2 (via PN2222) | same |
| BLD-750 F/R | 39 (via PN2222) | same |
| Optional E-stop status (not fitted) | 7 | no installed wire; ignored by default firmware |
| GP8630N SDA / SCL | 8 / 9 | bench-confirmed, matches `bench/throttle/main.py` |
| AS5600 steering PWM | 13 | Rev A workbook |
| AS5600 brake-position PWM | 14 | Rev A workbook |

These pin numbers came from the workbook, not a continuity check of the as-built harness. No axle encoder is
installed, and the firmware reports counts/speed unavailable as -1. Verify remaining pins before wiring.

Optional monitored upgrade: wire a normally closed (NC) contact from 3.3 V to GPIO7 with an external pull-down so a broken wire reads LOW, then set `ESTOP_INPUT_FITTED=1` and verify HIGH healthy / LOW pressed and the 3 ms filter on the actual board. This monitored signal does not replace the independent physical power cut.

## GP8630N driver (`src/dac_port.cpp`)

Ported from the working MicroPython bench test (`bench/throttle/main.py`): `Wire.begin(8, 9, 100000)`, confirm the
device ACKs at `0x58`, write register `0x01 = 0x1C` (0-10 V mode), wait 200 ms, then write register `0x02` with the
16-bit code `mv / 10000 * 65535` as LOW byte then HIGH byte (bench-verified: 700 mV -> 4587, 2000 mV -> 13107).
There is **no read-back** from this chip — an ACKed I2C transaction is the only health signal, so `dac_init()` and
`dac_write_mv()` return `false` on any failed step, which the firmware treats as `dac_ok=false`: refuse ARM, or
fault `DAC_FAIL` if already armed.

## Stop sequence

On any stop (fault, disarm, or a brake command) `vcu_logic.cpp` drops the target throttle to 0 V in the same
`step()` call that de-asserts EN / asserts BRK. `main.cpp`'s `loop()` is ordered to write the DAC to that 0 V value
*before* touching the EN/BRK GPIOs, so the SV line reaches 0 V first — this is a VERIFY: it matches the general
principle in the BLD-750-class driver manuals of not cutting enable or applying a mechanical brake while the
driver still has a nonzero speed command applied, but the actual BLD-750 manual's exact required sequence has not
been checked line-by-line against this ordering.

## Before the first powered test

See `../docs/BENCH_TESTS.md` for the full checklist (flash + HELLO + `dac_ok`, USB-unplug fault timing, multimeter
checks at 0/1/2 V, EN/BRK/F-R polarity measurement, the 0.75 V fault investigation, the throttle ramp test,
the braking test). `calibration_verified`, `vehicle_config_verified` and `braking_verified` in the ROS parameter
files stay `false` until a person reviews that checklist and sets them — nothing in this repository sets them
automatically.
