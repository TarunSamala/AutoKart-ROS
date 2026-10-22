# VCU commissioning checklist — BLD-750 / GP8630N

**Do this with the kart's drive wheels OFF THE GROUND and someone holding the physical E-stop.** Every step below
is a bench check; none of it is a driving test. Stop and investigate at the first sign of anything other than the
described behaviour — do not push through an unexpected result.

This checklist exercises `esp32_firmware/src/dac_port.cpp` (GP8630N over I2C), the DAC-then-EN/BRK stop sequencing
in `main.cpp`, and the EN/BRK/F-R/E-stop polarity macros in `esp32_firmware/include/config.h` — **none of these
polarities have been measured through the real PN2222 interface**, see the warning in the top-level `README.md`.
Fill in every blank below as you go; this file is the record, not just a script.

## Before you start
- Confirm the GP8630N module's allowed VCC is 3.3 V (do not assume — check the module's silkscreen/datasheet; some
  GP8630N breakout boards are 5 V-only on the logic side even though the output stage goes to 10 V).
  Result: _________________________
- Confirm wiring: ESP32 GPIO8 = SDA, GPIO9 = SCL to the GP8630N; GP8630N OUT -> BLD-750 SV; GP8630N GND -> BLD-750 COM.
- Have a multimeter on the SV line (GP8630N OUT to BLD-750 COM) for every step below.
- Flash the firmware: `cd esp32_firmware && pio run -t upload`.

## (a) HELLO and dac_ok
1. `python3 pi/tools/vcu_console.py /dev/ttyACM0`.
2. Confirm a `HELLO` line appears (firmware version, protocol version) within 5 s of reset. Result: ______
3. Confirm the telemetry line reports `dac_ok=1` (bit4 of the `flags` field, see `docs/PROTOCOL.md`). If it is 0,
   stop here: `dac_init()` failed — check the I2C wiring and the GP8630N's address (0x58) before going further.
   Result: ______
4. Try `arm`: with `dac_ok=1` it should be refused only if some other precondition fails (E-stop, link). It must
   still be refused if you unplug the GP8630N and retry (`dac_ok` should drop to 0 and ARM must NAK with "DAC").
   Result: ______

## (b) Unplug-USB fault and recovery
1. `arm`, `thr 0.3` (motor spinning freely, wheels off the ground).
2. Unplug the USB cable. Within 300 ms (`CMD_TIMEOUT_MS`) the VCU must fault (`LINK_TIMEOUT`) on its own.
   Confirm with the multimeter that SV is back at 0 V within that window. Measured time to 0 V: ______ ms
3. Reconnect, reopen `vcu_console.py`. Confirm the VCU is in FAULT (not silently re-armed) and needs `rst`.
   Result: ______

## (c) EN / BRK / F-R / E-stop polarity — measure, do not infer
`config.h` ships its **original, unverified** polarity macros. The bench fact that EN and BRK were jumpered by hand
to COM and the motor ran tells you something about the BLD-750's inputs, nothing about which ESP32 GPIO level the
PN2222 stage needs to reproduce that state. **Do not guess; measure each one.**
1. With the motor wheel free to spin by hand (not under load, wheels off the ground), `arm`, `thr 0.3`. Confirm the
   motor spins only while `en=1` in telemetry. Measure the actual voltage at the BLD-750 EN pin (relative to COM)
   in this state: ______ V. Does it match what `GPIO_EN_ENABLE_LEVEL` implies through the PN2222? ______
2. `dis` (disarm). Confirm the motor is free to spin by hand again (EN released) and that `brk=1` in telemetry
   (brake asserted by default in SAFE, per `BRAKE_LINE_IN_SAFE`). Measure the BLD-750 BRK pin voltage in this
   state: ______ V.
3. Try to spin the wheel by hand while disarmed. Does the axle resist turning (brake engaged) or spin freely?
   Result: ______. If it spins freely while `brk=1` is reported, the polarity is **backwards**: stop, do not arm
   again, and fix `GPIO_BRK_ASSERT_LEVEL` in `config.h` before retrying (then redo this whole section).
4. `arm`, `thr 0.3`, then `brake 100`. Confirm `brk=1` in telemetry, that the axle resists turning the same way it
   did in step 3, and that `dac_mv` drops to 0 immediately (braking forces throttle to zero). Measure the delay
   between the brake command and SV reaching 0 V: ______ ms (should be effectively instant, per `vcu_logic.cpp`).
5. Confirm `GPIO_FR_FORWARD_LEVEL`: with the wheel free, `arm`, `thr 0.3`, and confirm the wheel turns in the
   direction you expect as "forward" for the kart. Direction observed: ______ (matches expectation: Y/N)
6. Confirm `ESTOP_OK_LEVEL`: with the E-stop circuit healthy, telemetry should report `estop=1`. Trip the E-stop
   (see step (d)) and confirm it flips to `estop=0`.
7. Only once every measurement above matches what the macros claim should you treat this section as PASSED. If any
   mismatch, fix the specific macro, re-flash, and redo this entire section — do not assume fixing one macro
   doesn't affect another.
   **Section result (PASS / FAIL, date, initials): ______________________**

## (d) Physical E-stop
1. `arm`, `thr 0.3`.
2. Press the physical E-stop.
3. Confirm the VCU faults (`ESTOP` bit), SV drops to 0 V, and `rst` is refused (NAK "CAUSE_PRESENT") until the
   E-stop is released. Result: ______

## (e) Measure SV with a multimeter at 0, 1, 2 V
With the meter on SV (relative to COM), wheels off the ground:
1. `arm`, `thr 0.0`: SV should read ~0 V. Measured: ______ V
2. `thr 1.0`: SV should read close to 1.0 V. Measured: ______ V
3. `thr 2.0`: SV should read close to 2.0 V and must **never** exceed `THROTTLE_MAX_MV` (2000 mV, the absolute
   design ceiling, enforced in `config.h`, the bridge, and the controller). Measured: ______ V
4. Try a throttle command above the cap from the console and confirm the firmware **faults** (`CMD_RANGE`) rather
   than outputting more than 2 V — this proves the firmware-side cap is real, not just a console-side clamp.
   Result: ______
5. `rst` to clear the fault, `dis` to disarm.
6. DAC accuracy (commanded vs measured, from steps 1-3): ______________________

## (f) Throttle ramp test — find the alarm voltage, don't just avoid it
Run `python3 pi/tools/throttle_ramp_test.py /dev/ttyACM0`. It requires a typed `YES` confirming wheels off the
ground and E-stop in hand, then steps 0 -> 2.0 V in 0.1 V increments, holding each step 3 s, printing telemetry and
asking "driver alarm LED on? (y/n)" at every step. It returns to 0 V and disarms at the first `y`, at Ctrl-C, or if
the VCU itself faults.
1. Run it. Voltage where the motor first turned (from the tool's output): ______ V
2. Voltage where the driver alarm LED first appeared, if any (from the tool's output): ______ V
3. The earlier bench run showed a fault near **0.75 V**. Did it reproduce here? ______. If yes or if any other
   alarm voltage appeared below 2.0 V, investigate — do not just note it and move on:
   - Driver current-limit setting: ______________________
   - Acceleration / soft-start trimmer settings on the BLD-750: ______________________
   - Supply voltage sag under load (measure supply at the alarm voltage): ______________________
   - Grounding of SV and COM (continuity, shared ground with ESP32 logic ground?): ______________________
   - Noise on SV (scope if available; a GP8630N glitch during a step could look like an alarm): ______________________
   - Conclusion / fix applied: ______________________
4. Only once the ramp reaches 2.0 V with no alarm (or the alarm cause above is understood and fixed) should 2.0 V
   be treated as safe to run at.
   **Section result (PASS / FAIL, date, initials): ______________________**

## (g) Braking test — real deceleration at 2 V
Run `python3 pi/tools/braking_test.py /dev/ttyACM0 --volts 2.0 --runs 3` in an open, flat, clear area, with the
physical E-stop in hand. Start lower (e.g. `--volts 0.8`) first if this is the first time the kart has driven under
its own power at all, and only move to `--volts 2.0` once low-speed runs behave as expected.
1. Measured steady speed at 2.0 V (`v_run`): ______ m/s — enter this where your process tracks `v_run at 2 V`.
2. Measured deceleration: mean ______ m/s², worst ______ m/s².
3. Measured reaction latency (brake command -> speed drop): ______ ms.
4. Copy the tool's suggested `a_verified_mps2`, `t_latency_s`, and `volts_per_mps` into
   `pi/ros2_ws/src/autokart_bringup/config/autokart.yaml`.
5. Only set `braking_verified: true` in that same file once the runs are repeatable. **This is your decision to
   make after reviewing the numbers — nothing in this repo sets it for you.**

## Gate
`calibration_verified` (`autokart_perception`), `vehicle_config_verified` and `braking_verified`
(`autokart_safety`) all stay `false` in `autokart.yaml` until you — the person who ran this checklist — set them
after reviewing the actual measurements above. No script or tool in this repo sets them automatically; the safety
node (`autokart_safety/core.py`) refuses to arm while any of them is `false`.
