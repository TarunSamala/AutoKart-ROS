# AutoKart full test plan — BLDC, brake, steering

Camera/perception/cone-detection is already covered by [`TEST_REPORT.md`](TEST_REPORT.md) and is not
repeated here in full, only referenced where an integration test needs it. This document covers
everything still needed for the **BLDC throttle**, **brake**, and **steering** subsystems, from
first electrical bring-up to a physically-verified powered run — organized as one ordered test
suite per subsystem, plus the integration tests that tie them together.

Every test below states: **purpose**, **prerequisites**, **procedure**, **pass criteria**, and
**current status**. Status uses four values, used honestly and consistently with
[`TEST_REPORT.md`](TEST_REPORT.md)'s convention:

- **PASS** — run, with evidence, and it met the pass criteria.
- **FAIL** — run, with evidence, and it did not.
- **NOT TESTED** — not run yet (hardware missing, prerequisite test not done, or simply not reached).
- **BLOCKED** — cannot be run at all yet because something upstream of it (hardware, a prior test,
  or a measured value) doesn't exist.

**No test in this document arms or powers the vehicle's drive wheels until explicitly marked
otherwise.** The firmware's own interlock (`VCU_OUTPUT_WIRING_VERIFIED=0`) and
`autokart_vcu_safety`'s three verified flags remain the authority on whether `ARM` is accepted —
nothing in this plan, or in running every test in it, changes either of those on its own. A human
has to review the evidence each test produces and deliberately flip those flags, exactly as
`docs/BENCH_TESTS.md`'s gate already states.

---

## 0. Current platform state (as of this session)

- Two VCU firmware targets exist and both build clean: `esp32-s3` (original board,
  `vcu_firmware/platformio.ini`) and `pico` (RP2040 bench port added this session). Both share
  the exact same `vcu_logic.cpp`/`protocol.cpp` safety/wire-protocol code — no divergence there.
- The Pico (`FW_VERSION 0.1.0-rp2040-pico-bench-unwired`) is flashed and was verified live: boots
  `SAFE`, `en=0`, `brk=1`, `dac_mv=0`, `dac_ok=False` (nothing wired to it), `ARM` → `NAK 7`.
- **Nothing is physically wired to the Pico's GPIOs.** The pin assignments in `config.h`'s
  `ARDUINO_ARCH_RP2040` block are placeholders, each marked `[VERIFY, UNWIRED]`.
- The ESP32-S3 board's prior bench state (`docs/BENCH_TESTS.md`, 2026-10-19): USB-only, DAC/BLD-750
  traction stage disconnected, no axle encoder installed, no E-stop input wire.
- The experimental harness (`autokart_bldc test/`) is a **passive, read-only** Pico monitor as of
  the current on-disk state — it transmits no `ARM`/`CMD`/`DIS`/`RST`/`OBS` frame at all, and
  refuses to start if `throttle.motor_output_enabled` is ever set `true`. It cannot run several of
  the tests below by design; those are marked for `tools/vcu_console.py` or the real
  `autokart_vehicle_bridge` instead, same as `docs/BENCH_TESTS.md` already specifies.
- No steering actuator driver exists in this repository yet for either board. `vcu_firmware` only
  *reads* the AS5600 steering PWM signal for telemetry — see §3 below for what's actually missing.

---

## 1. BLDC / throttle test suite

The throttle path is: host command (`CMD` frame, `throttle_mv`) → `vcu_logic.cpp`'s ramp/clamp →
`dac_port.cpp` (I²C) → GP8630N DAC → BLD-750's `SV` analog input.

### 1.1 DAC bus connectivity (`dac_ok`)

- **Purpose:** confirm the GP8630N ACKs on the I²C bus before anything else is attempted. There is
  no read-back from this chip — an ACKed transaction is the *only* health signal `dac_port.cpp` has
  (see its own file comment), so this must be checked explicitly, not assumed from "it compiled."
- **Prerequisites:** GP8630N wired to the VCU's I²C pins and powered at 3.3 V (never from the same
  rail as BLD-750's VCC — see the wiring note in the earlier pinout answer in this conversation).
- **Procedure:** flash the board, open `tools/vcu_console.py <port>` (ESP32-S3) and read `TEL`'s
  `dac_ok` bit (bit4 of `flags`), or on the Pico, use this harness's passive monitor and read
  `vcu_telemetry` → note there's no `dac_ok` surfaced in this harness's trimmed telemetry view yet;
  use `tools/vcu_console.py` against the Pico for the raw `flags` field instead.
- **Pass criteria:** `dac_ok=1` immediately after boot.
- **Status:** **FAIL on the Pico** (`dac_ok=False`, confirmed live this session — expected, nothing
  is wired to it). **NOT TESTED on the ESP32-S3** with current wiring (DAC reported disconnected
  2026-10-19; would need to be reconnected and re-checked, `docs/BENCH_TESTS.md` §a).

### 1.2 DAC voltage accuracy

- **Purpose:** confirm the commanded millivolt value actually appears on the `SV` line, not just
  that the I²C write was ACKed.
- **Prerequisites:** 1.1 PASS, multimeter on `SV` (GP8630N `OUT`) relative to `COM`, wheels off the
  ground, traction isolated is fine for this step (SV can be measured with BLD-750 disconnected).
- **Procedure:** `docs/BENCH_TESTS.md` §(e) — command 0 V / 1.0 V / 2.0 V via `vcu_console.py`'s
  `thr` command and measure each.
- **Pass criteria:** measured voltage within the tool's documented tolerance of each commanded
  value; a command above `THROTTLE_MAX_MV` (2000) faults `CMD_RANGE` rather than exceeding 2 V.
- **Status:** NOT TESTED on either board with current wiring.

### 1.3 Throttle ramp / alarm investigation

- **Purpose:** find the actual voltage where the motor first turns and where any driver alarm
  appears — a previous bench note recorded a BLD-750 fault near 0.75 V whose cause was never
  resolved; this is the test that's supposed to resolve it before 2 V is trusted.
- **Prerequisites:** 1.1–1.2 PASS, BLD-750 connected, motor free to spin, wheels off the ground.
- **Procedure:** `tools/throttle_ramp_test.py <port>` — steps 0→2.0 V in 0.1 V holds; if an alarm or
  fault shows up before 2.0 V, investigate current-limit setting, soft-start trimmer, supply sag
  under load, and SV/COM grounding before accepting 2.0 V as a safe ceiling (`docs/BENCH_TESTS.md`
  §(f) has the full investigation checklist).
- **Pass criteria:** ramp reaches 2.0 V with no alarm, or the alarm's cause is identified and fixed.
- **Status:** NOT TESTED. Not possible until the BLD-750 is reconnected on whichever board is the
  target.

### 1.4 EN line polarity

- **Purpose:** confirm the firmware's `GPIO_EN_ENABLE_LEVEL` macro actually produces "motor can
  run" at the BLD-750's EN pin through the PN2222 stage — a macro value is a claim, not a fact,
  until measured on that exact transistor stage.
- **Prerequisites:** wheels off the ground, motor free to spin, multimeter on the BLD-750 EN pin
  relative to COM.
- **Procedure:** `docs/BENCH_TESTS.md` §(c).1 — `arm`, `thr 0.3`, confirm the motor only spins while
  `en=1` in telemetry, and that the measured EN pin voltage matches what `GPIO_EN_ENABLE_LEVEL`
  through the PN2222 should produce.
- **Pass criteria:** measured voltage state matches the macro's claim exactly; mismatch means fix
  the macro and redo the whole section, not just this one check.
- **Status:** NOT TESTED on either board. The operator-reported polarity values in `config.h`
  (`GPIO_EN_ENABLE_LEVEL=HIGH`, etc.) were measured against the **ESP32-S3's** PN2222 stage — they
  are explicitly not re-verified for the Pico's GPIOs (see the pinout answer earlier: "a GPIO logic
  level that worked through one ESP32-S3 pin's electrical characteristics is not guaranteed to
  behave identically through a Pico GPIO without re-measuring").

### 1.5 Link-timeout fault (`LINK_TIMEOUT`)

- **Purpose:** confirm the firmware independently cuts throttle to 0 V if the host stops sending
  `CMD` frames — this is a hardware-level fail-safe, not something that should depend on the host
  software behaving.
- **Prerequisites:** 1.1–1.2 PASS, wheels off the ground.
- **Procedure:** `docs/BENCH_TESTS.md` §(b) — `arm`, `thr 0.3`, then unplug the USB cable and
  measure the time to `SV = 0 V` with a multimeter. Must be within `CMD_TIMEOUT_MS` (300 ms).
- **Pass criteria:** measured time ≤ 300 ms; VCU shows `FAULT`/`LINK_TIMEOUT` on reconnect, not
  silently re-armed.
- **Status:** NOT TESTED on either board.

### 1.6 Command-range fault (`CMD_RANGE`)

- **Purpose:** confirm a throttle command above the 2000 mV cap is treated as a bug and latches a
  fault, rather than being silently clamped — this is already covered by the **host-tested** logic
  (`vcu_logic.cpp`, unit-tested, verified PASS this session on both the existing suite and after
  the Pico port with no regression) but has not been exercised against real firmware-on-hardware.
- **Status:** PASS (host unit test, `test_vcu_logic.cpp`, confirmed this session). NOT TESTED on
  real hardware (logic is identical on both boards by construction, so a hardware confirmation on
  either board would stand for both, but neither has been done).

### 1.7 Software throttle preview

- **Purpose:** confirm the experimental harness's throttle *preview* logic (never transmitted,
  see §0) stays at 0 mV unless explicitly enabled, ramps correctly, and clamps to the configured
  ceiling.
- **Status:** **PASS** — `tests/test_software_only.py` → `throttle_preview_disabled_by_default`,
  confirmed `dac_mv=0` with `motor_output_enabled=false` across every live run logged in `results/`.

---

## 2. Brake test suite

The brake path is: `CMD` frame's `brake_pct` field (≥50 asserts) or a fault/disarm → `vcu_logic.cpp`
→ `apply_outputs()` → PN2222 → BLD-750's `BRK` pin.

### 2.1 BRK line polarity

- **Purpose:** same reasoning as §1.4 but for the brake line, with an extra failure mode that
  matters more: a **backwards** brake polarity means the kart appears to brake in software while
  the physical brake is actually released.
- **Prerequisites:** wheels off the ground, able to try spinning the wheel by hand.
- **Procedure:** `docs/BENCH_TESTS.md` §(c).2–3 — `dis` (disarm), confirm `brk=1` in telemetry
  (brake asserted by default in `SAFE`, per `BRAKE_LINE_IN_SAFE=1`), measure the BLD-750 BRK pin
  voltage, and **physically try to spin the wheel by hand**. It must resist. If it spins freely
  while `brk=1`, the polarity is backwards — stop, fix `GPIO_BRK_ASSERT_LEVEL`, re-flash, redo the
  entire section.
- **Pass criteria:** wheel resists by hand while `brk=1`; measured pin voltage matches the macro.
- **Status:** NOT TESTED on either board. Confirmed only in software: both the ESP32-S3 and Pico
  builds boot with `brk=1` in telemetry (the Pico build was verified live this session: `brk=1` at
  boot, matching `BRAKE_LINE_IN_SAFE=1`) — but that is the *logical* state only; nothing has
  measured the actual BLD-750 BRK pin or tried to spin a wheel on either board.

### 2.2 Brake-under-load response

- **Purpose:** confirm the brake actually engages (not just the logical flag) while armed and
  moving, and that throttle drops to 0 V *before* or simultaneously with the brake engaging, per
  the documented shutdown sequence (`dac_write_mv(0)` happens before `apply_outputs()` in
  `main.cpp`'s `loop()` — this ordering is preserved identically in the Pico port, unchanged code).
- **Procedure:** `docs/BENCH_TESTS.md` §(c).4 — `arm`, `thr 0.3`, `brake 100`; confirm `brk=1`,
  wheel resists the same way as 2.1, and measure the delay between the brake command and `SV`
  reaching 0 V.
- **Pass criteria:** brake engages, SV reaches 0 V promptly (bench-recorded delay, no fixed
  threshold given in the source docs beyond "measure it").
- **Status:** NOT TESTED on either board.

### 2.3 Physical stopping distance / braking test

- **Purpose:** produce the one number this entire project is gated on —
  `brake_min_mps2`, which both `autokart_vcu_safety`'s `braking_verified` flag and this
  experimental harness's `obstacle_checker.py` depend on to compute any real stopping clearance.
- **Procedure:** `tools/braking_test.py`, per `docs/BENCH_TESTS.md` §(h).
- **Pass criteria:** repeatable measured deceleration across multiple runs; `brake_min_mps2` set to
  80% of the smallest observed value.
- **Status:** **BLOCKED.** `docs/BENCH_TESTS.md` states explicitly: *"The tool still requires
  encoder telemetry, which this kart lacks, and therefore cannot measure braking here. No braking
  test is authorized by this document or this software change."* Neither board has an axle encoder
  installed. This is not a software gap — it's a missing sensor. Until an encoder is installed and
  calibrated (`docs/BENCH_TESTS.md` §(g1)), this test cannot run on *either* board, and
  `config/test_config.yaml`'s `stopping.measured` stays `false` in this harness.

### 2.4 Software stop-controller decision logic

- **Purpose:** confirm the mission state machine actually transitions to a STOP request the moment
  an obstacle is detected inside the stop zone, and stays there.
- **Status:** **PASS** — `tests/test_software_only.py` → `stop_controller_state_machine`: three
  clear-path steps reach `DRIVING`, then an obstacle input transitions to `STOP_REQUESTED` with
  `decision=0` in the same run. Also confirmed in every live run: the harness held `decision=0`
  continuously across every frame logged in `results/*.csv` because perception never authorized a
  path. This is a *software decision* only (§0) — it has never been confirmed against an actual
  physical stop, which needs 2.1–2.3 above plus an armed run.

---

## 3. Steering test suite

**This is the subsystem with the biggest real gap.** The hardware in the original request
(NEMA34 stepper + R86mini driver) has **no firmware interface at all** in this repository. What
exists today (`vcu_firmware`) only *reads* an AS5600 PWM signal for steering telemetry — it does
not drive anything. `docs/VCU_PROTOCOL.md` is explicit: *"steer_cdeg is parsed and ignored —
steering is observed, never actuated, in this release."* This is true on both the ESP32-S3 and the
new Pico port identically, because it's the same unmodified `vcu_logic.cpp`.

So before any "steering test" in the usual bench sense, there's a build step this plan has to name
honestly rather than skip past:

### 3.0 Steering actuator driver — not yet built

- **What's missing:** STEP/DIR (or equivalent) output pins to the R86mini driver, a firmware module
  to generate step pulses for the NEMA34, a steering angle→step-count mapping, and a zero/limit
  reference (the R86mini is a stepper driver; it has no inherent "center" the firmware can read
  back — some kind of homing/limit switch or an absolute encoder in a closed loop would be needed
  to know where "straight ahead" is on power-up).
- **Decision needed before writing it:** does steering stay open-loop (step count only, assume it
  never loses steps) or closed-loop (AS5600 feedback used to verify/correct position, not just
  report it)? The current AS5600 wiring is already present for *feedback*, which argues for closed-
  loop, but that's a architecture decision for a person to make, not inferred here.
- **Status:** NOT BUILT. Not "NOT TESTED" — there is no code path to test yet. This needs a design
  decision and a firmware change before any of 3.1–3.4 below are reachable.

### 3.1 AS5600 steering PWM read validity (existing capability)

- **Purpose:** confirm the firmware's existing PWM-capture code (`pwm_raw()` in `main.cpp`, shared
  unmodified between both board ports) produces a valid raw reading from a real AS5600 sensor.
- **Prerequisites:** AS5600 wired to `PIN_STEER_PWM` and powered.
- **Procedure:** read `TEL`'s `steer_raw` field (0–4095, or −1 if invalid) while manually rotating
  the steering linkage; confirm it tracks smoothly with no dropouts, per the ~920 Hz PWM assumptions
  documented in `main.cpp`'s `pwm_raw()` comment ("VERIFY on bench").
- **Pass criteria:** `steer_raw` tracks continuously across the full steering range with no −1
  glitches during normal motion.
- **Status:** NOT TESTED on either board — no AS5600 is currently wired to either board's
  `PIN_STEER_PWM`.

### 3.2 Steering zero/scale calibration

- **Purpose:** establish the mapping from `steer_raw` counts to an actual steering angle in
  degrees/radians, and the zero offset for "wheels straight."
- **Prerequisites:** 3.1 PASS.
- **Procedure:** not yet scripted anywhere in this repo (unlike the encoder/latency tools, there is
  no `tools/steering_calibration.py`). Would need: steer to a known physical angle (e.g. using a
  protractor/jig against the front wheel), record `steer_raw`, repeat at several angles, fit
  raw→angle.
- **Status:** NOT TESTED. Tooling doesn't exist yet either — flagging this as a real gap, not
  implying it's close.

### 3.3 Software steering angle preview

- **Purpose:** confirm the experimental harness's curvature→steering-angle preview math
  (`control/steering_controller.py`, bicycle-model `atan(wheelbase * curvature)`) is computed
  correctly from the geometry engine's existing curvature profile.
- **Status:** **PASS** (synthetic input only) — `tests/test_software_only.py` →
  `steering_preview_logic`: for a synthetic `curvature_m_inv=0.2` sample with
  `wheelbase_m=1.15`, computed `steering_angle_rad=0.226`. Not yet exercised with a real multi-cone
  corridor (blocked on the same `NO_PATH`/single-cone-visibility limitation noted in
  `TEST_REPORT.md` item 3). This value is a **preview only** — see §0 and §3.0: there is no
  actuator to send it to yet.

### 3.4 Closed-loop steering actuation (if the closed-loop design is chosen)

- **Purpose:** confirm commanded step counts actually move the NEMA34 to the intended angle and
  that AS5600 feedback matches within tolerance.
- **Status:** **BLOCKED** on §3.0 (driver doesn't exist) and §3.2 (no calibration method yet).

---

## 4. F/R (direction) test

- **Purpose:** confirm the F/R line's logic level actually corresponds to "forward," and that
  direction is set once at boot and never changed at runtime, matching the firmware's documented
  design (`main.cpp`: *"forward only; direction is never changed at run time"*).
- **Procedure:** `docs/BENCH_TESTS.md` §(c).5 — wheel free, `arm`, `thr 0.3`, confirm the observed
  turn direction matches "forward" for the kart's actual orientation.
- **Status:** NOT TESTED on either board.

---

## 5. E-stop / physical power cut test

- **Purpose:** confirm the one actual emergency stop this kart has — the manual battery/supply
  disconnect — physically removes traction power, independent of any firmware state.
- **Procedure:** `docs/BENCH_TESTS.md` §(d). Note the firmware's E-stop *input* (`ESTOP_INPUT_FITTED`)
  is a separate, optional, currently-disabled monitoring feature (both boards build with it `0` by
  default) — it is explicitly **not** the E-stop itself, just an optional status readback the
  firmware cannot act on alone.
- **Status:** NOT TESTED on either board. No E-stop input wire is fitted on either board
  (`estop_input_fitted=False` confirmed live on the Pico this session, matching the ESP32-S3's
  documented state).

---

## 6. Integration / system-level tests

### 6.1 VCU identity and passive link health

- **Status:** **PASS on the Pico** — this session confirmed, read-only, via this harness's
  `vcu/serial_interface.py`: `HELLO` reports the exact expected firmware identity
  (`0.1.0-rp2040-pico-bench-unwired`, proto 2), telemetry streams with a fresh `link_age_ms`, and a
  manual `ARM` sent via a separate script (not this harness) correctly got `NAK 7`. **NOT TESTED**
  on the ESP32-S3 in this session (no serial device was available when that board was last
  connected — see the earlier USB-enumeration troubleshooting in this conversation).

### 6.2 End-to-end "stop before cone" — software decision only

- **Status:** **PASS** (software only) — see §2.4 and `TEST_REPORT.md` item 9. Every live run held
  `decision=STOP` continuously; this has never been checked against a physically moving kart
  because 1.1–1.6, 2.1–2.3, and the ARM gate are all still open.

### 6.3 First powered ground run

- **Prerequisites (unchanged from `docs/BENCH_TESTS.md` §(i), restated here for completeness):**
  1. Sections 1.1–1.6 (BLDC) and 2.1–2.2 (brake) all PASS, measured, on the board actually being
     armed.
  2. §2.3 (braking test) PASS — which is **BLOCKED** on an axle encoder not existing yet.
  3. §4 (F/R direction) and §5 (E-stop/power cut) PASS.
  4. A steering decision made and, if closed-loop is chosen, §3.0–3.4 built and PASS. (Note: the
     original mission could technically run with steering unactuated/open-loop on a short straight
     lane, same as the firmware's current "observed, never actuated" steering state — that's a
     scope decision for a person, not assumed here.)
  5. `calibration_verified`, `vehicle_config_verified`, `braking_verified` all set `true` in
     `kart/ros2_ws/src/autokart_vcu_safety/config/vcu_safety.yaml` by a person who reviewed the
     actual evidence from the tests above — this plan does not set them and has no path to.
  6. Arming done through the existing `tools/vcu_console.py` or `autokart_vehicle_bridge`, outside
     this experimental harness, which (as of the current on-disk state) has no `ARM` code path at
     all.
- **Status:** **BLOCKED.**

---

## 7. Status summary

| Subsystem | Tests PASS | Tests FAIL | NOT TESTED | BLOCKED / NOT BUILT |
|---|---|---|---|---|
| Camera / perception | see `TEST_REPORT.md` (8 PASS) | — | 2 | — |
| BLDC / throttle | 2 (§1.6 host logic, §1.7 preview) | 1 (§1.1 on Pico, expected) | 4 (§1.1 ESP32-S3, §1.2–1.4) | 0 |
| Brake | 1 (§2.4 software) | — | 3 (§2.1, §2.2, boot-state only confirmed) | 1 (§2.3, no encoder) |
| Steering | 1 (§3.3 synthetic preview) | — | 2 (§3.1, §3.2) | 2 (§3.0 driver not built, §3.4) |
| F/R direction | — | — | 1 (§4) | 0 |
| E-stop / power cut | — | — | 1 (§5) | 0 |
| Integration | 2 (§6.1 Pico, §6.2 software) | — | 1 (§6.1 ESP32-S3) | 1 (§6.3 powered run) |

## 8. What actually unblocks the most tests next

In order of leverage:

1. **Install and calibrate an axle encoder** — unblocks §2.3 (braking test), which unblocks
   `braking_verified`, which is one of three hard gates on §6.3.
2. **Reconnect the DAC/BLD-750 to whichever board is the real target** and work through §1.1–1.6,
   §2.1–2.2, §4 as a single bench session, exactly as `docs/BENCH_TESTS.md` already sequences them.
3. **Decide the steering architecture** (open-loop vs. closed-loop) — everything in §3 is stalled
   on that one decision, not on missing parts.
4. Fit the E-stop input wire (§5's optional monitored upgrade) and complete the physical power-cut
   verification — independent of the above, can happen any time.

None of these are software changes to safety logic. They are physical bring-up work and one
architecture decision, matching exactly what `docs/BENCH_TESTS.md` already asks for — this document
just lays BLDC, brake, and steering out side by side with explicit pass criteria instead of as one
long sequential checklist.
