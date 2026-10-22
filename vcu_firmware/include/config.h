// AutoKart VCU configuration. Review EVERY value marked VERIFY before connecting the BLD-750.
#pragma once

#define PROTO_VERSION 2
#ifndef ESTOP_INPUT_FITTED
#define ESTOP_INPUT_FITTED 0
#endif
#if ESTOP_INPUT_FITTED != 0 && ESTOP_INPUT_FITTED != 1
#error "ESTOP_INPUT_FITTED must be 0 or 1"
#endif
// RP2040 (Raspberry Pi Pico) build identity: distinct from the ESP32-S3 FW_VERSION strings above so a
// HELLO frame can never be misread as coming from the other board. See README.md "RP2040 (Pico) port".
#if defined(ARDUINO_ARCH_RP2040)
#define FW_VERSION "0.1.0-rp2040-pico-bench-unwired"
#elif ESTOP_INPUT_FITTED
#define FW_VERSION "0.2.2-estop-fitted-locked"
#else
#define FW_VERSION "0.2.2-no-estop-input-locked"
#endif

// ---- Pins (from the GPIO map) ----------------------------------------------
#if defined(ARDUINO_ARCH_RP2040)
// Raspberry Pi Pico (RP2040) port. NONE of these pins are physically wired to the PN2222/BLD-750/GP8630N/
// AS5600 on this kart as of this change -- the bench state in docs/BENCH_TESTS.md (traction isolated, DAC
// disconnected) predates this port and is unaffected by it. These are placeholder assignments chosen to
// avoid the Pico's boot-sensitive pins (GPIO23/24/25/29 and the QSPI flash pins); VERIFY and update every
// one of them against the actual wiring before connecting anything, exactly as config.h's header comment
// already requires for the ESP32-S3 pins below.
#define PIN_BLDC_EN       2   // -> PN2222 -> BLD-750 EN                    [VERIFY, UNWIRED]
#define PIN_BLDC_BRK      3   // -> PN2222 -> BLD-750 BRK                   [VERIFY, UNWIRED]
#define PIN_BLDC_FR       4   // -> PN2222 -> BLD-750 F/R                   [VERIFY, UNWIRED]
#define PIN_ESTOP_STATUS  5   // optional status input; no wire fitted      [VERIFY, UNWIRED]
#define PIN_I2C_SDA       6   // GP8630N throttle DAC (Pico I2C1 SDA)       [VERIFY, UNWIRED]
#define PIN_I2C_SCL       7   // GP8630N throttle DAC (Pico I2C1 SCL)       [VERIFY, UNWIRED]
#define PIN_STEER_PWM     8   // AS5600 steering (PWM)                      [VERIFY, UNWIRED]
#define PIN_BRAKE_PWM     9   // AS5600 brake    (PWM)                      [VERIFY, UNWIRED]
#else
#define PIN_BLDC_EN      1   // -> PN2222 -> BLD-750 EN
#define PIN_BLDC_BRK     2   // -> PN2222 -> BLD-750 BRK
#define PIN_BLDC_FR      39  // -> PN2222 -> BLD-750 F/R
#define PIN_ESTOP_STATUS 7   // optional status input; no wire fitted on this kart
#define PIN_I2C_SDA      8   // GP8630N throttle DAC
#define PIN_I2C_SCL      9
#define PIN_STEER_PWM    13  // AS5600 steering (PWM)
#define PIN_BRAKE_PWM    14  // AS5600 brake    (PWM)
#endif

// ---- Output polarity (GPIO logic level, i.e. BEFORE the PN2222 inverts) ----
// Operator-reported measurements on the installed PN2222 / BLD-750 wiring:
// GPIO1 HIGH -> EN shorted to COM (run permitted).
// GPIO2 LOW -> BRK open from COM (quick electronic stop).
// GPIO39 LOW -> F/R open from COM (forward).
// Optional GPIO7 input: HIGH healthy, LOW pressed/broken wire. No wire is fitted now.
// The earlier HIGH observations also found BRK and F/R shorted to COM, consistent
// with normal operation and reverse. These are operator reports, not a live
// electrical test by this software. GPIO LOW at boot keeps EN open and BRK open.
#define GPIO_EN_ENABLE_LEVEL   HIGH   // operator-reported: HIGH shorts EN to COM
#define GPIO_BRK_ASSERT_LEVEL  LOW    // operator-reported: LOW opens BRK from COM
#define GPIO_FR_FORWARD_LEVEL  LOW    // operator-reported: LOW leaves F/R open; set once at boot
#define ESTOP_OK_LEVEL         HIGH   // applies only with ESTOP_INPUT_FITTED=1

// Keep the physical VCU unarmable despite the reported polarity measurements.
// DAC SV voltage, combined EN/BRK behavior, independent physical power cut, braking,
// and encoder-free ground-speed/pose are not qualified. Host tests override
// this to exercise armed logic; the deployed build retains the lock.
// This default (0) applies identically to the RP2040 (Pico) port above: swapping the
// microcontroller changes nothing about whether the output wiring has been verified,
// and nothing in this file lets the Pico build skip that gate.
#ifndef VCU_OUTPUT_WIRING_VERIFIED
#define VCU_OUTPUT_WIRING_VERIFIED 0
#endif

// Brake line while disarmed (SAFE), as a logical (not GPIO) value: 1 = assert the brake, 0 = release it.
// apply_outputs() / GPIO_BRK_ASSERT_LEVEL above turn this logical value into the actual pin level.
#define BRAKE_LINE_IN_SAFE 1

// ---- Throttle --------------------------------------------------------------
// THROTTLE_MAX_MV is the absolute design ceiling (2 V), enforced here AND in the ROS vehicle bridge
// (kart/ros2_ws/src/autokart_vehicle_bridge/autokart_vehicle_bridge/core.py THROTTLE_MAX_MV) — both
// must be changed together, neither is derived from the other. A command above this cap while ARMED
// is treated as a bug upstream and latches CMD_RANGE, it is never silently clamped.
#define THROTTLE_MAX_MV      2000     // hard cap: 2 V
// THROTTLE_RUN_MV: measured start voltage of the motor on the bench — SV did not move the motor below
// roughly this voltage. The commanded run voltage (whenever actually driving, i.e. not 0 V) is never below
// this unless the kart is stopped entirely. With THROTTLE_RUN_MV == THROTTLE_MAX_MV, speed control is
// effectively on/off: see the vehicle bridge's SpeedController-equivalent gate (core.py throttle_for_speed).
// The earlier bench run also showed a BLD-750 FAULT near 0.75 V whose cause is unresolved (see
// docs/BENCH_TESTS.md) — that fault has NOT reappeared at 2 V in the facts given for this change, but
// investigate it, do not just ignore it.
#define THROTTLE_RUN_MV       2000    // measured motor start voltage; run commands are 0 or >= this
#define THROTTLE_SLEW_MV_S   1000     // maximum ramp-up rate (drops are instant); 0->2 V takes ~2 s, deliberately gentle
#define CMD_TIMEOUT_MS        300

// ---- Throttle DAC (GP8630N over I2C) ---------------------------------------
#define GP8630N_ADDR          0x58    // 7-bit I2C address
#define GP8630N_REG_MODE      0x01    // output-range select register
#define GP8630N_MODE_0_10V    0x1C    // selects the 0-10 V output range
#define GP8630N_REG_DAC       0x02    // DAC code register, 2 bytes: LOW byte then HIGH byte
#define GP8630N_FULL_SCALE_MV 10000   // the configured output range, 0-10 V -> code 0-65535
#define I2C_CLOCK_HZ         100000   // 100 kHz, per the bench test
#define I2C_BUS_TIMEOUT_MS       20   // Wire.setTimeOut(): abort a wedged I2C transaction quickly

// ---- Timing ----------------------------------------------------------------
#define TEL_PERIOD_MS        20
#define HELLO_PERIOD_MS      5000
