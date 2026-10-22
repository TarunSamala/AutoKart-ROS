// AutoKart VCU configuration. Review EVERY value marked VERIFY before connecting the BLD-750.
#pragma once

#define FW_VERSION "0.1.0"
#define PROTO_VERSION 1

// ---- Pins (from the GPIO map) ----------------------------------------------
#define PIN_BLDC_EN      1   // -> PN2222 -> BLD-750 EN
#define PIN_BLDC_BRK     2   // -> PN2222 -> BLD-750 BRK
#define PIN_BLDC_FR      39  // -> PN2222 -> BLD-750 F/R
#define PIN_ESTOP_STATUS 7   // isolated auxiliary contact, monitor only
#define PIN_I2C_SDA      8   // GP8630N throttle DAC
#define PIN_I2C_SCL      9
#define PIN_AXLE_A       15  // AS5047P A
#define PIN_AXLE_B       16  // AS5047P B
#define PIN_STEER_PWM    13  // AS5600 steering (PWM)
#define PIN_BRAKE_PWM    14  // AS5600 brake    (PWM)

// ---- Output polarity (GPIO logic level, i.e. BEFORE the PN2222 inverts) ----
// Hardware rule: the PN2222 base pull-downs make the reset/boot state GPIO = LOW.
// That state MUST mean "drive inhibited". VERIFY against your BLD-750 wiring.
//
// VERIFY — UNCONFIRMED. On the bench, EN and BRK were both jumpered BY HAND directly to COM (not through
// the PN2222 stage) and the motor ran; that says something about the BLD-750's EN/BRK inputs, but nothing
// about which GPIO level this firmware's PN2222 open-collector outputs need to reproduce "shorted to COM"
// vs "open" — that depends on the PN2222 wiring (NPN vs PNP, pull-up vs pull-down) which has not been
// measured. Do NOT infer a value from the bench observation; the levels below are the firmware's ORIGINAL
// unverified defaults, kept as-is. Confirm every one of them with a multimeter, through the actual PN2222
// interface, per docs/BENCH_TESTS.md, before connecting the BLD-750 — see the warning in README.md.
#define GPIO_EN_ENABLE_LEVEL   HIGH   // GPIO HIGH = transistor on = drive enabled (VERIFY)
#define GPIO_BRK_ASSERT_LEVEL  HIGH   // GPIO HIGH = BRK asserted (VERIFY — unconfirmed, see note above)
#define GPIO_FR_FORWARD_LEVEL  LOW    // VERIFY: level that selects FORWARD. Only set at boot.
#define ESTOP_OK_LEVEL         HIGH   // VERIFY: level read when the E-stop circuit is healthy

// Brake line while disarmed (SAFE), as a logical (not GPIO) value: 1 = assert the brake, 0 = release it.
// apply_outputs() / GPIO_BRK_ASSERT_LEVEL above turn this logical value into the actual pin level.
#define BRAKE_LINE_IN_SAFE 1

// ---- Throttle --------------------------------------------------------------
// THROTTLE_MAX_MV is the absolute design ceiling (2 V), enforced here, in the bridge command gate
// (autokart_vehicle_bridge), and in the controller (autokart_controller/core.py THROTTLE_V_MAX) — all three
// must be changed together, they are not derived from one another. A command above this cap while ARMED
// is treated as a bug upstream and latches CMD_RANGE, it is never silently clamped.
#define THROTTLE_MAX_MV      2000     // hard cap: 2 V
// THROTTLE_RUN_MV: measured start voltage of the motor on the bench — SV did not move the motor below
// roughly this voltage. The commanded run voltage (whenever actually driving, i.e. not 0 V) is never below
// this unless the kart is stopped entirely. With THROTTLE_RUN_MV == THROTTLE_MAX_MV, speed control is
// effectively on/off: see autokart_controller/core.py SpeedController. The earlier bench run also showed a
// BLD-750 FAULT near 0.75 V whose cause is unresolved (see docs/BENCH_TESTS.md) — that fault has NOT
// reappeared at 2 V in the facts given for this change, but investigate it, do not just ignore it.
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

// ---- Axle encoder (AS5047P ABI) -------------------------------------------
#define AXLE_CPR             4096     // VERIFY: counts per wheel rev after x4 quadrature
#define TYRE_RADIUS_M        0.10f    // VERIFY: effective rolling radius
#define AXLE_MOTION_COUNTS   20       // counts before "motion seen" flag is set

// ---- Timing ----------------------------------------------------------------
#define TEL_PERIOD_MS        20
#define HELLO_PERIOD_MS      5000
