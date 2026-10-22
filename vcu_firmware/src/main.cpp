// AutoKart VCU firmware (ESP32-S3 or RP2040/Pico, Arduino framework -- see platformio.ini envs).
// Scope of this release: receive commands over USB serial and actuate the BLDC (BLD-750) only.
// Steering and brake fields are parsed and reported but NOT actuated.
// The RP2040 (Pico) build is an unwired bench port: see README.md "RP2040 (Pico) port" and the
// [VERIFY, UNWIRED] pin comments in config.h. It changes no safety logic (vcu_logic.cpp/protocol.cpp
// are untouched, platform-independent, and still host-tested) and inherits the same
// VCU_OUTPUT_WIRING_VERIFIED=0 lock, so ARM still NAKs with code 7 on this board too.
#include <Arduino.h>
#include <math.h>
#if defined(ARDUINO_ARCH_RP2040)
#include <hardware/watchdog.h>
#ifndef IRAM_ATTR
#define IRAM_ATTR  // ESP-only attribute (places code in internal RAM); no equivalent needed on RP2040
#endif
#else
#include <esp_task_wdt.h>
#endif

#include "config.h"
#include "dac_port.h"
#include "estop_input.h"
#include "protocol.h"
#include "vcu_logic.h"

static VcuLogic logic;
static bool dac_ok = false;

// ---------------- AS5600 PWM inputs ----------------
struct PwmIn {
  volatile uint32_t t_rise_us = 0, period_us = 0, high_us = 0, last_us = 0;
};
static PwmIn steer_pwm, brake_pwm;

static void IRAM_ATTR pwm_edge(PwmIn& p, int pin) {
  uint32_t now = micros();
  if (digitalRead(pin)) {
    p.period_us = now - p.t_rise_us;
    p.t_rise_us = now;
  } else {
    p.high_us = now - p.t_rise_us;
    p.last_us = now;
  }
}
static void IRAM_ATTR isr_steer() { pwm_edge(steer_pwm, PIN_STEER_PWM); }
static void IRAM_ATTR isr_brake() { pwm_edge(brake_pwm, PIN_BRAKE_PWM); }

// AS5600 PWM: ~920 Hz, 4351 clocks per period, 128 clocks header, 4096 clocks of position. VERIFY on bench.
static int pwm_raw(PwmIn& p) {
  noInterrupts();
  uint32_t period = p.period_us, high = p.high_us, last = p.last_us;
  interrupts();
  if ((uint32_t)(micros() - last) > 100000u) return -1;
  if (period < 900 || period > 1300) return -1;
  long raw = lroundf((float)high * 4351.0f / (float)period) - 128;
  if (raw < -4 || raw > 4100) return -1;
  if (raw < 0) raw = 0;
  if (raw > 4095) raw = 4095;
  return (int)raw;
}

// ---------------- helpers ----------------
static void apply_outputs(const VcuOutputs& o) {
  digitalWrite(PIN_BLDC_BRK, o.brk ? GPIO_BRK_ASSERT_LEVEL : !GPIO_BRK_ASSERT_LEVEL);
  digitalWrite(PIN_BLDC_EN, o.en ? GPIO_EN_ENABLE_LEVEL : !GPIO_EN_ENABLE_LEVEL);
}

static void send_frame(const char* payload) {
  char buf[200];
  int n = frame_build(payload, buf, sizeof(buf));
  if (n > 0) Serial.write((const uint8_t*)buf, n);
}

static void send_nak(uint32_t seq, int code) {
  char p[40];
  snprintf(p, sizeof(p), "NAK,%lu,%d", (unsigned long)seq, code);
  send_frame(p);
}

#if ESTOP_INPUT_FITTED
static bool estop_ok_raw() { return digitalRead(PIN_ESTOP_STATUS) == ESTOP_OK_LEVEL; }
static bool estop_ok_filtered() {
  static EstopFilter filter;
  return filter.update(estop_ok_raw());  // 3 consecutive bad reads (about 3 ms)
}
#endif

static uint32_t fw_tel_seq = 0;

static VcuInputs make_inputs() {
  VcuInputs in;
  in.now_ms = millis();
#if ESTOP_INPUT_FITTED
  in.estop_ok = effective_estop_ok(estop_ok_filtered());
#else
  in.estop_ok = effective_estop_ok(false);
#endif
  in.dac_ok = dac_ok;
  return in;
}

static void handle_line(const char* line) {
  char payload[120];
  if (frame_parse(line, payload, sizeof(payload)) < 0) return;
  char* f[8];
  int n = split_fields(payload, f, 8);
  uint32_t now = millis();
  long seq = 0;
  if (n >= 2 && !parse_i32(f[1], &seq)) return;

  if (!strcmp(f[0], "CMD") && n == 5) {
    long thr, st, br;
    if (!parse_i32(f[2], &thr) || !parse_i32(f[3], &st) || !parse_i32(f[4], &br)) return;
    logic.on_cmd(now, (uint32_t)seq, thr, st, br);
  } else if (!strcmp(f[0], "OBS") && n == 7) {
    long stamp_ms, cone_id, forward_mm, gap_mm, decision;
    if (!parse_i32(f[2], &stamp_ms) || !parse_i32(f[3], &cone_id) ||
        !parse_i32(f[4], &forward_mm) || !parse_i32(f[5], &gap_mm) ||
        !parse_i32(f[6], &decision)) return;
    if (stamp_ms < 0 || cone_id < -1 || forward_mm < -1 ||
        !logic.on_observation((int)decision)) return;
    char ack[48];
    snprintf(ack, sizeof(ack), "OBSACK,%lu,%ld", (unsigned long)(uint32_t)seq, decision);
    send_frame(ack);
  } else if (!strcmp(f[0], "ARM") && n == 2) {
    int r = logic.on_arm((uint32_t)seq, make_inputs());
    if (r) send_nak((uint32_t)seq, r);
  } else if (!strcmp(f[0], "DIS") && n == 2) {
    logic.on_disarm();
  } else if (!strcmp(f[0], "RST") && n == 2) {
    int r = logic.on_reset(make_inputs());
    if (r) send_nak((uint32_t)seq, r);
  }
}

static void poll_serial() {
  static char buf[128];
  static size_t len = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      buf[len] = '\0';
      if (len) handle_line(buf);
      len = 0;
    } else if (c != '\r') {
      if (len < sizeof(buf) - 1) buf[len++] = c;
      else len = 0;  // overflow: drop the frame
    }
  }
}

void setup() {
  // 1) Safe pin states before anything else. (Hardware pull-downs cover the time before this runs.)
  pinMode(PIN_BLDC_EN, OUTPUT);
  pinMode(PIN_BLDC_BRK, OUTPUT);
  pinMode(PIN_BLDC_FR, OUTPUT);
  digitalWrite(PIN_BLDC_EN, !GPIO_EN_ENABLE_LEVEL);
  digitalWrite(PIN_BLDC_BRK, BRAKE_LINE_IN_SAFE ? GPIO_BRK_ASSERT_LEVEL : !GPIO_BRK_ASSERT_LEVEL);
  digitalWrite(PIN_BLDC_FR, GPIO_FR_FORWARD_LEVEL);  // forward only; direction is never changed at run time
#if ESTOP_INPUT_FITTED
  pinMode(PIN_ESTOP_STATUS, INPUT);
#endif

  Serial.begin(115200);

  // 2) Throttle DAC to 0 V.
  dac_ok = dac_init() && dac_write_mv(0);

  // 3) Sensors.
  // No axle encoder is installed. TEL uses -1 for counts and speed so the host
  // cannot mistake an unmeasured speed for a stationary kart.
  pinMode(PIN_STEER_PWM, INPUT);
  pinMode(PIN_BRAKE_PWM, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_STEER_PWM), isr_steer, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BRAKE_PWM), isr_brake, CHANGE);

  // 4) Watchdog (1 s). A reset leaves pins to the hardware pull-downs, which must mean "inhibited".
#if defined(ARDUINO_ARCH_RP2040)
  watchdog_enable(1000, true);  // pico-sdk hardware watchdog: 1000 ms, pause on debug halt
#elif defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdt = {.timeout_ms = 1000, .idle_core_mask = 0, .trigger_panic = true};
  esp_task_wdt_reconfigure(&wdt);
  esp_task_wdt_add(NULL);
#else
  esp_task_wdt_init(1, true);
  esp_task_wdt_add(NULL);
#endif

  char p[48];
  snprintf(p, sizeof(p), "HELLO,%s,%d", FW_VERSION, PROTO_VERSION);
  send_frame(p);
}

void loop() {
#if defined(ARDUINO_ARCH_RP2040)
  watchdog_update();
#else
  esp_task_wdt_reset();
#endif
  static uint32_t last_tel = 0, last_hello = 0, last_dac = 0;

  poll_serial();

  VcuInputs in = make_inputs();
  VcuOutputs out = logic.step(in);

  // Write the DAC BEFORE touching EN/BRK: on any stop (fault, disarm, brake) logic.step() already set
  // out.dac_mv to 0 in this same call, so this ordering gets the SV line to 0 V first and only then
  // de-asserts EN / asserts BRK, matching the shutdown sequence the BLD-750 manual expects (never cut
  // enable or apply the mechanical brake while the driver still has a nonzero speed command applied).
  static uint16_t last_dac_mv = 0xFFFF;
  if (out.dac_mv != last_dac_mv || (in.now_ms - last_dac) >= 20) {
    dac_ok = dac_write_mv(out.dac_mv);
    last_dac_mv = out.dac_mv;
    last_dac = in.now_ms;
  }

  apply_outputs(out);

  uint32_t now = in.now_ms;
  if (now - last_tel >= TEL_PERIOD_MS) {
    last_tel = now;

    int sraw = pwm_raw(steer_pwm), braw = pwm_raw(brake_pwm);
    uint32_t flags = estop_telemetry_flags(in.estop_ok) |
                     (sraw >= 0 ? 2u : 0u) | (braw >= 0 ? 4u : 0u) |
                     (dac_ok ? 16u : 0u);
    char p[160];
    snprintf(p, sizeof(p), "TEL,%lu,%lu,%u,%lu,%lu,%d,%d,%u,%ld,%ld,%d,%d,%lu,%lu", (unsigned long)fw_tel_seq++,
             (unsigned long)now, out.state, (unsigned long)out.fault, (unsigned long)flags, out.en ? 1 : 0,
             out.brk ? 1 : 0, out.dac_mv, -1L, -1L, sraw, braw,
             (unsigned long)logic.ack_seq(), (unsigned long)logic.link_age_ms(now));
    send_frame(p);
  }
  if (now - last_hello >= HELLO_PERIOD_MS) {
    last_hello = now;
    char p[48];
    snprintf(p, sizeof(p), "HELLO,%s,%d", FW_VERSION, PROTO_VERSION);
    send_frame(p);
  }
}
