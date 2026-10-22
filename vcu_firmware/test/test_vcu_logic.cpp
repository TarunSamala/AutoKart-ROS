// Host-side tests for the VCU safety logic and framing. Build: see tests/run_all.sh
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "protocol.h"
#include "vcu_logic.h"

static int checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); return 1; } } while (0)

static VcuInputs in(uint32_t t, bool estop = true, bool dac = true) { VcuInputs i; i.now_ms = t; i.estop_ok = estop; i.dac_ok = dac; return i; }

int main(int argc, char** argv) {
  // --- CRC / framing ---
  if (argc > 1 && !strcmp(argv[1], "crc")) {  // used by the python cross-check
    for (int i = 2; i < argc; i++) printf("%04X\n", crc16_ccitt((const uint8_t*)argv[i], strlen(argv[i])));
    return 0;
  }
  CHECK(crc16_ccitt((const uint8_t*)"123456789", 9) == 0x29B1);  // standard CCITT-FALSE check value
  char frame[120], payload[100];
  CHECK(frame_build("CMD,1,500,0,0", frame, sizeof(frame)) > 0);
  CHECK(frame_parse(frame, payload, sizeof(payload)) == 13);
  CHECK(strcmp(payload, "CMD,1,500,0,0") == 0);
  frame[3] ^= 1;  // corrupt
  CHECK(frame_parse(frame, payload, sizeof(payload)) == -1);
  CHECK(frame_parse("garbage", payload, sizeof(payload)) == -1);
  char p2[] = "A,1,2";
  char* f[4];
  CHECK(split_fields(p2, f, 4) == 3 && !strcmp(f[2], "2"));
  long v;
  CHECK(parse_i32("-12", &v) && v == -12);
  CHECK(!parse_i32("1x", &v) && !parse_i32("", &v) && !parse_i32("-", &v));

  // --- boot state ---
  VcuLogic L;
  VcuOutputs o = L.step(in(0));
  CHECK(o.state == VCU_SAFE && !o.en && o.dac_mv == 0);
  CHECK(o.brk == (BRAKE_LINE_IN_SAFE != 0));

  // --- arm refusals ---
  CHECK(L.on_arm(1, in(10)) == NAK_LINK);                    // no command link yet
  CHECK(L.on_cmd(100, 1, 0, 0, 0));
  CHECK(L.on_arm(2, in(110, false)) == NAK_ESTOP);
  CHECK(L.on_arm(2, in(110, true, false)) == NAK_DAC);
  CHECK(L.on_arm(2, in(500)) == NAK_LINK);                   // 400 ms since last cmd
  CHECK(L.on_cmd(600, 2, 300, 0, 0));
  CHECK(L.on_arm(3, in(610)) == NAK_THROTTLE_NONZERO);
  CHECK(L.on_cmd(620, 3, 0, 0, 0));
  CHECK(L.on_arm(4, in(630)) == NAK_NONE);
  CHECK(L.state() == VCU_ARMED);
  CHECK(!L.on_observation(3));
  CHECK(L.state() == VCU_ARMED);
  CHECK(L.on_observation(1));
  CHECK(L.state() == VCU_ARMED);

  // --- throttle ramp, cap, instant drop ---
  o = L.step(in(640));
  CHECK(o.state == VCU_ARMED && o.en && o.dac_mv == 0);
  CHECK(L.on_cmd(650, 4, THROTTLE_MAX_MV, 0, 0));
  uint32_t t = 650;
  uint16_t prev = 0;
  for (int i = 0; i < 30; i++) { t += 20; L.on_cmd(t, 5 + i, THROTTLE_MAX_MV, 0, 0); o = L.step(in(t)); CHECK(o.dac_mv >= prev && o.dac_mv <= THROTTLE_MAX_MV); prev = o.dac_mv; }
  CHECK(prev > 0 && prev <= THROTTLE_MAX_MV);
  CHECK(L.state() == VCU_ARMED);
  for (int i = 0; i < 100; i++) { t += 20; L.on_cmd(t, 100 + i, THROTTLE_MAX_MV, 0, 0); o = L.step(in(t)); }
  CHECK(o.dac_mv == THROTTLE_MAX_MV);                        // reaches, never exceeds, the commissioning cap
  L.on_cmd(t + 10, 300, 0, 0, 0); o = L.step(in(t + 10));
  CHECK(o.dac_mv == 0);                                      // drops instantly
  L.on_cmd(t + 20, 301, THROTTLE_MAX_MV / 2, 0, 100); o = L.step(in(t + 20));
  CHECK(o.brk && o.dac_mv == 0 && o.en);                     // braking forces throttle to zero
  CHECK(L.on_observation(2));
  o = L.step(in(t + 21));
  CHECK(o.state == VCU_SAFE && !o.en && o.brk && o.dac_mv == 0);
  CHECK(L.on_cmd(t + 22, 302, 0, 0, 100));
  CHECK(L.on_arm(299, in(t + 22)) == NAK_NONE);
  CHECK(L.last_steer_cdeg() == 0);

  // --- over-cap command faults ---
  L.on_cmd(t + 30, 303, THROTTLE_MAX_MV + 1, 0, 0); o = L.step(in(t + 30));
  CHECK(o.state == VCU_FAULT && (o.fault & FAULT_CMD_RANGE) && !o.en && o.brk && o.dac_mv == 0);
  CHECK(L.on_reset(in(t + 40)) == NAK_NONE && L.state() == VCU_SAFE && L.fault() == 0);

  // --- stale/duplicate seq ignored ---
  CHECK(!L.on_cmd(t + 50, 303, 0, 0, 0));
  CHECK(L.on_cmd(t + 50, 400, 0, 0, 0));

  // --- link timeout while armed ---
  CHECK(L.on_arm(401, in(t + 60)) == NAK_NONE);
  L.on_cmd(t + 70, 401, THROTTLE_MAX_MV - 100, 0, 0); L.step(in(t + 70));
  o = L.step(in(t + 70 + CMD_TIMEOUT_MS + 5));
  CHECK(o.state == VCU_FAULT && (o.fault & FAULT_LINK_TIMEOUT) && o.dac_mv == 0 && !o.en && o.brk);
  CHECK(L.on_reset(in(t + 500)) == NAK_LINK);                // still no fresh link
  L.on_cmd(t + 510, 402, 0, 0, 0);
  CHECK(L.on_reset(in(t + 520, false)) == NAK_CAUSE_PRESENT);
  CHECK(L.on_reset(in(t + 520)) == NAK_NONE);

  // --- e-stop while armed ---
  L.on_cmd(t + 530, 403, 0, 0, 0);
  CHECK(L.on_arm(404, in(t + 540)) == NAK_NONE);
  o = L.step(in(t + 545, false));
  CHECK(o.state == VCU_FAULT && (o.fault & FAULT_ESTOP) && !o.en);

  // --- disarm ---
  L.on_cmd(t + 560, 405, 0, 0, 0);
  CHECK(L.on_reset(in(t + 565)) == NAK_NONE);
  CHECK(L.on_arm(406, in(t + 566)) == NAK_NONE);
  L.on_disarm();
  o = L.step(in(t + 570));
  CHECK(o.state == VCU_SAFE && !o.en && o.dac_mv == 0);

  // --- DAC failure while armed ---
  L.on_cmd(t + 575, 407, 0, 0, 0);
  CHECK(L.on_arm(408, in(t + 576)) == NAK_NONE);
  o = L.step(in(t + 580, true, false));
  CHECK(o.state == VCU_FAULT && (o.fault & FAULT_DAC));

  printf("OK: %d checks passed\n", checks);
  return 0;
}
