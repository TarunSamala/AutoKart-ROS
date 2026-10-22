#include <assert.h>
#include <stdio.h>
#include "estop_input.h"
#include "vcu_logic.h"

int main() {
  VcuLogic logic;
  VcuInputs in{0, effective_estop_ok(false), true};
  assert(logic.on_cmd(0, 1, 0, 0, 0));
  in.now_ms = 10;
#if ESTOP_INPUT_FITTED
  assert(estop_telemetry_flags(true) == 33u);
  assert(estop_telemetry_flags(false) == 32u);
  EstopFilter filter;
  assert(filter.update(false));
  assert(filter.update(false));
  assert(!filter.update(false));
  assert(filter.update(true));
  assert(logic.on_arm(2, in) == NAK_ESTOP);
  in.estop_ok = effective_estop_ok(true);
  assert(logic.on_arm(2, in) == NAK_NONE);
  in.estop_ok = effective_estop_ok(false);
  assert(logic.step(in).fault & FAULT_ESTOP);
#else
  assert(estop_telemetry_flags(true) == 0u);
  assert(effective_estop_ok(false));
  assert(logic.on_arm(2, in) == NAK_NONE);
  assert(!(logic.step(in).fault & FAULT_ESTOP));
#endif
  puts("estop mode PASS");
}
