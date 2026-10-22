#include <assert.h>
#include "config.h"
#include "vcu_logic.h"

int main() {
  static_assert(VCU_OUTPUT_WIRING_VERIFIED == 0, "default production build must stay locked");
  VcuLogic logic;
  VcuInputs inputs = {100, true, true};
  assert(logic.on_cmd(100, 1, 0, 0, 100));
  assert(logic.on_arm(2, inputs) == NAK_WIRING_UNVERIFIED);
  assert(logic.on_observation(1));
  assert(logic.on_arm(3, inputs) == NAK_WIRING_UNVERIFIED);
  assert(logic.on_observation(0));
  VcuOutputs outputs = logic.step(inputs);
  assert(outputs.state == VCU_SAFE && !outputs.en && outputs.brk && outputs.dac_mv == 0);
  return 0;
}
