// VCU safety/state logic. Pure C++ (no Arduino) so it is unit-tested on the host.
#pragma once
#include <stdint.h>

enum VcuState : uint8_t { VCU_SAFE = 0, VCU_ARMED = 1, VCU_FAULT = 2 };

enum VcuFault : uint32_t {
  FAULT_LINK_TIMEOUT = 1,
  FAULT_ESTOP = 2,
  FAULT_DAC = 4,
  FAULT_CMD_RANGE = 8,
};

enum VcuNak : int {
  NAK_NONE = 0,
  NAK_WRONG_STATE = 1,
  NAK_ESTOP = 2,
  NAK_DAC = 3,
  NAK_LINK = 4,
  NAK_THROTTLE_NONZERO = 5,
  NAK_CAUSE_PRESENT = 6,
  NAK_WIRING_UNVERIFIED = 7,
};

struct VcuInputs {
  uint32_t now_ms;
  bool estop_ok;   // effective state: true when input absent; fitted pin uses filtered HIGH healthy
  bool dac_ok;     // DAC initialised and last write succeeded
};

struct VcuOutputs {
  bool en;            // BLD-750 enable asserted
  bool brk;           // BLD-750 brake asserted
  uint16_t dac_mv;    // throttle voltage to write to the DAC
  uint8_t state;
  uint32_t fault;
};

class VcuLogic {
 public:
  VcuLogic();
  // Returns false if the command was ignored (duplicate / old sequence).
  bool on_cmd(uint32_t now_ms, uint32_t seq, long throttle_mv, long steer_cdeg, long brake_pct);
  int on_arm(uint32_t seq, const VcuInputs& in);   // 0 = accepted, else VcuNak
  void on_disarm();
  // Camera observation transport never arms or requests throttle. STOP or
  // target reached can only disarm; OBSERVE has no actuator effect.
  bool on_observation(int decision);
  int on_reset(const VcuInputs& in);               // 0 = accepted, else VcuNak
  VcuOutputs step(const VcuInputs& in);            // call every loop iteration

  uint8_t state() const { return state_; }
  uint32_t fault() const { return fault_; }
  uint32_t ack_seq() const { return last_seq_; }
  uint32_t link_age_ms(uint32_t now_ms) const { return have_cmd_ ? now_ms - last_cmd_ms_ : 0xFFFFFFFFu; }
  bool link_ok(uint32_t now_ms) const;
  long last_steer_cdeg() const { return steer_cdeg_; }  // parsed, never actuated in this release

 private:
  void enter_fault(uint32_t bit);
  bool seq_is_new(uint32_t seq) const;

  uint8_t state_;
  uint32_t fault_;
  bool have_cmd_;
  uint32_t last_cmd_ms_;
  uint32_t last_seq_;
  bool have_seq_;
  long throttle_mv_;
  long steer_cdeg_;
  long brake_pct_;
  uint32_t last_step_ms_;
  bool have_step_;
  uint16_t dac_mv_;
};
