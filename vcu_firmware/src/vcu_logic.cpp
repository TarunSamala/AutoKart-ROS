#include "vcu_logic.h"
#include "config.h"

VcuLogic::VcuLogic()
    : state_(VCU_SAFE), fault_(0), have_cmd_(false), last_cmd_ms_(0), last_seq_(0), have_seq_(false),
      throttle_mv_(0), steer_cdeg_(0), brake_pct_(0), last_step_ms_(0), have_step_(false), dac_mv_(0) {}

bool VcuLogic::seq_is_new(uint32_t seq) const {
  if (!have_seq_) return true;
  // 32-bit wraparound-safe "newer than"
  return (int32_t)(seq - last_seq_) > 0;
}

bool VcuLogic::link_ok(uint32_t now_ms) const {
  return have_cmd_ && (uint32_t)(now_ms - last_cmd_ms_) <= CMD_TIMEOUT_MS;
}

void VcuLogic::enter_fault(uint32_t bit) {
  fault_ |= bit;
  state_ = VCU_FAULT;
  dac_mv_ = 0;  // drop throttle immediately
}

bool VcuLogic::on_cmd(uint32_t now_ms, uint32_t seq, long throttle_mv, long steer_cdeg, long brake_pct) {
  if (!seq_is_new(seq)) return false;
  have_seq_ = true;
  last_seq_ = seq;
  have_cmd_ = true;
  last_cmd_ms_ = now_ms;
  throttle_mv_ = throttle_mv;
  steer_cdeg_ = steer_cdeg;
  brake_pct_ = brake_pct;
  // A command above the cap or below zero is a software bug upstream: do not clamp silently.
  if (state_ == VCU_ARMED && (throttle_mv < 0 || throttle_mv > THROTTLE_MAX_MV || brake_pct < 0 || brake_pct > 100)) {
    enter_fault(FAULT_CMD_RANGE);
  }
  return true;
}

int VcuLogic::on_arm(uint32_t seq, const VcuInputs& in) {
  (void)seq;
  if (!VCU_OUTPUT_WIRING_VERIFIED) return NAK_WIRING_UNVERIFIED;
  if (state_ != VCU_SAFE) return NAK_WRONG_STATE;
  if (!in.estop_ok) return NAK_ESTOP;
  if (!in.dac_ok) return NAK_DAC;
  if (!link_ok(in.now_ms)) return NAK_LINK;
  if (throttle_mv_ != 0) return NAK_THROTTLE_NONZERO;
  state_ = VCU_ARMED;
  dac_mv_ = 0;
  return NAK_NONE;
}

void VcuLogic::on_disarm() {
  if (state_ == VCU_ARMED) state_ = VCU_SAFE;  // a latched FAULT stays latched until RST
  dac_mv_ = 0;
}

bool VcuLogic::on_observation(int decision) {
  if (decision < 0 || decision > 2) return false;
  if (decision != 1) on_disarm();
  return true;
}

int VcuLogic::on_reset(const VcuInputs& in) {
  if (state_ != VCU_FAULT) return NAK_WRONG_STATE;
  if (!in.estop_ok) return NAK_CAUSE_PRESENT;
  if (!in.dac_ok) return NAK_CAUSE_PRESENT;
  if (!link_ok(in.now_ms)) return NAK_LINK;
  fault_ = 0;
  state_ = VCU_SAFE;
  dac_mv_ = 0;
  return NAK_NONE;
}

VcuOutputs VcuLogic::step(const VcuInputs& in) {
  uint32_t dt_ms = have_step_ ? (uint32_t)(in.now_ms - last_step_ms_) : 0;
  last_step_ms_ = in.now_ms;
  have_step_ = true;

  if (state_ == VCU_ARMED) {
    if (!in.estop_ok) enter_fault(FAULT_ESTOP);
    else if (!in.dac_ok) enter_fault(FAULT_DAC);
    else if (!link_ok(in.now_ms)) enter_fault(FAULT_LINK_TIMEOUT);
  }

  VcuOutputs o;
  o.state = state_;
  o.fault = fault_;
  if (state_ == VCU_ARMED) {
    bool braking = brake_pct_ >= 50;
    long target = braking ? 0 : throttle_mv_;
    if (target > THROTTLE_MAX_MV) target = THROTTLE_MAX_MV;  // belt and braces; range check above already faults
    if (target < 0) target = 0;
    if ((long)dac_mv_ > target) {
      dac_mv_ = (uint16_t)target;  // decreases are instant
    } else {
      uint32_t max_step = (uint32_t)THROTTLE_SLEW_MV_S * dt_ms / 1000u;
      uint32_t diff = (uint32_t)(target - (long)dac_mv_);
      dac_mv_ = (uint16_t)(dac_mv_ + (diff < max_step ? diff : max_step));
    }
    o.en = true;
    o.brk = braking;
    o.dac_mv = dac_mv_;
  } else if (state_ == VCU_FAULT) {
    dac_mv_ = 0;
    o.en = false;
    o.brk = true;
    o.dac_mv = 0;
  } else {  // SAFE
    dac_mv_ = 0;
    o.en = false;
    o.brk = BRAKE_LINE_IN_SAFE != 0;
    o.dac_mv = 0;
  }
  return o;
}
