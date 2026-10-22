#pragma once
#include "config.h"
#include <stdint.h>

// The physical power cut has no ESP32 status wire in the default build.
// Never infer an E-stop state from a floating GPIO7.
inline bool effective_estop_ok(bool filtered_pin_ok) {
#if ESTOP_INPUT_FITTED
  return filtered_pin_ok;
#else
  (void)filtered_pin_ok;
  return true;
#endif
}

class EstopFilter {
 public:
  bool update(bool raw_ok) {
    if (raw_ok) bad_ = 0;
    else if (bad_ < 255) ++bad_;
    return bad_ < 3;
  }
 private:
  uint8_t bad_ = 0;
};

constexpr unsigned VCU_FLAG_ESTOP_OK = 1u;
constexpr unsigned VCU_FLAG_ESTOP_INPUT_FITTED = 1u << 5;

inline unsigned estop_telemetry_flags(bool effective_ok) {
#if ESTOP_INPUT_FITTED
  return VCU_FLAG_ESTOP_INPUT_FITTED | (effective_ok ? VCU_FLAG_ESTOP_OK : 0u);
#else
  (void)effective_ok;
  return 0u;
#endif
}
