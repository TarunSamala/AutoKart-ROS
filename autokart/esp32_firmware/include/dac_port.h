// Throttle DAC port (GP8630N over I2C). There is no read-back from the GP8630N: a successful I2C ACK
// on every step is the only health signal this module has, which is why dac_init()/dac_write_mv() must
// return false (and the caller must treat the DAC as not-ok) on any failed transaction.
#pragma once
#include <stdint.h>

bool dac_init();                 // true when the DAC answers on the bus, is set to the 0-10 V range and 0 V
bool dac_write_mv(uint16_t mv);  // true when the write was acknowledged. mv is clamped to 0..THROTTLE_MAX_MV.
