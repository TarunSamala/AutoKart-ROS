// GP8630N throttle DAC driver, ported from a working MicroPython bench test
// (ESP32-S3 + GP8630N + BLD-750, I2C 0x58, 0-10 V output range).
//
// There is no read-back from this chip: an ACKed I2C transaction is the only health signal available,
// so every return path below is false unless every step actually succeeded.
#include "dac_port.h"
#include <Arduino.h>
#include <Wire.h>
#include "config.h"

static bool i2c_write(uint8_t reg, const uint8_t* data, size_t len) {
  Wire.beginTransmission(GP8630N_ADDR);
  Wire.write(reg);
  for (size_t i = 0; i < len; i++) Wire.write(data[i]);
  return Wire.endTransmission() == 0;  // 0 = ACKed by the device
}

bool dac_init() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
  Wire.setTimeOut(I2C_BUS_TIMEOUT_MS);  // abort a wedged bus quickly instead of hanging the loop

  // Confirm the device ACKs on the bus before relying on it for anything.
  Wire.beginTransmission(GP8630N_ADDR);
  if (Wire.endTransmission() != 0) return false;

  // Select the 0-10 V output range.
  uint8_t mode = GP8630N_MODE_0_10V;
  if (!i2c_write(GP8630N_REG_MODE, &mode, 1)) return false;
  delay(200);  // bench-proven settle time after the mode write

  // Start at 0 V. dac_write_mv() does its own clamp + ACK check.
  return dac_write_mv(0);
}

bool dac_write_mv(uint16_t mv) {
  if (mv > THROTTLE_MAX_MV) mv = THROTTLE_MAX_MV;  // second clamp; the caller already enforces this too

  // code = mv / FULL_SCALE_MV * 65535, bench-verified: 700 mV -> 4587, 2000 mV -> 13107.
  uint32_t code = (uint32_t)mv * 65535UL / GP8630N_FULL_SCALE_MV;
  uint8_t payload[2] = { (uint8_t)(code & 0xFF), (uint8_t)((code >> 8) & 0xFF) };  // LOW byte first, then HIGH
  return i2c_write(GP8630N_REG_DAC, payload, sizeof(payload));
}
