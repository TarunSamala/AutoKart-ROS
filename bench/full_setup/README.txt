INTEGRATED FULL BENCH CONTROLLER
================================

File: main.py
Firmware: autokart-full-bench-controller
Version: 2.0.0
Target: ESP32-S3 running MicroPython

Included functions
------------------

- TB6600/NEMA23 steering and mechanical brake control.
- GP8630N analog command and BLD-750 EN/BRK/F-R outputs.
- RA-02/SX1278 wireless E-stop receiver.
- Wi-Fi RC interface with one-controller lease and heartbeat watchdog.
- Safe-first BLD output initialization and bounded non-blocking HTTP service.

Version 2.0.0 safety behavior
-----------------------------

- Steering is clamped to the last documented validated +/-20 degree range.
- Throttle is clamped to 0.70 V, below the prior reported ~0.75 V fault.
- Arming and E-stop reset require five current valid LoRa RUN packets.
- Remote STOP, LoRa heartbeat timeout, receiver exception, transmitter restart
  while armed, browser heartbeat loss, and GP8630N failure latch a safe stop.
- E-stop reset does not re-arm; ARM is a separate action.
- Stop order is BLD EN stop, BLD BRK apply, DAC zero, mechanical brake command,
  and steering hold. Digital outputs do not depend on a working I2C bus.

Known physical limitations
--------------------------

Steering and mechanical-brake position remain open-loop and require correct
physical zero at boot. The code is prototype firmware, not an independently
certified emergency-stop system. Use a hardwired normally-safe traction cut.
