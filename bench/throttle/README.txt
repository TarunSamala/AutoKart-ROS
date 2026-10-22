THROTTLE COMMAND BENCH TEST
===========================

File: main.py
Firmware: autokart-throttle-bench-controller
Version: 1.1.0
Target: ESP32-S3 running MicroPython

This Wi-Fi test changes only the GP8630N 0-10 V command sent to BLD-750 SV.
BLD-750 EN and BRK were originally expected to be handled manually. It is not
a complete drive controller and must not be used for unattended motion.

The code and browser UI are both clamped to 0.70 V, below the previously
observed fault near 0.75 V. Test with driven wheels off the ground.
