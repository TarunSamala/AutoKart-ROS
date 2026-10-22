STEERING BENCH TEST
===================

File: main.py
Firmware: autokart-steering-bench-test
Version: 1.1.0
Target: ESP32-S3 running MicroPython

Standalone terminal test for a TB6600/NEMA23 steering actuator with a 17T
motor gear and 51T steering gear (3:1). The range is conservatively limited to
the last documented validated value of -20 to +20 degrees.

This is open-loop. Physically center the steering before boot. Verify direction
and clearance with a small command before using full-left or full-right.
