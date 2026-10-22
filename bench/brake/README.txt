MECHANICAL BRAKE BENCH TEST
===========================

File: main.py
Firmware: autokart-brake-bench-test
Version: 1.0.0
Target: ESP32-S3 running MicroPython

Standalone terminal test for the brake TB6600 and NEMA23. It uses the latest
documented 0..15 degree travel, 3200 pulses/revolution, 1:1 ratio, active-low
enable, and DIR LOW for APPLY.

This is open-loop. Physically release the brake before boot so the software
zero is meaningful. Use the smallest motion first and verify direction. The
driver is disabled at startup and on program exit.

Commands: apply, release, move <0..15>, status, disable, quit.
