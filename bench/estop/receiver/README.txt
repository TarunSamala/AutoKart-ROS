E-STOP RECEIVER
===============

File: main.py
Firmware: autokart-lora-estop-receiver
Version: 1.1.0
Target: ESP32-S3 running MicroPython

The receiver validates protocol magic, version, transmitter ID, CRC, session,
and sequence. It requires five valid RUN packets, starts stopped, and latches
on a valid STOP command or heartbeat timeout. Debug radio state is printed at
a bounded interval.

Deploy by copying this main.py to the ESP32-S3 only after a radio diagnostic
passes. This standalone receiver demonstrates the link state; final traction
isolation must be hardwired and independently verified.
