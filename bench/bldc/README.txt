BLDC / BLD-750 BENCH CONTROLLER
===============================

File: main.py
Firmware: autokart-bldc-bench-controller
Version: 1.1.0
Target: ESP32-S3 running MicroPython

Purpose
-------

Web bench controller for BLD-750 EN, BRK, F/R, and GP8630N analog speed
command. It initializes EN stopped and BRK applied before I2C or Wi-Fi setup.
The browser watchdog forces 0 V, brake, and stop if commands disappear.

Current limits
--------------

Maximum SV command is 0.70 V because an earlier bench run reported a BLD-750
fault near 0.75 V. Do not raise the code or UI limit until the fault is found
and a new ceiling is validated with the wheels off the ground.

This controller is for traction bench work only. It does not include the LoRa
E-stop or steering/mechanical brake logic; use full_setup for integration.
