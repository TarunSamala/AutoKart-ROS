AUTOKART HARDWARE BENCH
=======================

Purpose
-------

This directory contains the active MicroPython firmware arranged by physical
subsystem. Each deployable device directory uses main.py because MicroPython
runs that filename automatically after boot.

Directory map and active versions
---------------------------------

estop/receiver/       LoRa E-stop receiver, version 1.1.0.
estop/transmitter/    LoRa E-stop transmitter, version 1.1.0.
estop/diagnostics/    Radio-only diagnostic scripts, version 1.0.0.
bldc/                 BLD-750 digital and GP8630N bench control, version 1.1.0.
throttle/             GP8630N analog-command-only test, version 1.1.0.
brake/                Standalone TB6600 brake test, version 1.0.0.
steering/             Standalone 3:1 TB6600 steering test, version 1.1.0.
full_setup/           Integrated controller, version 2.0.0.
archive/              Every original/superseded file, preserved and renamed.
validate_bench.py     Host-side syntax, naming, documentation, and pin checks.

Required workflow
-----------------

1. Read the selected folder's README.txt and PINOUT.txt.
2. Disconnect traction power and lift the driven wheels.
3. Verify the board model, voltage levels, grounds, driver polarity, and pins.
4. Run: python3 bench/validate_bench.py
5. Copy only the selected device's main.py to that controller.
6. Test E-stop and watchdog behavior before enabling motion.

Safety
------

These programs are prototype bench firmware. A software E-stop can fail if
the MCU, radio, wiring, power, I2C bus, or application fails. Use a hardwired,
normally-safe E-stop that independently removes traction energy on a vehicle.
