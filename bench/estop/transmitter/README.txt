E-STOP TRANSMITTER
==================

File: main.py
Firmware: autokart-lora-estop-transmitter
Version: 1.1.0
Target: Raspberry Pi Pico running MicroPython

The transmitter sends a versioned CRC-protected heartbeat every 100 ms. Button
press sends an immediate STOP burst; release must remain stable before RUN is
sent. A session value changes after reboot so receivers can reject stale data.

The verified switch logic is LOW when released and HIGH when pressed. Confirm
that physical behavior with a meter before relying on the firmware.
