WIRELESS E-STOP SUBSYSTEM
=========================

receiver/ contains the active ESP32-S3 receiver firmware, version 1.1.0.
transmitter/ contains the active Raspberry Pi Pico transmitter, version 1.1.0.
diagnostics/ contains radio-only link tests; they do not implement E-stop logic.

Protocol
--------

Frequency: 433 MHz
Protocol version: 1
Transmitter ID: 0x31
Heartbeat: 100 ms
Receiver timeout: 350 ms
RUN qualification: 5 valid packets

The production-oriented receiver starts stopped and latches STOP or link loss.
The integrated full_setup controller requires a stable RUN stream before reset
or arm. A reset never arms the kart.

This remains software-controlled prototype equipment, not a safety-rated or
independent vehicle emergency-stop circuit.
