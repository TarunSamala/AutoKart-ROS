LORA RADIO DIAGNOSTICS
======================

receiver_radio_test.py     ESP32-S3 string receiver, version 1.0.0.
transmitter_radio_test.py  Raspberry Pi Pico string transmitter, version 1.0.0.

These scripts prove SPI wiring, SX1278 detection, RF frequency, and basic packet
transfer. They intentionally omit the AutoKart CRC, heartbeat, sequence, STOP,
RUN, and latching behavior. Never use either diagnostic as an E-stop.
