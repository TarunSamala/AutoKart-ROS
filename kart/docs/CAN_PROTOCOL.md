# CAN protocol
`MotionCanMessage` ID `0x180` uses little-endian sequence, speed [cm/s], steering [mrad], brake [permille], validity [ms], and an 8-bit additive integrity byte. It is a simulation protocol, not a production safety protocol; decoder rejects bad layout/CRC. CAN commands remain high-level and must reach VCU validation before plant outputs.
