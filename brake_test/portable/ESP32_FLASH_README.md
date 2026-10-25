# Matching ESP32-S3 firmware

These two files are the matching 0.40 V blue-cone bench firmware:

- `main.py`
- `bldc_bench_config.py`

Flash them once before using the portable dashboard:

```bash
PORT=/dev/ttyACM0
python3 -m mpremote connect "$PORT" fs cp bldc_bench_config.py :bldc_bench_config.py
python3 -m mpremote connect "$PORT" fs cp main.py :main.py
python3 -m mpremote connect "$PORT" soft-reset
```

Connect the ESP32 by USB after flashing. The dashboard automatically probes
`/dev/serial/by-id/*`, `/dev/ttyACM*`, then `/dev/ttyUSB*` and enables START
only after serial telemetry reports `gp_online`, `encoder_ready`, and
`wheel_locked` as true. Keep normal internet Wi-Fi connected.

This is the dedicated wheels-off no-heartbeat profile. It is not production or
ground-driving firmware.
