# AutoKart one-cone production integration — safety-first software starter

This is a **non-actuating** laptop preflight and independent stop-envelope
prototype. It is NOT the production vehicle-control firmware. It is intended
to be copied into the existing AutoKart repo, **without overwriting** existing
camera SDK ownership, YOLO26s integration, or existing ESP32 production firmware.

## Run now — traction power disconnected

```sh
cd ~/Desktop/autonomous_kart
python3 ak_preflight.py
python3 -m unittest discover -s tests -p 'test_one_cone_logic.py' -v
python3 dry_run.py --jsonl examples/bench_simulation.jsonl
```

## Sensor input contract for next integration

The existing HP60C SDK owner and ROS perception node must supply:
- calibrated bumper-to-cone-surface longitudinal gap in metres
- wheel speed from axle encoder (metres/second), validity and fresh timestamp
- source=LIVE only for genuinely measured camera frames (never synthetic)
- calibration_status=MOUNT_VERIFIED only with a verified signed-off calibration
- depth_valid and observation_age_s
- independent emergency-stop-clear safety signal

The decision engine outputs `APPROACH_CANDIDATE` (not a drive command!) or
`STOP_REQUEST`. It latches STOP until a separately supervised reset.

The margin, latency, deceleration and max speed in the code are **example
placeholder parameters**, not vehicle acceptance values. They must be replaced
with measured worst-case data. The camera minimum measurable distance and
camera-to-bumper offset must be measured; it cannot guarantee 15cm by itself.

## Non-negotiable release gates

- Do not flash replacement firmware until the existing project is backed up,
  controller pin logic and startup states inspected, and traction isolated.
- Only a tested independent brake/E-stop can guarantee a physical stop.
- To activate propulsion later: add an independently reviewed, initially
  disabled ESP32 VCU module with verified enable/brake logic, encoder feedback,
  command CRC/counter/timeouts, limit checks, and a separate safety authority.
- Test with wheels off ground, then low-speed with barriers / remote stop.
- Do not connect `APPROACH_CANDIDATE` directly to a motor driver.

## Device IDs seen from user `lsusb`

- HP60C 3482:6723
- ESP32 USB Serial JTAG 303a:1001

## What this DOES NOT yet do

- Does not implement HP60C SDK access / RGB-depth alignment.
- Does not load the project's YOLO26s model or command the GP8630N DAC.
- Does not use actual measured braking performance.
- Does not certify 15cm stopping tolerance or real-world safety.

## Optional ESP32 read-only firmware log

```sh
python3 -m pip install pyserial
python3 ak_serial_readonly.py --port /dev/ttyACM0 --seconds 5
```

Keep traction electrically isolated. Some devices reset when USB serial opens;
this script does not transmit commands or overwrite firmware. If the current
firmware prints nothing unless polled, zero bytes is not a link failure.
