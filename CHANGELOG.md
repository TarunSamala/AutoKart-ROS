# Changelog

All notable engineering changes to this repository. Format loosely follows
[Keep a Changelog](https://keepachangelog.com/); dates are UTC. This file summarizes milestones --
`docs/HANDOVER_STATUS.md` is the detailed, append-only engineering log and is authoritative for
exact findings, measurements, and file lists.

**No entry below authorizes vehicle motion.** `calibration_verified`, `vehicle_config_verified` and
`braking_verified` are set only by the operator, after reviewing physical evidence, and remain
`false` as of every entry in this file.

## [Unreleased]

### Added
- `tools/venue_capture.sh` + `tools/venue_capture_frames.py`: guided venue recording session saving
  stamp-matched RGB+depth pairs and a distance sheet for later calibration/model review.
- `tools/controlled_run.py`: direct-VCU controlled run demo (ramp to the 2.0 V cap, staged hold,
  the mission's stop sequence), tested only against a simulated serial link.
- ARM and RESET FAULT buttons on both Mission Control dashboards (`/ops` and the separate AutoKart
  Vehicle Dashboard app), forwarding the two already-whitelisted gateway request types that had no
  frontend control yet.
- A dedicated HP60C capture thread in the live perception ROS node, decoupling RGB/depth image
  delivery from ONNX inference so the camera's own capture rate is no longer gated behind it.
- `VERSION`, this `CHANGELOG.md`, `scripts/run_all_tests.sh` (a single non-interactive entry point
  for the suites `scripts/test_session.sh` runs interactively), a `/health` endpoint on the
  dashboard gateway.

### Changed
- `scripts/start_phase1.sh`: starting an already-running graph is now a safe no-op (exit 0, not an
  error), and the previous run's `ros.log`/`ui.log` are rotated with a timestamp instead of being
  silently overwritten (last 10 kept per name).
- `scripts/first_run.sh`: now also requires `tools/readiness_report.py` to report
  `READY FOR FIRST RUN`, in addition to the existing `scripts/system_check.sh` GO gate.

### Known limitations carried forward (see `docs/HANDOVER_STATUS.md` for detail)
- Camera pitch (17.8° operator-measured vs. 11.4° ground-plane estimate) is unresolved.
- No measured braking deceleration, encoder scale, or end-to-end latency exists yet for this
  no-encoder hardware revision.
- `calibration_verified` / `vehicle_config_verified` / `braking_verified` are all `false`.
- Live after-fix camera paired RGB+depth rate is not yet re-measured (camera was off the USB bus
  at the time of the fix); best isolated capture ceiling measured so far is ~7.2 Hz, below the 9 Hz
  target.

## [0.1.0-pre-commissioning] - earlier sessions through 2026-10-19
Everything prior to the Unreleased section above: the phase-1 ROS graph (perception, mapping,
planning, controller, vehicle bridge, VCU safety supervisor, dashboard gateway), the ESP32-S3 VCU
firmware (arming-locked, no axle encoder, no E-stop input wire), the YOLO26s detector integration,
the shared vehicle-geometry package, and the full qualification tool suite
(`tools/depth_check.py`, `tools/encoder_calibration.py`, `tools/latency_report.py`,
`tools/braking_test.py`, `tools/readiness_report.py`, and others). See `docs/HANDOVER_STATUS.md`
for the complete, dated history -- it predates this file and is not repeated here.
