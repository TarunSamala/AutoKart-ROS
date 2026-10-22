# AutoKart Safety Envelope Monitor (Agent 5) — Canonical Reference

> **CORRECTION (2026-10-18, later):** statements in this document that no HP60C is attached are WRONG or STALE. An HP60C (USB `3482:6723`) is attached and streams RGB+depth at ~11-13 fps via the vendor SDK; the live Vehicle Dashboard was verified against real frames (see `ui_split/VALIDATION_MATRIX.md`). Items below marked NOT TESTED because "no camera" are now testable; those not re-run in `ui_split/VALIDATION_MATRIX.md` remain NOT TESTED. Calibration is still UNVERIFIED and no qualified metric geometry exists.

This consolidates work already done and verified in this session (RC-05/RC-06) and its later audit addendum. It does not change behavior; it is the requested canonical doc pointing at the real implementation and its evidence.

## What exists

Package `kart/ros2_ws/src/autokart_safety_bridge`, node `front_safety_envelope_monitor` (`autokart_safety_bridge_ros/node.py`, classification logic in `envelope.py`).

- **Non-negotiable contract, enforced structurally, not just by convention:** the node has no publisher, service client, or action client to anything VCU/brake/throttle/steering — confirmed via `ros2 node info` (Publishers: only `/safety/envelope`, `/rosout`, `/parameter_events`). `vehicle_authority` must be `"OFF"` and `safety_monitor_mode` must be `"DIAGNOSTIC_ONLY"` or the node refuses to start (`RuntimeError` / `rclpy.exceptions.InvalidParameterValueException`), and an `on_set_parameters` callback rejects any later runtime attempt (`ros2 param set`) to change either value — verified live, not just unit-tested.
- **States:** `CLEAR` / `BRAKING` / `CRITICAL` / `UNKNOWN`, published in `SafetyEnvelope.msg`'s `limiting_reason` field as `"<STATE>:<reason>[:CALIBRATION_UNVERIFIED]"`.
  - `CLEAR` requires both odometry and the obstacle feed to be fresh (within `max_odometry_age_s`/`max_obstacle_age_s`) **and** no obstacle inside the monitored corridor — never just "no detections," since a stale feed with zero detections is `UNKNOWN`, not `CLEAR`.
  - `BRAKING`: nearest obstacle clearance inside the speed-dependent trigger distance `D = 1.0 + v·T + v²/(2a) + M` (provisional `T`/`a`/`M`; never a measured braking test).
  - `CRITICAL`: nearest obstacle inside the fixed `minimum_bumper_clearance_m` (1.0 m).
  - `UNKNOWN`: any missing/stale input. Verified live by killing the publisher process mid-run — the gateway's independent freshness channel went `CURRENT→STALE` within ~1s and stayed `STALE`, never reverting to a fabricated `CLEAR`.
- **Provenance (`data_source` field, added in the post-RC-06 audit):** `LIVE`/`SIMULATED`/`REPLAY`/`UNVERIFIED`. Required as an explicit node parameter; an unset or unrecognized value publishes `UNVERIFIED`, **never defaults to `LIVE`.** This closes the finding that a live DDS transport is not proof of a physical sensor.
- **Geometry inputs:** `bumper_setback_m` (1.0668 m), `minimum_bumper_clearance_m` (1.0 m), `corridor_half_width_m` (0.508 m) — all explicitly labeled PROVISIONAL pending a physical `base_link` origin survey (see `AUTOKART_CAMERA_GEOMETRY.md`).

## Evidence trail

- Classification logic: 13 pure-Python unit tests (`test_envelope.py`).
- Node behavior (parameter rejection, data_source handling) against the real `rclpy` runtime: 10 tests (`test_node_contract.py`).
- Live DDS verification (publish rate, message content, state-transition timeline, topic dropout, runtime parameter rejection): `docs/RC06_FINAL_ACCEPTANCE_REPORT.md` and its addendum, with raw evidence in `docs/RC06_evidence_2026-10-18/`.
- Gateway/dashboard integration (how `/safety/envelope` reaches the browser, provenance display, staleness independent of odometry): `docs/RC06_FINAL_ACCEPTANCE_REPORT.md` §3.4–3.7, tests in `kart/ros2_ws/src/autokart_dashboard_gateway/test/`.

## What this document does not claim

- No real obstacle has ever been measured against this monitor (no HP60C in this environment). Every number above comes from a synthetic ROS publisher standing in for the sensor, clearly logged as such.
- Corridor geometry currently accounts for a fixed half-width, not steering/path curvature (TASK E3's curvature consideration is not implemented — the kart's predicted swept path isn't modeled, only a straight-ahead corridor).
- Obstacle footprint/size is not modeled; only a single `(x, y)` point per obstacle is consumed.

See `AUTOKART_OPEN_ISSUES.md` for the consolidated list across every subsystem.
