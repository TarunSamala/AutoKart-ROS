# AutoKart CORE and LAB software gate — 2026-10-13

## Executed results

- `./autokart_apps/run_tests.sh`: **49 Python passed**, **23 browser-engine tests passed**, TypeScript and Vite production build passed. The build emitted a bundle-size advisory; there was no build failure.
- `./autokart_apps/run_core.sh`: the deterministic sample produced 3 gates, a valid diagnostic path and controller candidate, with `vehicle_authorized: false`.
- The LAB API was started on loopback and `/api/core/status` and `/api/core/step` returned HTTP 200. The browser at `/shared` displayed the graph, gates, centerline, speed, steering, and disabled authority. Start, pause, reset, and single frame were exercised in the browser. Dragging one cone changed the live gate count from 4 to 3; an automated test also verifies immediate Python recomputation after an edit.
- `python3 autokart_apps/verify_shared_http.py` passed against the running LAB service. It resets LAB state and gives identical straight and skidpad samples to independent CORE CLI and LAB HTTP processes. Delaunay edges, Gabriel edges, boundaries, gates, centerline, observed skidpad topology, planning, mission state, and controller candidate were exactly equal. Straight: 3 corridor gates, 0.000000 rad steering. Skidpad: two observed loops, valid diagnostic arc, 0.175111 rad steering.
- The first version of that HTTP check found a mismatch on its second case because the stateful LAB adapter had retained the straight sample. The check now resets LAB before each case; the final run passed both. Final test failures: 0.
- A simulated `Frame` passed through existing `Pipeline` and then `AlgorithmCore`, yielding cones, graph, gates, centerline, and a diagnostic controller candidate. Its cone observations were injected synthetic data, not detected from the drawn RGB pixels. Recorded replay passed through `Replay` and `Pipeline` with invalid pose correctly withholding the controller candidate.
- Tests cover missing/stale pose, repeated timestamp, invalid cone coordinate, blocked path, unknown braking capability, bounded cone expiry, skidpad incomplete/single-color support, winding oscillation, and four Pure Pursuit wheelbase/lateral-offset cases. All leave vehicle authority false.

## Pure Pursuit correction

The prior formula omitted wheelbase: `atan2(2*y_local, distance²)`. The corrected kinematic bicycle relation is `atan2(2*wheelbase_m*y_local, distance²)`, equivalent to `atan(2 L sin(alpha) / L_d)`. The controller validates positive finite wheelbase, and simulator callers pass the configured kart wheelbase. The original failing test reproduced a constructor error before the fix; it and the expanded steering cases now pass.

## Status and next gate

| Area | Status | Evidence or blocker |
|---|---|---|
| Shared geometry and diagnostic control | Software tested | Direct/adapter tests, CLI/API equality, 49 Python tests |
| LAB browser | Software tested | HTTP 200, visual view, controls, build |
| CORE Pi compatibility | **NOT TESTED on Pi** | This run used x86_64, Python 3.10.12; ARM64 SDK/dependency installation needs target check |
| Live HP60C perception | **NOT TESTED here** | No target camera run in this milestone; existing default model is orange only |
| ROS 2 and VCU | **BLOCKED** | Typed read-only ROS publication and feedback adapter not connected; no command transport added |
| Vehicle safety | **NO-GO** | Independent E-stop, physical braking, watchdog and traction isolation are unverified |

The next engineering gate is target Pi deployment and measured HP60C/odometry performance. Additional LAB scenarios, recorded RGB-D browsing, richer stage timing, ROS read-only publication, and systemd integration remain unimplemented. Do not treat a valid diagnostic candidate as actuation approval.
