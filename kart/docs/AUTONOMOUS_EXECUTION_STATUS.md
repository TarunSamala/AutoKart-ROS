# Autonomous Execution Status

Date: 2026-07-29

CURRENT PHASE:
Continuous Gazebo follower acceptance implemented and natively validated.

IMPLEMENTED:

- Internal deterministic dynamics remains authoritative.
- Pure Pursuit commands route through VCU, independent safety, actuators and
  the dynamic four-wheel plant.
- Frozen reset-versioned snapshots drive continuous Gazebo pose, measured
  steering and measured wheel/rear-axle rotation.
- `/clock`, TF, joint state, odometry and follower diagnostics have one native
  source in follower mode.
- Native RGB, depth and LiDAR payloads preserve capture time; internal IMU is
  the sole configured IMU authority.
- Pause/resume/reset/disconnect/process-loss and no-double-physics tests exist.
- A complete lap and controlled stop reached final VCU `SAFE`.

EVIDENCE:

- baseline: 125 passed;
- follower suite: 13 passed;
- final full suite after follower implementation: 138 passed in 61.92 s;
- ROS package-local tests: 6 passed;
- native SDF checks: PASS;
- authoritative log hash identical at follower off/20/100/200 Hz;
- native lap: 12.260 s; controlled-stop finish: 16.560 s;
- native maximum clock offset: 1 ms; double-physics violations: 0.

KNOWN LIMITATIONS:

- Real kart measurements/CAD and surveyed track remain unavailable.
- RViz/Gazebo GUI screenshots were not captured.
- Camera info, degraded depth point cloud and native multi-kart Gazebo remain.
- GPU measurement was unavailable; rendering used llvmpipe.
- Chrono, real Jetson/VCU/CAN and HIL hardware were not executed.
- Real-kart readiness remains rejected.

NEXT ACTION:
Implement and natively validate calibrated camera-info plus degraded RGB-D
point-cloud delivery against measured sensor calibration.

## Phase 16 update — 2026-07-30

WORKSTATION REAL-TIME STATUS: `REALTIME_DEGRADED`

- Native Gazebo follower laps completed at aggregate RTF `1.004939` and
  `1.004746`; the final repeat's minimum complete five-second window RTF was
  `1.004636`.
- The exact previous authoritative vehicle-log SHA-256 was preserved.
- The first run had zero VCU and independent-safety misses. The exact repeat
  had two 1 ms VCU wall-deadline misses and zero independent-safety misses, so
  stable qualification is not claimed.
- The lap completed at 12.260 s and the controlled stop reached `SAFE` at
  16.560 s.
- RGB, depth, LiDAR, and IMU payloads were non-empty; timestamp violations and
  double-physics violations were zero; maximum sensor queue depth was one.
- One RGB and one depth sample were explicitly rejected as stale and one
  LiDAR sample was rejected during warm-up. No stale sample was delivered as
  valid, and no drop was hidden in a generic counter.
- Pure Pursuit was natively requalified. Stanley, Follow-the-Gap, rendered
  cone perception, the complete obstacle/localization matrix, and native
  multi-lap thermal operation remain unqualified in Phase 16.
- Jetson, physical HIL, and real-kart operation remain unexecuted and rejected.
- Final Phase 16 repository regression: 145 passed in 54.53 s.

NEXT ACTION:
Execute the bounded 30-minute native Gazebo, ROS, dashboard, and recorder soak
on the target Jetson hardware profile and close its timing and thermal gates.
