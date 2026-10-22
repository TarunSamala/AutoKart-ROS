# Phase 16 Real-Time SIL Report

PHASE:
Phase 16 — Real-Time SIL Qualification and Autonomous Racing Robustness

STATUS:
PARTIAL

VERDICT:
PROTOTYPE_TEST_REQUIRED

The preset demonstrates sustained real-time throughput, but the exact
post-regression repeat recorded two VCU deadline misses after an earlier
zero-miss run. Stable SIL timing is therefore not approved. The unexecuted
native robustness matrix, full-stack 30-minute soak, Jetson qualification,
HIL, and physical-kart scope are also not closed.

TEST PLATFORM:

- CPU: Intel Core i7-9750H, 6 cores / 12 threads
- GPU: Intel UHD 630 through Mesa Iris; GTX 1650 Mobile present through
  nouveau, but utilization measurement unavailable
- RAM: 15.34 GiB
- OS: Ubuntu 22.04.5, Linux 6.8.0-136-generic, PREEMPT_DYNAMIC
- Rendering backend: Gazebo Classic 11 headless, Mesa Iris; `nvidia-smi`
  unavailable

BASELINE:

- Previous RTF: reported 0.197; reproduced native baseline 0.181988
- Previous regression count: 138 passed

IMPLEMENTED:

- Performance profiler: host, process, authoritative core, ROS executor,
  Gazebo, sensor, serialization, queue, disk, timing-window, and artifact
  measurements
- Timing budgets: bounded mean, p95, p99, maximum, deadline-miss,
  consecutive-miss, and jitter reporting for physics, VCU, and independent
  safety
- Real-time preset: `configs/presets/gazebo_realtime_sil.yaml`
- Balanced preset: `configs/presets/gazebo_engineering_balanced.yaml`
- Offline preset: `configs/presets/gazebo_high_fidelity_offline.yaml`
- Sensor presets: explicit RGB, depth, LiDAR, and IMU resolution/rate/queue
  configurations for all three fidelity levels
- ROS optimization: authoritative core process isolation, one-way bounded
  latest-state queue, single-threaded delivery executor, bounded evidence
  hashing, and optional profile-specific CPU assignment
- Queue policy: bounded producer and sensor queues with warm-up, intentional
  decimation, overflow, processing deadline, transport, and stale-rejection
  counters
- Timing diagnostics: scheduler RTF windows, clock drift, task deadlines,
  follower state age, queue depth, and stale-valid rejection gate
- Dashboard performance view: RTF, task latency, follower/sensor latency,
  resource status, queue depths, categorized drops, and qualification class
- Soak test: 30-minute headless authoritative core durability run completed
- Jetson deployment preparation: environment check, deployment, and profiling
  scripts created; deployment is not automatic

REAL-TIME FULL-LAP RESULT:

- Lap completed: yes
- Lap time: 12.260 s simulation time
- Wall time: 16.482 s for 16.560 s synchronized simulation
- Sustained RTF: 1.004746
- Minimum RTF: 1.004636 over complete five-second windows
- Maximum physics execution: 2.325 ms; 1,037 warning-level 1 ms budget overruns,
  with no physics cycle skipped
- Maximum VCU execution: 1.256 ms
- Maximum safety-task execution: 0.265 ms
- Deadline misses: VCU 2; independent safety 0; physics warnings 1,037
- Final state: `SAFE` after controlled stop at 16.560 s

The authoritative vehicle-log SHA-256 remained
`28164b960133afe84a2b8fe6d192a3d41b9174b68f495e11bb7ac137b8b349a2`.
An earlier artifact (`realtime_qualified`) reached RTF 1.004939 with zero
VCU/safety misses. The differing wall-deadline result makes repeatability the
open issue; the deadline was not relaxed.

SENSOR RESULTS:

- RGB: 166 captured, 165 delivered, one stale rejection, zero empty payloads
- Depth: 166 captured, 165 delivered, one stale rejection, zero empty
  payloads
- LiDAR: 165 captured, 165 delivered, one warm-up rejection, zero empty
  payloads
- IMU: 3,310 captured and delivered, zero drops, zero empty payloads
- Drops by reason: warm-up 1; intentional decimation 0; queue overflow 0;
  processing deadline 0; transport 0; stale rejection 2
- Maximum queue depth: 1
- Timestamp violations: 0
- Stale valid deliveries: 0
- Maximum latency: depth 20 ms; RGB 10 ms; LiDAR 10 ms; IMU 3 ms

AUTONOMY RESULTS:

- Pure Pursuit: native real-time full lap and controlled stop passed
- Stanley: implementation and regression contracts preserved; native
  comparative qualification not executed
- Follow-the-gap: implementation and regression contracts preserved; native
  comparative qualification not executed
- Cone track: ground-truth-debug foundations preserved; rendered-sensor
  detection/classification lap not executed
- Wet track: internal physics/fault and convergence cases passed; native
  rendered real-time lap not executed
- Obstacle: existing scenario contracts preserved; complete dynamic-obstacle
  real-time matrix not executed
- Localization degradation: interfaces and baseline tests preserved; native
  loss/recovery qualification not executed

FAULTS UNDER LOAD:

- Command loss: PASS, detected at 0.205 s, propulsion removed, final `SAFE`
- VCU freeze: PASS, detected at 0.148 s, propulsion removed, final
  `EMERGENCY_STOP`
- E-stop: PASS, detected at 0.001 s, propulsion removed, latched lockout
- Steering jam: PASS, detected at 0.101 s, propulsion removed, controlled stop
- Brake failure: PASS, detected at 0.241 s, propulsion removed, latched lockout
- Stuck torque: PASS, detected at 0.305 s, propulsion removed, latched lockout
- Compute overload: PASS for a bounded 0.4 ms callback load, zero VCU and
  independent-safety deadline misses

These seven cases used the headless authoritative plant under bounded CPU
load. Repetition with native Gazebo and the full sensor stack is
`NOT_EXECUTED`.

SOAK TEST:

- Duration: 1,800.02 s simulated, 354.05 s wall
- RTF stability: aggregate 5.084 in deterministic headless execution
- Memory growth: 23.77 MiB, from 32.15 to 55.92 MiB
- Queue growth: 0
- Deadline misses: physics 0; VCU 0; independent safety 0
- Faults: none; 233 laps completed
- Thermal/energy: SOC 0.900 to 0.409, motor 40.80 °C, controller 43.75 °C;
  brake temperature is not modeled

The native Gazebo, ROS, dashboard, recorder, and race-control 30-minute soak
is `NOT_EXECUTED`.

REGRESSION:

- Passed: 145
- Failed: 0
- Skipped: 0
- ROS package-local: 6 tests, 0 errors, 0 failures, 0 skipped after a clean
  dependency-closure build

TARGET-COMPUTE RESULT:

- Workstation: `REALTIME_DEGRADED`; sustained RTF passed, but the final repeat
  had two VCU deadline misses
- Jetson: `NOT_EXECUTED_EXTERNAL_DEPENDENCY`
- Native HIL: `NOT_EXECUTED_EXTERNAL_DEPENDENCY`

FILES CREATED:

- Three Gazebo fidelity presets and four hardware profiles
- `configs/scenarios/gazebo_realtime_sil_full_lap.yaml`
- `configs/tracks/phase16_soak_circle.yaml`
- `kart_sim/realtime.py` and `kart_sim/phase16.py`
- Phase 16 profiler, runner, gate, soak, reporting, and Jetson scripts
- Phase 16 native-artifact dashboard launcher
- `tests/realtime/test_phase16.py`
- Eleven Phase 16 implementation and qualification documents
- `artifacts/phase16` bounded evidence, reports, representative payloads, and
  plots

FILES MODIFIED:

- Gazebo preset/runtime generation, full-lap executor, follower snapshot path,
  fast environment, CRC implementation, dashboard server/UI, ROS messages,
  follower bridge, Gazebo follower/sensor plugins, launch wrapper, user/test/
  demonstration/status documentation, and change manifest

KNOWN LIMITATIONS:

- The real-time sensor profile is 160x90 RGB/depth at 10 Hz and 180-sample
  LiDAR at 10 Hz; it is not evidence for high-resolution perception.
- Physics retained its validated 1 kHz rate. Lower rates failed declared
  convergence limits.
- Balanced mode exceeded its 0.5 throughput target at RTF 0.740, but one VCU
  timing outlier makes that observed run `REALTIME_DEGRADED`.
- High-fidelity offline native execution, complete P3-P8 ablation matrix,
  native full-stack 30-minute soak, rendered cone perception, complete
  controller comparison, and dynamic-obstacle/localization robustness remain
  unexecuted.
- GPU utilization could not be sampled with the installed driver stack.
- The PREEMPT_DYNAMIC workstation kernel did not repeat the zero-miss VCU
  result reliably; no deadline was extended to conceal that result.
- Jetson, HIL, physical CAN/VCU, and real vehicle measurements were not
  executed.

REAL-VEHICLE READINESS:
REJECTED

NEXT ENGINEERING TASK:
Execute the bounded 30-minute native Gazebo, ROS, dashboard, and recorder soak
on the target Jetson hardware profile and close its timing and thermal gates.
