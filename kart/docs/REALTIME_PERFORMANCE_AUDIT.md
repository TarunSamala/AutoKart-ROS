# Real-Time Performance Audit

Phase 16 was measured before rates or fidelity were changed. The previous
native full-lap result was `RTF 0.181988` (reported baseline `0.197`) on:

- Intel Core i7-9750H, 6 cores / 12 threads
- 15.34 GiB RAM, Ubuntu 22.04.5, Linux 6.8 PREEMPT_DYNAMIC
- Mesa Iris on Intel UHD 630, headless Gazebo Classic 11
- NVIDIA GTX 1650 Mobile present through nouveau; `nvidia-smi` unavailable

The baseline recorder used 101.27 s wall time, 102.10 s user time,
2.86 s system time, and 178,216 KiB maximum RSS. Sampled `gzserver` load was
approximately 140–150% CPU and 545 MiB RSS.

## Measured bottlenecks

`cProfile` showed that the headless core was dominated by four-wheel dynamics,
track projection, VCU/feedback construction, CRC calculation, immutable
snapshot construction, and evidence serialization. The ROS executor profile
then showed significant wait-set/callback overhead when the authoritative loop
and executor shared one Python process. Rendered sensor rate reductions alone
did not remove this bottleneck.

Implemented measurements and optimizations:

1. Exact `binascii.crc_hqx` CRC replacement, verified against the old result.
2. Direct snapshot copies from completed plant state; no asynchronous mutable
   telemetry reads.
3. Snapshot allocation at 100 Hz while wheel rotation and authoritative logs
   remain integrated at 1,000 Hz.
4. Separate authoritative process with a bounded, one-way snapshot queue.
5. Single-threaded ROS executor for the real-time profile.
6. Separate CPU cores for Gazebo, ROS delivery, and the authoritative plant on
   the measured workstation.
7. Bounded representative payload hashing and bounded evidence queues.
8. A 1.005 wall-time pacing target to compensate scheduler wake-up overhead;
   physics remains exactly 1 ms.

The first qualifying artifact is `artifacts/phase16/realtime_qualified`. It
reached aggregate `RTF 1.004939`, minimum complete five-second-window
`RTF 1.004762`, and zero VCU/safety misses.

The exact post-regression repeat is `artifacts/phase16/realtime_final`. It
preserved aggregate `RTF 1.004746`, all safety/sensor invariants, and the exact
authoritative hash, but recorded two 1 ms VCU wall-deadline overruns (maximum
1.256 ms). Its gate correctly reports `REALTIME_DEGRADED`. Consequently this
workstation demonstrates real-time throughput but not stable zero-miss
qualification on the non-real-time PREEMPT_DYNAMIC kernel.

The authoritative log SHA-256 remained
`28164b960133afe84a2b8fe6d192a3d41b9174b68f495e11bb7ac137b8b349a2`
through all qualifying changes.

GPU utilization is `NOT_EXECUTED_EXTERNAL_DEPENDENCY` because the installed
driver stack exposes neither `nvidia-smi` nor an equivalent sampled counter.

## Recorded resource evidence

| Item | Baseline | Exact final repeat |
|---|---:|---:|
| Simulation duration | 16.560 s | 16.560 s |
| Synchronized core wall time | about 91.0 s | 16.482 s |
| Outer bridge/recorder wall time | 101.27 s | 20.83 s |
| RTF | 0.181988 | 1.004746 |
| Bridge/recorder CPU | not retained | 102% |
| Bridge/recorder maximum RSS | 178,216 KiB | 147,724 KiB |
| Authoritative core CPU time | not retained | 13.35 s |
| Authoritative core maximum RSS | not retained | 71.87 MiB |
| Gazebo server sampled CPU/RSS | 140–150% / 545 MiB | not separately sampled |
| Final artifact bytes | not retained | 13,183,663 |
| Approximate artifact write rate | not retained | 0.63 MB/s outer-wall average |
| Maximum sensor queue depth | 1 | 1 |
| Maximum producer acknowledgement lag | not retained | 60 ms bounded lead |

GPU utilization and temperature are
`NOT_EXECUTED_EXTERNAL_DEPENDENCY`. Per-callback ROS serialization time,
per-sensor renderer CPU time, final-run Gazebo/render CPU split, concurrent
dashboard load, and recorder-only ablation are `NOT_MEASURED`. They are not
inferred from aggregate process CPU. Configured capture-to-delivery latency
was verified from timestamps (RGB 10 ms, depth 20 ms, LiDAR 10 ms, IMU 3 ms);
that is not relabelled as callback execution time.
