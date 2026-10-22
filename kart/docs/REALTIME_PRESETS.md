# Real-Time Presets

All three presets retain internal physics authority, the VCU, independent
safety, 1 kHz actuator/physics integration, and follower-only Gazebo.

| Preset | Intended result | Measured workstation result |
| --- | --- | --- |
| `gazebo_realtime_sil` | synchronized SIL, `RTF >= 1.0` | `1.004939` first run; `1.004746` repeat with two VCU misses, degraded |
| `gazebo_engineering_balanced` | higher sensor fidelity, `RTF >= 0.5` | `0.740073`; one VCU wall-time outlier, degraded |
| `gazebo_high_fidelity_offline` | deterministic dataset/research rendering | schema/runtime generation validated; native full run not executed |

The real-time preset uses 160×90 RGB/depth at 10 Hz, 180-sample LiDAR at
10 Hz, and internal IMU at 200 Hz. This is a timing/SIL profile, not a claim of
high-resolution perception performance.

Balanced uses 640×480 RGB/depth at 20 Hz and 540-sample LiDAR at 15 Hz.
Offline uses 1280×720 RGB/depth at 30 Hz and 1080-sample LiDAR at 20 Hz.

Run:

```bash
./scripts/run_gazebo_realtime_sil.sh
./scripts/run_gazebo_balanced.sh
./scripts/run_gazebo_offline_fidelity.sh
```

`gazebo_scheduler_target_rtf` is a Gazebo scheduling target, not a physics
time scale. Internal time, physics `dt`, timestamps, and sensor capture time
are unchanged.

The preset is a qualification candidate, not an unconditional certification.
The artifact gate decides the status of each run and returns a failing process
status when any mandatory condition is false.
