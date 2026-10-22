# Gazebo Follower Full-Lap Demonstration

Run:

```bash
./scripts/run_gazebo_follower.sh \
  --scenario configs/scenarios/gazebo_follower_full_lap.yaml
```

The deterministic scenario uses `research_circle.yaml`, seed `20260729`,
Pure Pursuit, the VCU, independent safety and the internal dynamic four-wheel
plant. Gazebo Classic 11 runs as a static visual/sensor follower.

## Expected state sequence

```text
POWER_OFF -> BOOT -> SELF_TEST -> SAFE -> READY -> AUTONOMOUS
-> one complete lap -> CONTROLLED_STOP -> SAFE
```

The executed lap completed at 12.260 s and reached final `SAFE` at 16.560 s
after 16,560 physics steps. Gazebo applied 1,657 follower updates. The
authoritative log hash was `28164b960133afe84a2b8fe6d192a3d41b9174b68f495e11bb7ac137b8b349a2`.

Maximum position, orientation, steering and wheel-rotation mismatch were 0 at
authoritative update boundaries; maximum clock offset was 1 ms. No critical
diagnostic, fault or double-physics violation occurred.

## Artifacts

`artifacts/gazebo_follower/full_lap` contains:

- run manifest and per-file SHA-256 checksums;
- internal vehicle, control request, Gazebo pose and joint logs;
- full sensor-delivery metadata and native diagnostics;
- representative changing RGB frames, depth statistics and LiDAR scans;
- twelve plots including trajectory/error, steering, wheel rotation, clocks,
  sensor timing/drop, RTF, controlled stop and update-rate independence.

The measured software-render run had aggregate RTF 0.197, approximately 103%
CPU for the ROS recorder process and 176.7 MiB maximum RSS. GPU measurement was
not executed because `nvidia-smi` was unavailable.

Additional commands:

```bash
./scripts/test_gazebo_synchronization.sh
./scripts/test_gazebo_process_loss.sh
./scripts/test_no_double_physics.sh
./scripts/test_gazebo_follower.sh
./scripts/report_gazebo_follower.sh
```

The samples demonstrate native rendered payloads; they are not a prerecorded
animation and pose always comes from the current internal simulation state.
