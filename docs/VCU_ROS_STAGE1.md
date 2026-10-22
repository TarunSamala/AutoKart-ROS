# VCU ROS Stage 1 — typed bench status and package discovery

**Date:** 2026-10-17  
**Roadmap gate:** M4 Pi/ESP32 bench contract, partial software work  
**Vehicle release:** **NO-GO**; no actuator authority added.

## What changed

| Source | Change | Purpose |
|---|---|---|
| `kart/ros2_ws/src/autokart_msgs/msg/BenchVcuStatus.msg` | New typed, bench-specific status | Preserve the existing eight-word NOOP status fields, host receipt time, packet validity and stale state without representing them as measured vehicle feedback. |
| `kart/ros2_ws/src/autokart_msgs/CMakeLists.txt` | Registers the new interface | Generate Python/C/C++ ROS type support. Existing message definitions remain unchanged. |
| `kart/ros2_ws/src/autokart_diagnostics/scripts/bench_vcu_status` | Read-only ROS executable | Decode `/autokart/bench/status` into `/autokart/bench/vcu_status`; reject malformed/regressed packets and publish stale status after 500 ms. |
| `kart/ros2_ws/src/autokart_diagnostics/{CMakeLists.txt,package.xml}` | Installs the executable and declares runtime dependencies | Make the existing diagnostics package runnable without creating another package or transport stack. |
| Nine CMake `package.xml` manifests | Added `<export><build_type>ament_cmake</build_type></export>` | Fix pre-existing `ros.catkin` misclassification, which prevented these built packages from appearing in a sourced ROS overlay. No package names changed. |
| `docs/NODE_CONTRACTS.md`, `PROJECT_INDEX.md`, `vcu/README.md` | Document the bench-only boundary | Keep the new message separate from `/vcu/state` and the production command design. |

The decoder accepts only protocol version 1, exactly eight integer fields and
an empty array layout. It checks nonzero boot ID, bench SAFE state 0, known
fault bits, command-age sentinel consistency, and monotonic uptime/sequence
within a boot. It never publishes a command, opens serial, enables firmware,
or maps bench status into the dashboard's LIVE VCU channel. Its fixed
`vehicle_authorized=false` is a declaration of this node's scope, not proof
of a physical interlock. ROS topic origin is not authenticated.

## Build and discovery evidence

The untouched 16-package workspace built in a temporary baseline overlay:
[`baseline_build.log`](vcu_ros_evidence/baseline_build.log). After adding the
message and adapter, an isolated full build again finished all 16 packages:
[`final_build.log`](vcu_ros_evidence/final_build.log). The final one-package
refresh installed the last layout-validation change:
[`diagnostics_refresh.log`](vcu_ros_evidence/diagnostics_refresh.log). The
installed executable checksum matched its source after the refresh.

The first after-change overlay exposed a pre-existing metadata problem:
`colcon list` had classified nine `ament_cmake` packages as `ros.catkin`.
Although those CMake targets compiled, sourcing that overlay did not expose
`autokart_msgs` or `autokart_diagnostics` to `ros2 pkg`. After the manifest
correction, all 16 packages classify as ROS 2 `ament_cmake`/`ament_python`,
and the sourced final overlay exposes all 16 packages, the
`autokart_msgs/msg/BenchVcuStatus` interface, and the
`autokart_diagnostics bench_vcu_status` executable. See
[`discovery.log`](vcu_ros_evidence/discovery.log).

The build reported Gazebo CMake and unavailable pytest registration warnings.
They did not fail the build. **No ROS runtime, Agent connection, hardware
status stream, actuator, or test suite was run in this stage.** The typed
message is host-side only; the existing bench firmware still sends
`std_msgs/UInt64MultiArray` and accepts only NOOP commands.

## Reproduce the local build

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/autokart_vcu_ros_final_log build \
  --base-paths kart/ros2_ws/src \
  --build-base /tmp/autokart_vcu_ros_final_build \
  --install-base /tmp/autokart_vcu_ros_final_install \
  --executor sequential
source /tmp/autokart_vcu_ros_final_install/setup.bash
ros2 interface show autokart_msgs/msg/BenchVcuStatus
ros2 pkg executables autokart_diagnostics
```

The `/tmp` overlay is a local build artifact and can be rebuilt. Source files
remain at their existing repository paths. The historical workspace
`kart/ros2_ws/build`, `install` and `log` trees were not overwritten.

## Next M4 steps

1. Obtain the as-wired ESP32-S3, sensor, brake and independent-stop evidence.
2. Freeze a **separate production** Pi↔VCU request and telemetry contract,
   with boot/session identity, integrity, measurement validity and measured
   stopping response. The bench NOOP array is not a vehicle command schema.
3. Build an ESP-IDF hardware adapter only after the pinout and safe boot
   states are verified; preserve firmware backups before any approved flash.
4. With traction isolated, prove actual Agent/client sessions, status timing,
   disconnect/stale behavior and packet provenance on the Pi. Then perform
   independent physical safety validation before any vehicle authorization.
