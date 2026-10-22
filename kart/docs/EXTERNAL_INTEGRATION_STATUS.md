# External Integration Status

## ROS 2

IMPLEMENTED LOCALLY:
- ROSIDL message files, conversion functions, topic/QoS/time/frame contracts,
  deterministic mock transport, and optional package boundary.

TESTED LOCALLY:
- Wire conversion/CRC, clock, frames, QoS, ground-truth isolation, controller
  to VCU freshness, and deterministic mock SIL.

NOT EXECUTED:
- Native `rclpy` graph, launch, rosbag, DDS QoS interoperability, and native
  publisher/subscriber tests.

REASON:
- External dependency unavailable (`ros2` CLI detected; `rclpy` unavailable).

COMMAND TO VERIFY WHEN AVAILABLE:
- `colcon build --base-paths adapters/ros2`
- `colcon test --packages-select kart_sim_ros2_adapter`

## Project Chrono

IMPLEMENTED LOCALLY:
- Backend protocol, analytical adapter, explicit Chrono boundary, schema-like
  configuration, dependency detection, and no-silent-fallback policy.

TESTED LOCALLY:
- Analytical backend interface, snapshot/wheel data, Chrono configuration, and
  explicit unavailable-dependency failure.

NOT EXECUTED:
- Native Chrono kart construction, dynamics, tires, determinism, and
  analytical-versus-Chrono comparison.

REASON:
- External dependency unavailable (`pychrono`/`chrono` not found).

COMMAND TO VERIFY WHEN AVAILABLE:
- `python3 -c "import pychrono; print(pychrono.__version__)"`
- `python3 -m pytest tests/native/test_chrono_backend.py -q` after providing the native adapter test.

## CARLA / Unreal

IMPLEMENTED LOCALLY:
- Coordinate conversion, synchronous visual-follower contract, deterministic
  track payload, render-sensor callback interface, engine authority policy,
  and headless mock.

TESTED LOCALLY:
- Coordinate round trip, authoritative following, duplicate actors, track
  export ordering/widths, and measurement-versus-delivery timing.

NOT EXECUTED:
- CARLA/Unreal clients, map import, rendering, native callbacks, weather,
  engine synchronization, and rendered-sensor fidelity.

REASON:
- External dependencies unavailable (`carla` and `unreal` modules not found).

COMMAND TO VERIFY WHEN AVAILABLE:
- `python3 -c "import carla; print(carla.__file__)"`
- `python3 -m pytest tests/native/test_carla_unreal_adapter.py -q` after installing the engine-specific test.

## Jetson / RP2350 / CAN HIL

IMPLEMENTED LOCALLY:
- Exact simulation-only CAN layout, CRC-8/SAE-J1850, counter/wrap/gap checks,
  timeout-safe command, simulated plant bridge, timing records, bus-load
  calculation, vCAN detection, and a no-real-output safety policy.

TESTED LOCALLY:
- Encode/decode, CRC vector/corruption, counter wrap/duplicate, timeout, plant
  command conversion, safe stale response, and bus load.

NOT EXECUTED:
- vCAN traffic, SocketCAN timestamping/bus-off, real Jetson, real RP2350 VCU,
  real CAN adapters, bench E-stop, and any physical output.

REASON:
- `vcan0` is not configured and hardware is unavailable.

COMMAND TO VERIFY WHEN AVAILABLE:
- `ip link show vcan0`
- `python3 -m pytest tests/native/test_socketcan_vcan.py -q`
- Follow `docs/HIL_SAFETY_PLAN.md` before any hardware test.

## Real-kart calibration

IMPLEMENTED LOCALLY:
- Dataset metadata schema, immutable checksum, frozen calibration/validation/
  holdout split registry, parameter registry, CSV import, deterministic bounded
  optimizer, synthetic recovery, and report template.

TESTED LOCALLY:
- Split leakage rejection, bounds, deterministic recovery, identifiability
  rejection, and report claim state.

NOT EXECUTED:
- Real data import, time alignment, physical parameter fitting, uncertainty
  estimation from measurements, held-out validation, and sim-to-real testing.

REASON:
- Real-kart datasets and hardware measurements are unavailable.

COMMAND TO VERIFY WHEN AVAILABLE:
- `python3 -m pytest tests/native/test_real_dataset_import.py -q --real-data <path>`
- Execute the staged procedures in `docs/REAL_TEST_PLAN.md`.
