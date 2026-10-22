PHASE:
Phase 12 — CARLA / Unreal Integration

SOFTWARE FRAMEWORK:
COMPLETE

NATIVE INTEGRATION:
NOT_EXECUTED_EXTERNAL_DEPENDENCY

OVERALL STATUS:
PARTIAL

VERDICT:
MORE_DATA_REQUIRED

IMPLEMENTED LOCALLY:
- Explicit simulator-physics authority, synchronous visual-follower contract,
  coordinate conversion, actor validation, deterministic track payload and
  render-sensor callback/time contract with a headless mock.

TESTED LOCALLY:
- Coordinate round trip, authoritative following, duplicate actor rejection,
  track ordering/width validation and sensor measurement/delivery chronology.

NOT EXECUTED:
- CARLA/Unreal clients, map import, native render callbacks, weather,
  synchronization, performance and rendered-sensor fidelity.

REASON:
- `carla` and `unreal` modules are unavailable.

COMMAND TO VERIFY WHEN AVAILABLE:
- `python3 -c "import carla; print(carla.__file__)"`
- Run the future native engine adapter tests.

CLAIM:
- No rendered-sensor fidelity is claimed.
