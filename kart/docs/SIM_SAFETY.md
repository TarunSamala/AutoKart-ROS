# Simulation Safety

The upgrade preserves the existing VCU, dynamic safety envelope, controlled
stop, independent watchdog/E-stop, fault latching and guarded reset.

Integrated command path:

```text
source -> high-level command -> freshness/sequence/CRC/epoch/mode/limit checks
       -> speed/curvature safety envelope -> independent safety
       -> actuator dynamics -> physics authority
```

New command checks include validity, operating mode, acceleration/deceleration
limits and reset epoch. Reset creates a new epoch. A pre-reset command produces
`COMM-EPOCH`.

Fast-mode reset defaults to SAFE and does not re-arm. Explicit operator
acknowledgement is required. Moving reset needs physical stopped confirmation;
active faults need repair acknowledgement. Independent E-stop removes motor
enable and causes termination.

The following remain invariant:

- no stale or invalid command controls the kart;
- no ROS/Gazebo/dashboard/race-control direct actuator path;
- independent safety overrides normal software;
- E-stop removes propulsion;
- faults latch;
- no automatic autonomy restart;
- unknown occupancy is unsafe;
- one physics owner per namespace/epoch.

`low_level_component_test` is documented in `PHYSICS_AUTHORITY.md` and is not
reachable from the integrated launch, GUI or dashboard.

This is simulation evidence only. Functional-safety compliance and real-kart
readiness remain rejected.

