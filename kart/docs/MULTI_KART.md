# Multi-Kart Racing

`MultiKartRace` composes independent `AutoKartFastEnv` instances. Each has its
own plant, VCU, independent safety object, sensor seed, command sequence, fault
set, reset epoch, collision body and namespace.

```text
/kart_01/...
/kart_02/...
/kart_03/...
```

The vector wrapper rejects duplicate namespaces and mismatched action counts.
Opponent poses enter only as LiDAR/collision actors; they do not share vehicle
state. Pairwise body collision is deterministic and creates a race-control
penalty. Lap crossings and safety states feed a shared read-only race snapshot.

Start-grid spacing is validated by the scenario editor. Supported role
configuration is schema-level: autonomous/manual request source, pace vehicle,
static/moving obstacle, overtaking and yellow/stopped-vehicle scenarios. Manual
requests still use the VCU `manual` source; they do not become actuator writes.

Run:

```bash
AUTOKART_AGENTS=3 ./scripts/run_multi_kart_race.sh
python3 -m kart_sim.cli multi-race --agents 4 --steps 500
```

Native multi-model Gazebo spawning and ROS graph namespace isolation are not
yet executed; Tier-1 isolation is covered by automated tests.

