# Physics Authority Policy

**SIMULATION USE ONLY — NOT APPROVED FOR DIRECT VEHICLE DEPLOYMENT.**

## Invariant

Exactly one backend owns chassis integration, wheel integration, contacts, and
simulation time for a kart during an execution epoch.

```yaml
simulation:
  physics_backend: internal   # internal | chrono | gazebo
  gazebo_role: disabled       # disabled | visual_follower | authority
```

The valid combinations are:

| `physics_backend` | `gazebo_role` | Vehicle integration |
|---|---|---|
| `internal` | `disabled` | deterministic internal plant |
| `internal` | `visual_follower` | internal plant; Gazebo consumes poses/joints |
| `chrono` | `disabled` | Chrono adapter |
| `chrono` | `visual_follower` | Chrono adapter; Gazebo consumes poses/joints |
| `gazebo` | `authority` | reviewed Gazebo system plugin only |

Every other combination is rejected before launch. In particular,
`physics_backend: internal` plus a Gazebo drive/vehicle plugin is invalid.

## Authority lease

The platform creates an immutable authority lease at launch:

```text
backend name + kart namespace + reset epoch + fixed step + owner ID
```

Only the lease owner may call vehicle integration. A second owner, a time step
from a visual plugin, or a mismatched reset epoch is a fatal configuration
error. The selected authority is written into telemetry and run metadata.

## Internal or Chrono authority

```text
VCU-safe actuator requests
-> actuator models
-> internal/Chrono physics
-> chassis and four-wheel state
-> Gazebo world-pose follower + joint-state follower
```

Gazebo gravity/contact/drive effects on the kart are disabled in follower mode.
The visual model may retain collision geometry for visualization or sensor
occlusion, but those contacts do not feed a second kart motion solution.
Collision events used by the authoritative core come from the common world
representation or an explicitly reviewed contact adapter.

## Gazebo authority

Gazebo authority is opt-in and not the default. It requires a custom system or
hardware interface that:

- accepts only VCU-validated actuator requests;
- models steering/brake/motor actuator dynamics rather than ideal velocity;
- enforces the common rear axle constraint and reports axle/wheel state;
- publishes a complete authoritative state once per physics step;
- passes backend parity, safety, reset, and no-double-physics tests.

Gazebo authority must be identified in every report. Its output is compared
against the internal model on straight, circle, brake, steering-step, and
split-friction cases. A standard differential-drive or Ackermann velocity
plugin is not accepted as the engineering plant.

## Visual follower contract

The follower input contains:

- reset epoch and simulation timestamp;
- base pose and twist;
- steering-column/actuator positions;
- four wheel angles and angular rates;
- rear axle angular state;
- brake, VCU and E-stop indicators.

The follower has no command subscription and no actuator service. It rejects
backwards timestamps, epoch mismatch, duplicate state owners, and direct
actuator command fields.

## Reset

Reset is a transaction:

1. remove command authority and assert stopped/disabled output;
2. pause the scheduler and visual follower;
3. increment epoch and clear command queues;
4. reset exactly one selected physics backend;
5. synchronize follower pose and joints without stepping physics;
6. require new VCU self-test, READY transition and enable request.

Old commands are invalid even when their wall-clock expiry has not passed.

## Component-test exception

`low_level_component_test` can command one isolated actuator model directly.
It requires all of:

- no autonomy or ROS motion-command subscriber;
- wheels raised or propulsion mechanically disabled for HIL;
- race and safety-evaluation flags disabled;
- explicit component and duration;
- telemetry banner `DIRECT_COMPONENT_TEST`;
- real actuator output disabled unless separately armed by a physical test
  procedure.

This exception cannot be enabled by Gazebo GUI, dashboard, or race control.

