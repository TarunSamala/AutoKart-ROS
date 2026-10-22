# Race Control

Race-control states are:

```text
SETUP -> GRID -> READY -> COUNTDOWN -> RUNNING
                                      |-> YELLOW -> RUNNING
                                      |-> RED
                                      |-> FINISHED
                                      `-> ABORTED
```

Invalid transitions raise a simulation error. Per-kart state includes lap
count, current/best lap, sectors, cone and track-limit penalties, collisions,
DNF, E-stop, safety state and faults.

Race control receives simulation events and safety observations. It has no
command or reset method for the VCU/independent safety layer. An E-stop marks
the kart DNF but remains latched and reset only through the safety workflow.

The web dashboard has a dedicated Race tab. Normalized telemetry preserves
`race_control`; absent race data shows SETUP/zero rather than synthesizing a
competition result.

Multi-kart results include the race snapshot in their `info` mapping. Native
networked race-control services and official event scoring are out of scope
until the rules and integration are verified.

