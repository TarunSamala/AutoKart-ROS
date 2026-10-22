# Formula Student-Style Events

This platform makes no official Formula Student compliance claim. Dimensions,
procedures, scoring and rules require verification against current rules and
measured facilities.

Configurable templates are in `configs/events`:

- acceleration: standing start, timed distance, maximum speed and stop zone;
- skidpad: left/right lap counts, timing, cone penalties and lateral report;
- autocross: unknown cone track, perception/planning interface and one lap;
- trackdrive: multiple laps, thermal/energy/fault/safety reporting;
- emergency-brake test: trigger, E-stop, stopping distance/time and boundary.

`kart_sim.formula_student.EventEvaluator` deterministically accumulates start
and finish time, laps, cone and track-limit events, maximum speed, consumed
energy, EBS stop distance/time and boundary violation. `RaceControl` owns
session/timing status but cannot clear E-stop or command actuators.

Event tracks are one-source YAML and are convertible to dashboard, cone,
occupancy, raceline and SDF forms. The skidpad centreline and event values are
provisional simulation inputs, not certified layouts.

Run:

```bash
./scripts/run_formula_student_event.sh acceleration
./scripts/run_formula_student_event.sh skidpad
./scripts/run_formula_student_event.sh autocross
./scripts/run_formula_student_event.sh trackdrive
./scripts/run_formula_student_event.sh emergency_brake_test
```

The fast commands route through the VCU. The existing EBS scenario additionally
exercises the detailed actuator/safety plant.

