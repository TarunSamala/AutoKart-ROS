# Independent Safety Layer

The independent layer is a separate deterministic task and state latch. It
monitors the VCU pulse and physical E-stop input at physics rate. Timeout or
E-stop activation independently requests motor isolation, contactor opening,
and brake fallback even when the VCU task no longer executes.

Its outputs have higher priority than VCU outputs:

```text
VCU output
→ independent permission check
→ force motor enable false
→ request contactor open
→ force brake fallback
→ dynamic actuator and vehicle models
```

Pulses do not clear a latched trip. Reset requires authorization, resolved
source, released E-stop and physically stopped vehicle. It never restores
autonomous operation.

The software model demonstrates logical separation only. Real independence,
normally-closed wiring, safety power, watchdog electronics, brake fallback,
contactor timing and failure behavior require hardware validation.
