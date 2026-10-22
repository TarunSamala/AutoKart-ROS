# Emergency Stop

Emergency inputs outrank normal commands. The integrated response is:

```text
trip
→ reject normal command
→ final torque request zero
→ motor enable false
→ contactor open request
→ independent isolation latch
→ full dynamic brake request
→ hold achieved steering
→ monitor torque, pressure, speed and stop
```

The physical-input simulation path is separate from the high-level command
path. The independent task continues during VCU freeze. Its stop latch is not
cleared by resumed VCU pulses; reset requires authorization, released/resolved
input and stopped vehicle. VCU reset then passes `SELF_TEST → SAFE → READY`;
there is no direct autonomous restart.

Executed simulation tests cover at rest, acceleration, cornering, stuck torque,
VCU freeze, safety-task freeze, remote request and welded contactor. Real
normally-closed wiring, safety power, contactor, watchdog and brake fallback
have not been tested. Real-vehicle readiness remains rejected.
