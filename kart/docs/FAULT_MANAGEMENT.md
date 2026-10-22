# Fault Management

`FaultManager` stores code, severity, first/latest occurrence, count, active or
historical status, latch policy, required response, and reset requirement.
Critical reset clears active records only after the centralized reset guards
pass; history remains. A bounded `FaultSnapshotBuffer` records observable VCU
state, speed, steering, brake pressure, motor torque and current on a new
fault.

Operational diagnostics use observable feedback. Fault-injector identity is
used only by test oracles to prove that the intended physical model fault was
activated.

Representative DTCs:

| Code | Meaning | Response |
| --- | --- | --- |
| `BRAKE-003` | Mechanical-transmission response missing | remove propulsion, emergency assessment |
| `MOTOR-003` | Uncommanded positive motor torque | disable, isolate, emergency brake |
| `POWER-004` | Traction isolation feedback mismatch | lockout and sustained emergency braking |
| `CONTROLLED_STOP_FAILED` | Stop did not meet progress timeout | emergency escalation |

Debounce is implemented in the steering, brake, and traction monitors. Physical
E-stop and independent watchdog actions are immediate and separately latched.
Reset never erases history or returns directly to autonomous mode.

Known limitation: this is a deterministic research simulator DTC model, not a
production diagnostic stack or a functional-safety certification artefact.
