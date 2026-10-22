# VCU State Machine

The implemented states are `POWER_OFF`, `BOOT`, `SELF_TEST`, `SAFE`, `MANUAL`,
`READY`, `AUTONOMOUS`, `DEGRADED`, `CONTROLLED_STOP`, `EMERGENCY_STOP`,
`FAULT_LOCKOUT`, and `SERVICE_MODE`.

The normal startup path is:

```text
POWER_OFF → BOOT → SELF_TEST → SAFE → READY
```

Autonomous entry is a dedicated guarded request; a generic transition cannot
enter it. Critical recovery is:

```text
EMERGENCY_STOP or FAULT_LOCKOUT
→ authorized reset
→ SELF_TEST
→ SAFE
→ READY
```

No old command or sequence state survives the reset. `CONTROLLED_STOP` executes
`INITIATE`, `REMOVE_PROPULSION`, `STABILIZE`, `DECELERATE`, `FINAL_STOP`,
`HOLD`, `COMPLETE`, or `FAILED`. Completion uses fresh vehicle/wheel speeds,
actual torque/current and brake hold; timeout escalates to emergency stop.

Run `PYTHONPATH=. python3 -m kart_sim.cli vcu state-machine --diagram` for the
machine-generated compact diagram.
