# Controlled Stop

The manager executes `INITIATE`, `REMOVE_PROPULSION`, `STABILIZE`,
`DECELERATE`, `FINAL_STOP`, `HOLD`, `COMPLETE`, or `FAILED`. It always
prohibits positive propulsion, holds the achieved steering angle, and commands
the dynamic brake actuator rather than directly setting deceleration.

Completion requires fresh valid vehicle and wheel-speed feedback, every valid
wheel below 0.05 m/s, vehicle speed below 0.05 m/s, actual motor torque below
0.1 Nm, motor current below 1 A, hydraulic pressure at or above 100 kPa, and
the condition held for 0.05 s. Motion after initial hold revokes completion.
Timeout generates `CONTROLLED_STOP_FAILED` and
`STOPPING_CAPABILITY_INSUFFICIENT`, removes motor enable, requests contactor
opening, and applies emergency braking.

Executed tests cover nominal, command loss, corner, wet surface, reduced brake,
false-zero feedback, rolling restart, timeout, grade hold, torque persistence
and missing pressure. Measured reference-scenario values are in the Phase 7
report. Thresholds remain provisional pending brake/vehicle measurements.
