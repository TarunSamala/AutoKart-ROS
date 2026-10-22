# Physics Rate Qualification

The reference is the validated 1,000 Hz dynamic four-wheel plant. Candidate
rates 500, 400, 250, and 200 Hz were compared over straight acceleration,
braking, constant-radius motion, slalom, wet stop, steering jam, brake cable
failure, and stuck torque.

Limits were 0.20 m position, 0.15 m/s speed, 0.15 rad/s yaw rate, 0.03 rad
steering, 0.25 m stopping distance, 10 ms fault detection, 20 ms torque
removal, and 2% energy.

Every lower rate failed at least one limit. At 500 Hz, for example, the
constant-radius position difference was 0.281 m, slalom position difference
0.472 m, and several energy cases exceeded 2%. Therefore:

```text
selected_operational_rate_hz: 1000
```

The complete machine-readable matrix is
`artifacts/phase16/physics_rate_qualification.json`. The physics rate was not
lowered to obtain real-time performance.
