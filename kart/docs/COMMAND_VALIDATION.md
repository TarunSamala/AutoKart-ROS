# Command Validation

Motion commands contain source, sequence, generation and expiry timestamps,
speed, steering, brake, E-stop request, and CRC-16/CCITT-FALSE. The CRC uses
polynomial `0x1021`, initial value `0xFFFF`, no reflection and no final XOR;
the standard `123456789` vector is `0x29B1`. CRC is error detection, not
authentication.

The VCU rejects unauthorized, stale, future, duplicate, reverse,
excessive-gap, non-finite, or corrupt commands before actuator effect. The
8-bit sequence counter accepts `255 → 0` wrapping. A missing or expired command
removes propulsion and initiates controlled stop. Steering and speed are
safety-limited and the modification is visible in final-command telemetry.

Production cryptographic authentication and measured communication timeout
budgets remain outside this simulator phase.
