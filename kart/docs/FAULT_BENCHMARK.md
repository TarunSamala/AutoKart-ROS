# Fault-injection benchmark

The deterministic benchmark separates physical scenario injection, observed
telemetry, safety diagnostics, and oracle evaluation. Run the fast critical
tier with `kart-sim benchmark critical-smoke`. It runs cable-break, stuck-torque,
and VCU-freeze scenarios in stable case-ID order and writes per-case evidence.
It does not claim real-world or exhaustive fault coverage.
