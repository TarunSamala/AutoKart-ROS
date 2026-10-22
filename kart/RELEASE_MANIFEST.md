# Research release manifest — 0.1.0-prototype

Reproduce locally: `./scripts/test.sh`; `kart-sim test-model autonomy`;
`kart-sim benchmark critical-smoke`; `kart-sim dashboard export <telemetry.jsonl>
--output-dir <report>`. The reference configurations are under `configs/karts`,
tracks under `configs/tracks`, SIL config under `configs/autonomy`, and critical
benchmark catalog under `configs/benchmarks`.

This is a headless simulation prototype. Real actuator outputs are not enabled
by this release. Native ROS, Chrono, CARLA/Unreal, hardware HIL, and real-kart
validation require external evidence and are not release claims.
