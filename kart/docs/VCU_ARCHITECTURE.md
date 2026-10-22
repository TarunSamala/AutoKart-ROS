# VCU Architecture

**Verdict: PROTOTYPE_TEST_REQUIRED.** `VCU` separates a high-level `MotionCommand` from resulting `VehicleCommand`. It has explicit state, checksum/sequence/freshness validation, a stopping-distance envelope, controlled-stop output, and an independent watchdog model. It is not yet wired into scenario execution or actuator feedback monitoring.
