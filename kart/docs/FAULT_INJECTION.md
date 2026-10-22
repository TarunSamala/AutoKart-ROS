# Fault Injection

Phase 4 supports deterministic direct injection through `DynamicFourWheelKart.inject_fault("target:fault")`. Implemented targets: `steering:jam`, `steering:power_loss`, `steering:linkage_break`, `brake:stepper_stall`, `brake:cable_break`, `brake:hydraulic_leak`, `brake:not_releasing`, `motor:torque_loss`, `motor:controller_disabled`, `motor:torque_stuck_positive`, `motor:wrong_sign`, `motor:chain_break`, and `battery:bms_shutdown`.

Declarative timed fault YAML, detection timers, and VCU safe-state responses are Phase 7 work. Current fault outputs are telemetry and must not be mistaken for a safety controller.
