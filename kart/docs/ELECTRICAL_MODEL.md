# Electrical Model

**Verdict: APPROVED_WITH_CONDITIONS.** Battery current is positive for discharge. The initial Thevenin model uses `Vterminal=VOC-I*R`; SOC is updated by `SOCdot=-I/(3600*capacity_Ah)`. A pre-charge RC state advances bus voltage before contactor closure. Battery energy, discharged energy, regenerated energy, SOC, contactor state, and bus voltage are telemetry fields.

Steering and brake are separate regulated power domains with brownout thresholds. VCU, compute, sensor, and safety domains are configuration placeholders only in this increment. BMS shutdown and welded-contactor state exist in the electrical abstraction but do not yet drive a VCU safety state machine.
