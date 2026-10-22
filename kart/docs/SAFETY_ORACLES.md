# Safety oracles

The campaign oracle reads telemetry rather than fault-injector identity. It
checks expected diagnostic evidence, safe-state entry, no positive drive in
emergency stop, and writes explainable per-case evidence. It is intentionally
conservative: missing telemetry fails a campaign rather than passing it.
