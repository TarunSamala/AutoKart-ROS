# Watchdogs

Heartbeat payload and execution watchdogs are distinct. The windowed task
watchdog detects too-fast updates, missed deadlines, stuck sequence and
expiry. The independent layer monitors a deliberately renewed VCU pulse; the
physics-rate independent task continues when VCU scheduling is frozen.

On independent expiry the model latches isolation and brake fallback. Recovery
does not automatically restore autonomous mode. Hardware timeout, challenge
generation, output drivers and power independence require isolated bench
measurement.
