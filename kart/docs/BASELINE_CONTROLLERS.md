# Baseline controllers

Pure Pursuit selects a lookahead point and applies `atan2(2 y_local, L^2)` with
steering saturation/rate limits. Stanley uses `heading_error +
atan2(k*cross_track_error, speed + softening)` and has explicit left/right sign
tests. The longitudinal PID clamps target speed to the safety limit, uses
conditional-integration anti-windup, arbitrates drive/brake, and inhibits drive
when a brake-stuck condition is confirmed. All controllers output only a
timed high-level `MotionCommand`; the VCU remains final authority.
