# Trajectory interface

`Trajectory` is a map-frame, time-parameterized sequence of points
`(t, x, y, z, heading, speed, acceleration, curvature, jerk?, track_s?)`.
Validation rejects stale trajectories, duplicate/decreasing times, short
horizons, frame mismatch, non-finite values, and configured speed,
acceleration, or curvature violations. A spatial path without timing is not a
valid trajectory.
