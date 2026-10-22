# Coordinate Systems and Units

Internal values use SI units: m, s, kg, N, N m, rad, rad/s, V, A, J, W, Pa, and K. Angular configuration values are radians; a human-readable degree comment may appear only at the boundary.

| Frame | Origin | Axes |
| --- | --- | --- |
| World `W` | scenario-defined | +X east/forward map direction, +Y left/north, +Z upward |
| Vehicle body `B` | vehicle reference point | +x forward, +y left, +z upward |
| Track `T` | centreline station | +x along increasing station, +y left, +z upward |
| Wheel `Wi` | contact/wheel centre (Phase 3) | +x rolling forward, +y left, +z upward |
| Sensor `S` | sensor mount | explicitly configured; camera optical convention documented with adapter |

Positive yaw and yaw rate are counter-clockwise when viewed from above. Planar propagation uses `Xdot = u cos(yaw) - v sin(yaw)` and `Ydot = u sin(yaw) + v cos(yaw)`. The Phase 2 test suite checks that a positive steering input produces positive yaw and positive world Y from an initial positive forward speed.
