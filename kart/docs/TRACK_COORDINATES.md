# Track Coordinates

`s` is accumulated centreline arc length in m; `e_y > 0` is left of travel; `e_z` is upward from centreline. `world_from_track(s,e_y)` uses the planar left normal `[-sin(heading), cos(heading), 0]`. Projection returns `s`, lateral/vertical offset, heading, margins, distance, and segment ID. Closed tracks wrap `s`; open tracks clamp it.
