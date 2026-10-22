# Dashboard data model

## Normalized telemetry record

Each record contains:

- `schema_version`, `sequence`, authoritative `timestamp_s`, validity, stale
  state, and display-interpolation metadata.
- `pose`: ENU `x/y/z` metres and roll/pitch/yaw radians.
- `motion`: body speeds, scalar speed, accelerations, yaw rate, and sideslip.
- `track`: arc length, lateral error, heading error, curvature, grade, bank,
  boundary margins, and four wheel surface lookups.
- `commands`: requested, validated, safety-limited, and final values.
- `vcu`: state, active/latched faults, independent safety, speed-limit
  candidates, stopping-distance terms, command age, and final authority state.
- `subsystems`: actual four-wheel, steering, brake, motor, battery, and power
  feedback.

Missing simulator signals remain `null` or visibly “not recorded.” Delivery
time is not substituted for measurement time.

## Track schema

Each sampled track point carries `s_m`, ENU position, heading, curvature,
grade, bank, independent left/right width, material, and explicit boundaries.
The payload also includes material definitions, priority surface regions,
corner segments, start/finish distance, closed/open state, and provenance.

## Kart schema

Wheelbase, front/rear tracks, wheel radius/width, mass, CG, and chassis display
envelope come from the validated kart configuration. `placeholder: true`
identifies the procedural visual; it is not a CAD claim.

