# Dashboard kart model

The current visual is a procedural engineering placeholder constructed from:

- validated wheelbase and independent front/rear track widths;
- tire radius and width;
- CG location and chassis envelope;
- four measured wheel steering/angular-speed/contact states.

It shows chassis, wheels, solid rear axle, seat, steering column, battery,
motor, VCU, compute enclosure, LiDAR puck, camera location, CG, and E-stop
indicator. Component enclosure dimensions and styling are schematic.

The interface advertises:

```text
PLACEHOLDER KART MODEL
replacement: dashboard/assets/kart/kart.glb
```

A real CAD-derived GLB remains required. Replacing it must not change telemetry
or physics scale.

