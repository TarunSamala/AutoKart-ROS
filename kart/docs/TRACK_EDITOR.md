# Track Editor and Conversion Pipeline

Track YAML is the geometry authority. Supported source fields include
centreline points, open/closed type, width, elevation through centreline `z`,
banking, materials/surface regions, kerbs/run-off/barriers/cones, sectors,
corners, spawn points and obstacles. The current internal interpolation is
piecewise linear; `spline_source_polyline` identifies spline-intent data but is
sampled as a deterministic polyline.

Validation checks:

- finite points, positive widths and non-duplicate segments;
- banking bound;
- centreline self-intersection;
- sampled minimum radius;
- deterministic SHA-256 version.

Export:

```bash
python3 -m kart_sim.cli track export configs/tracks/cone_track.yaml \
  --output-dir runs/track-export
```

Outputs are dashboard JSON, cone JSON, Gazebo SDF, PGM/map YAML, raceline CSV
and a manifest. Occupancy starts unknown/occupied and marks only track corridor
cells free, so unknown space is never silently free.

The existing corner segmenter and dashboard editor views consume the same
source. A graphical drag editor, surveyed import and true spline/elevation mesh
generation remain future work.

