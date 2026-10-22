# Track Model

**Verdict: PROTOTYPE_TEST_REQUIRED.** The headless `Track` model uses an ordered XY/Z polyline, deterministic cumulative arc length, and optional closed-loop wrapping. Users interact in metres of `s`; each segment has heading, discrete curvature, grade, banking, and independent left/right widths.

World-to-track uses nearest-segment projection. Positive lateral offset is left of increasing `s`. Boundaries use `world(s,+width_left)` and `world(s,-width_right)`. Polyline curvature is heading change divided by adjacent mean segment length. This is stable for adequately sampled tracks but is not a spline/clothoid representation.

Real-track work requires RTK/total-station or LiDAR centreline/boundary survey, elevation, banking, kerb and barrier measurement, with explicit uncertainty.
