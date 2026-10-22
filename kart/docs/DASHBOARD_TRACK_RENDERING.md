# Dashboard track rendering

`dashboard_track_payload()` samples the selected `Track` by true arc length.
World boundaries come from Phase 5 `world_from_track`; no arbitrary oval is
generated. Surface regions use their configured `s` and signed-lateral bounds.
Wet regions are visually distinct, but the color is never a friction authority.

`dashboard_research_lap.yaml` selects `research_circle.yaml`. The simulator
projects the body and four wheel contacts into that same track and records the
result. In the executed visual test the kart traverses increasing track distance
and remains within 1.2 m of the centreline while encountering both configured
dry and wet materials.

Track-survey fidelity remains provisional. `research_circle.yaml` is labelled
simulation-only and is not a real surveyed circuit.

