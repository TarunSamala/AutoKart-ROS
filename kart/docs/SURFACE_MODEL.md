# Surface Model

Surface lookup returns a named material with provisional longitudinal/lateral friction and rolling resistance. Track-coordinate regions use explicit `s` and lateral bounds; highest priority wins deterministically, then ID breaks ties. Each lookup is independent, enabling future per-wheel split-friction integration. Geometry currently exposes banking and height; these values are not yet consumed by vehicle tire/load calculations.
