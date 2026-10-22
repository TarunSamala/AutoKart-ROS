# Cone System

`kart_sim.cones` defines ground-truth cones with:

- unique ID, world position and yaw;
- blue, yellow, small/large orange, unknown, fallen and moved classes;
- collision radius, visibility and LiDAR reflectivity;
- displacement and knockdown state.

`boundary_cones` generates deterministic blue-left/yellow-right layouts from a
track. `track export` writes a JSON ground-truth layout. Generated ground truth
is marked `ground_truth_only` and is never substituted for operational
perception.

`ConePerception` produces a distinct noisy detection array with configurable
position/range error, misclassification, misses, false positives, range/FOV
limits and deterministic seed. Occlusion and camera appearance are renderer
responsibilities; the headless model provides visibility and reflectivity
inputs.

Moved and fallen cones remain identifiable by their source ID and state. Race
control accounts for cone penalties separately from perception confidence.

Operational topics:

```text
/perception/cones
/debug/ground_truth/cones     # disabled in production mode
```

The SDF generator gives each cone inertial, collision, visual and class color
geometry. Current native static worlds use generated/preview assets; rendered
reflectivity calibration is `MEASUREMENT REQUIRED`.

