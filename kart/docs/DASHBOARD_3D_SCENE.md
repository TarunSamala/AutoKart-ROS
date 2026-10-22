# Dashboard 3D scene

The scene uses local JavaScript perspective projection with a world-space
camera, ENU coordinates, and +z upward. It is a small dependency-free 3D
engineering renderer, not a photorealistic engine.

Rendered geometry includes the Phase 5 surface ribbon, centreline, independent
boundaries, start/finish line, material regions, corner/apex markers, actual
trajectory, fault locations, and the dimensioned procedural kart. Kart body
position/orientation comes only from normalized simulation records. Front
wheels use measured left/right steering angles. Wheel spoke phase integrates
measured angular speed over displayed measurement time.

Camera modes are orbit, chase, top-down, kart-front, driver-eye, corner-fixed,
and free. Mouse dragging or zooming affects only the camera.

Selectable overlays include trajectory, boundaries, surfaces, footprint,
forces, velocity, stopping envelope, corners, fault locations, and CG.

