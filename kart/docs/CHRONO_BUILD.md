# Chrono build

Project Chrono is optional. This environment has no `pychrono`/`chrono` Python
binding. Install a compatible official Chrono build and bindings, then implement
and validate the native adapter behind `ChronoMultibodyBackend`. CI without it
runs the analytical backend; an explicit Chrono request must fail clearly.
