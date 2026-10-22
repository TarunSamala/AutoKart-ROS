# Project Chrono integration

`kart_sim.backends.VehiclePhysicsBackend` provides a common reset/step/chassis,
wheel, diagnostics, and snapshot interface. `AnalyticalBackend` wraps the
validated dynamic four-wheel model unchanged. `chrono_multibody` is explicit:
requesting it when bindings are absent raises an error and never silently falls
back to the analytical model.
