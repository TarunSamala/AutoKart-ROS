# Physics Models

## Phase 2 planar bicycle model

- **Purpose:** provide a deterministic, bounded baseline for straight-line and initial corner response.
- **Equations:** `m(du/dt-rv)=ΣFx`, `m(dv/dt+ru)=ΣFy`, `Iz dr/dt=ΣMz` with world-frame pose integration described in `COORDINATE_SYSTEMS.md`.
- **Variables/units:** body speeds `u,v` (m/s), yaw rate `r` (rad/s), forces (N), mass (kg), yaw inertia (kg m²), steering (rad).
- **Assumptions:** rigid planar body; static axle loads; linear lateral stiffness with a friction cap; direct force commands.
- **Valid range:** numerical prototype only; no physical range is claimed before tests against measurements.
- **Limitations:** no individual wheels, combined slip, load transfer, axle torsion, suspension/chassis, or actuator state.
- **Implementation:** `kart_sim/vehicle.py`.
- **Verification:** `tests/physics_validation/test_planar_baseline.py`.
- **Future:** replace axle forces with four-wheel tire interface in Phase 3.

## Phase 3 four-wheel analytical backend

- **Purpose:** expose individual contact-patch telemetry and the first approximation of a solid rear axle, load transfer, and chassis twist.
- **Equations:** each wheel uses `kappa=(omega*r-vx)/max(|vx|,0.05)` and `alpha=atan2(vy,max(|vx|,0.05))`. Raw linear force is capped by `sqrt(Fx²+Fy²) <= mu*Fz`. Wheel velocities include `vx=u-r*yw`, `vy=v+r*xw`; rigid rear angular speed is shared by both rear wheels.
- **Variables/units:** load and force N, tire radius m, angular speed rad/s, slip ratio dimensionless, slip angle rad, chassis twist rad.
- **Assumptions:** quasi-static load transfer from preceding-step accelerations; no suspension kinematics; exact Ackermann geometry; perfect rigid axle; constrained axle speed with a lumped inertia and bearing loss.
- **Valid range:** prototype numerical behaviour only. No physical range is claimed.
- **Limitations:** the configuration includes future axle/chassis compliance fields but the current backend implements only the rigid-axle variant; front wheel rotational dynamics and brake distribution are simplified; no tire temperature/camber/load-sensitivity law; no reference-data comparison.
- **Implementation:** `kart_sim/four_wheel.py`, `kart_sim/tires.py`.
- **Verification:** `tests/unit/test_tires.py`, `tests/physics_validation/test_four_wheel_baseline.py`, and `configs/scenarios/four_wheel_skidpad.yaml`.
- **Future:** add wheel rotational states, compliant axle, tire model variants, and convergence/reference checks before treating outputs as engineering predictions.
