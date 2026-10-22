# Actuator Models

**Verdict: PROTOTYPE_TEST_REQUIRED.** All values are simulation-only provisional values.

`SteeringSystem` implements `J theta_ddot = Tmotor - Tload - Tc*tanh(omega/epsilon) - c*omega`; motor torque uses a provisional voltage-scaled linear torque-speed envelope. Shaft position passes through deterministic backlash and a compliant first-order linkage before road-wheel output. Actuator and linkage feedback remain distinct. Implemented faults: power loss, jam, linkage break. Tests: steering dynamic step/backlash and linkage-break integration.

`BrakeSystem` keeps commanded and actual stepper displacement separate. Actual movement is speed/voltage limited; cable slack and stiffness produce cable tension; pressure follows a rate-limited lumped relation `p=min(pmax,kp*x)` with leakage; disc torque is `p*A*mu*r`. Implemented faults: stepper stall, power loss, cable break, hydraulic leak, not releasing. `tests/unit/test_phase4_subsystems.py` verifies apply, release, and stall.

Not yet implemented: measured linkage tables, separate left/right linkage calibration, actuator command timeout, secondary sensor noise/delay, detailed stepper torque-speed curve, and brake-fade friction reduction.
