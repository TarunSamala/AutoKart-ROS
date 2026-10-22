# Actuator Plausibility

All thresholds in this document are **SIMULATION USE ONLY** and require real
bench characterisation.

## Steering

The monitor compares the requested and limited road-wheel target with actuator
shaft angle, linkage angle, left/right wheel angle, current, yaw response and
lateral response. Absence of shaft motion is `STEER-001`; shaft motion without
linkage motion is `STEER-004`; primary/secondary or linkage/wheel disagreement
is `STEER-002`; uncommanded wheel motion is `STEER-003`. High-speed loss uses
the emergency policy; lower-speed loss removes propulsion and controls the
stop. The simulation exposes shaft and linkage signals as internal diagnostic
signals. A physical VCU can use them only where corresponding encoders exist.

## Brake

The observable planned chain is actuator encoder → hydraulic pressure sensor →
wheel speed/IMU deceleration. Cable force and brake torque are simulator
internal diagnostic signals unless the physical kart adds a load sensor.

- No actuator motion after a brake request: `BRAKE-001`.
- Actuator moves but pressure does not rise: `BRAKE-003`, mechanical
  transmission or hydraulic-input failure.
- Pressure exists but position feedback does not: `BRAKE-SENSOR-002`.
- Physical torque/deceleration exists while pressure reads zero:
  `BRAKE-SENSOR-001`.
- Pressure exists but brake torque is deficient: `BRAKE-004`.
- Partial pressure-versus-position response: `BRAKE-005`.
- Pressure persists after release: `BRAKE-RELEASE-001`.

Low deceleration alone is not classified as cable break. Surface friction,
grade, wheel slip and measured pressure/torque evidence are considered. A
hydraulic pressure sensor is strongly recommended because actuator position
alone cannot establish transmitted braking.

## Traction

The monitor compares high-level demand, safety-limited command, enable,
contactor command/feedback, actual torque, current, motor/axle/wheel speeds,
chain torque and vehicle acceleration. `MOTOR-003` requires positive actual
torque after a zero final request for the removal timeout. `POWER-004` records
failed contactor isolation feedback. Current alone cannot diagnose stuck
torque.

## Verification and limits

Executed tests are mapped in
`docs/phase_reports/PHASE_07_ACCEPTANCE_CHECKLIST.yaml`. Real command-to-motion,
jam-current, pressure-rise, torque-decay, contactor, and stopping thresholds
remain unmeasured. Simulation success is not real-vehicle evidence.
