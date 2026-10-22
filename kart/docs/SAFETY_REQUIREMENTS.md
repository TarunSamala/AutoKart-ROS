# Safety Requirements (Simulation Prototype)

| ID | Requirement | Hazard/rationale | Implementation | Verification | Software status | Open assumption |
| --- | --- | --- | --- | --- | --- | --- |
| `SAFE-CMD-001` | Reject stale, future, non-finite, unauthorized, sequence-invalid or CRC-invalid commands before actuator effect. | Prevent retained/corrupt motion demand. | `communication.py`, `vcu.py` | command-envelope unit tests and command-timeout scenario | PASS | Real transport timing unmeasured. |
| `SAFE-STATE-001` | Autonomous entry requires all explicit guards; invalid transitions are atomic rejects. | Prevent unsafe enable. | `vcu.py` | state-guard tests | PASS | Guard thresholds provisional. |
| `SAFE-WDG-001` | A frozen VCU shall be overridden by an independently executing stop latch. | Prevent retained outputs after software freeze. | `IndependentSafety`, scenario scheduler | VCU/safety-task freeze scenarios | PASS | Hardware independence not executed. |
| `SAFE-STEER-001` | Steering command/shaft/linkage/wheel mismatch shall remove propulsion and stop. | Loss or runaway of path control. | steering plausibility monitor | steering integrated campaign | PASS | Real sensor locations/timing unmeasured. |
| `SAFE-BRAKE-001` | Actuator motion without pressure response shall be diagnosed and stopping capability reassessed. | Broken cable/leak can remove braking. | brake feedback-chain monitor | cable/leak/sensor distinction campaigns | PASS | Pressure sensor strongly recommended. |
| `SAFE-TRACTION-001` | Positive actual torque after zero request shall cause enable removal, isolation and emergency braking. | Uncontrolled propulsion. | traction monitor and independent override | stuck torque/welded contactor campaigns | PASS | Real torque/current/disable timing unmeasured. |
| `SAFE-ESTOP-001` | Physical E-stop shall outrank all normal commands and remain latched. | Immediate independent hazard response. | `IndependentSafety` | at-rest, acceleration, corner and stuck-torque tests | PASS | Physical loop not tested. |
| `SAFE-STOP-001` | A stop is complete only with fresh redundant motion evidence, zero torque/current and brake hold. | Prevent false stopped state. | `ControlledStopManager` | CS unit/integration tests | PASS | Physical threshold calibration required. |
| `SAFE-RESET-001` | Critical reset shall reject unsafe conditions and pass SELF_TEST → SAFE → READY without autonomous restart. | Prevent unsafe re-entry. | centralized reset validator | reset guard matrix | PASS | Real authorization mechanism pending. |
| `SAFE-ENV-001` | Clear distance, curvature, latency, uncertainty, friction and brake capability shall reduce actual speed permission. | Maintain stopping/cornering margin. | `SafetyEnvelope`, VCU limiter | envelope/wet tests | PASS | Guaranteed deceleration is provisional. |

These PASS results are software/simulation evidence only. They do not establish
real-vehicle safety or compliance.
