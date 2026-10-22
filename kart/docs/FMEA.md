# Phase 4 FMEA (Prototype)

| Function | Failure | Detection now | Safe response available now | Severity | Test |
| --- | --- | --- | --- | --- | --- |
| Steering | road-wheel jam | actuator/linkage following error telemetry | none; integration layer exposes fault | Critical | focused actuator test |
| Steering | linkage break | shaft/linkage disagreement | none; fault exposed | Critical | dynamic integration |
| Brake | stepper stall | commanded/actual/missed-step mismatch | none; fault exposed | Critical | unit test |
| Brake | cable break | zero tension/pressure | none; fault exposed | Critical | unit model path |
| Brake | fails to release | residual pressure | propulsion inhibit deferred | High | unit model path |
| Motor | torque stuck | controller fault telemetry | traction isolation deferred | Critical | model path |
| Drivetrain | chain break | zero chain torque | controlled stop deferred | High | unit test |
| Battery | undervoltage/BMS shutdown | terminal voltage/BMS state | propulsion disabled in dynamic wrapper | High | electrical unit test |
| Contactor | welded | electrical state model | secondary isolation deferred | Critical | not yet automated |

Phase 7 simulation responses and tests now exist for these rows; real-hardware
evidence remains open.

## Phase 5 additions

| Function | Failure | Detection | Mitigation | Severity | Test |
| --- | --- | --- | --- | --- | --- |
| Projection | wrong nearby segment | projection distance/segment trace | previous-s continuity (pending) | Critical | partial |
| Surface lookup | overlap ambiguity | priority ordering | deterministic priority | Critical | track unit |
| Boundary | invalid width | construction validation | reject track | High | track unit |
| Curvature | coarse polyline error | reference-circle test | resample/spline required | High | track unit |

| Brake transmission | Cable break | Actuator travel without cable force/pressure | Brake response unavailable | Actuator-position versus pressure mismatch (`BRAKE-003`) | Remove propulsion, emergency brake/isolation request, latch | Critical | integrated cable-break test |
| Traction torque control | Positive torque after zero command | Controller/gate-driver/command-latch fault | Uncontrolled propulsion | Requested/actual torque mismatch (`MOTOR-003`) | Disable motor, open contactor request, emergency brake, latch | Critical | integrated stuck-torque test |

## Phase 7 safety-control update

| Function | Failure | Cause | Vehicle effect | Detection | Safe response | Severity | Test |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Motion command | Stale/corrupt command | Compute or communication fault | Retained unsafe demand | age, expiry, CRC, sequence, source | reject and controlled stop | Critical | command envelope / timeout |
| VCU | Loop or safety-task freeze | Deadlock/scheduler fault | Outputs not renewed | independent pulse timeout | isolate traction and brake fallback | Critical | VCU/safety freeze |
| Steering | Jam/linkage break/runaway | mechanical/controller fault | path-control loss | target/shaft/linkage/wheel residuals | remove propulsion; controlled/emergency stop | Critical | steering campaign |
| Brake transmission | Cable break | fatigue/termination/overload | pressure and braking absent | actuator motion versus pressure/deceleration | remove propulsion, emergency assessment, lockout | Critical | cable-break scenario |
| Brake transmission | Partial force transfer | damaged strands/clamp slip | longer stop | pressure-position residual | degraded limit then stop | Critical | partial-cable scenario |
| Motor | Positive torque persists | gate/controller/latch fault | uncontrolled propulsion | final request versus torque/current/response | disable, contactor, emergency brake | Critical | stuck-torque scenario |
| Contactor | Welded | contact damage | isolation unavailable | command/feedback mismatch | sustained brake and lockout | Critical | combined welded scenario |
| Safety envelope | Stale/short clear path | sensing/compute degradation | insufficient stopping distance | age/distance/uncertainty inputs | reduce speed rapidly | Critical | envelope tests |
| Reset | Unsafe re-entry | incomplete repair or stale request | motion after critical fault | centralized reset guard matrix | reject with reasons; self-test path | Critical | reset matrix |

All critical rows above link to executed automated simulation tests in the Phase
7 acceptance checklist. Physical FMEA closure is not claimed.
# ROS 2/SIL interface additions

| Function | Failure | Effect | Detection | Mitigation | Severity | Verification |
| --- | --- | --- | --- | --- | --- | --- |
| ROS command transport | stale reliable message | outdated motion demand | VCU timestamp/expiry check | reject and controlled stop | Critical | `test_ros2_sil.py` |
| Time bridge | delivery time used as capture time | invalid data age/fusion | sensor-envelope validation | preserve measurement timestamp | High | `test_autonomy.py` |
| Frame bridge | cyclic/duplicate frame | wrong pose transform | `FrameTree.validate` | reject configuration | High | `test_autonomy.py` |
