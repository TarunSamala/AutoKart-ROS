PHASE:
Phase 7 — VCU State Machine, Command Validation, Watchdogs,
Plausibility, Safety Envelope, Controlled Stop, and Emergency Stop

STATUS:
COMPLETE

VERDICT:
APPROVED_WITH_CONDITIONS

PROMPT PURPOSE:
Complete VCU-to-actuator scenario integration and Phase 7 acceptance.

BASELINE TEST STATUS:
- Tests before modification: 31
- Passed: 31
- Failed: 0
- Skipped: 0

EXISTING BYPASSES FOUND:
- Integrated scenario YAML was converted directly to `VehicleCommand` and
  passed to the dynamic actuator wrapper before remediation.
- Direct low-level operation remains available only as explicitly logged
  `direct_test`; autonomous/safety use is rejected.

IMPLEMENTED:
- Scenario control modes: schema-checked `direct_test` and `vcu`.
- VCU scheduler integration: configurable deterministic VCU sample-and-hold
  plus independent physics-rate safety task.
- VCU input interface: typed feedback for vehicle/wheels, steering, brake,
  motor/current/chain, contactor, friction and timestamps.
- VCU output interface: state, final steering/brake/drive, enable, contactor,
  stop phase, faults, speed limit and reason.
- Direct bypass prevention: integrated direct writes and autonomous
  `direct_test` are rejected.
- State-to-actuator policies: every non-driving/critical state produces defined
  physical outputs.
- Command validation: source, CRC-16/CCITT-FALSE, finiteness, timestamp,
  expiry, 8-bit counter, wrap, gap and state compatibility.
- Command timeout: stale demand removes torque and drives the brake plant.
- Steering plausibility: actuator jam, linkage break, road-wheel jam, runaway,
  dual-sensor faults, power loss, rate/limit and high-speed response.
- Brake plausibility: actuator motion, missed steps, cable force, pressure,
  torque, release, fade, sensor distinction and vehicle response.
- Traction plausibility: requested/final/actual torque, current, enable,
  contactor, motor/chain/wheels and vehicle response.
- Safety-envelope command limiting: clear distance, curvature, delay,
  uncertainty, brake capability, friction, state and hysteretic recovery.
- Controlled-stop physical integration: eight phases, physical completion
  gates, rolling-restart revocation and timeout escalation.
- Emergency-stop physical integration: motor disable, contactor request,
  emergency brake, held steering, torque/pressure/motion monitoring and latch.
- Independent safety override: separate state latch, VCU/safety-task freeze
  response, motor isolation and brake fallback.
- Reset/latching: centralized reasoned guards and
  `SELF_TEST → SAFE → READY`; no command reuse or automatic autonomy.
- Safety invariants: no positive propulsion in controlled/emergency/lockout
  state; violations are critical DTCs.
- Fault management: structured retained history, DTC registry and bounded
  observable snapshots.

FILES CREATED:
- `kart_sim/communication.py`
- `kart_sim/fault_manager.py`
- `kart_sim/watchdog.py`
- `kart_sim/safety_config.py`
- `kart_sim/safety_health.py`
- `kart_sim/safety_diagnostics.py`
- `kart_sim/safety_metrics.py`
- Phase 7 scenarios under `configs/scenarios/vcu_*.yaml`
- Phase 7 unit/integration campaign tests
- `docs/phase_reports/PHASE_07_ACCEPTANCE_CHECKLIST.yaml`
- `docs/ACTUATOR_PLAUSIBILITY.md`
- `docs/FAULT_MANAGEMENT.md`
- `docs/INDEPENDENT_SAFETY_LAYER.md`
- `docs/VCU_STATE_MACHINE.md`
- `docs/COMMAND_VALIDATION.md`
- `docs/WATCHDOGS.md`

FILES MODIFIED:
- `kart_sim/vcu.py`
- `kart_sim/scenario.py`
- `kart_sim/dynamic_kart.py`
- `kart_sim/actuators.py`
- `kart_sim/powertrain.py`
- `kart_sim/electrical.py`
- `kart_sim/four_wheel.py`
- `kart_sim/cli.py`
- Safety architecture, FMEA, assumptions, status and requirements documents.

TESTS RUN:
- `./scripts/test.sh`
- `PYTHONPATH=. python3 -m kart_sim.cli test-model vcu`
- `PYTHONPATH=. python3 -m kart_sim.cli test-model controlled-stop`
- `PYTHONPATH=. python3 -m kart_sim.cli test-model emergency-stop`
- `PYTHONPATH=. python3 -m kart_sim.cli test-model safety-envelope`
- `PYTHONPATH=. python3 -m kart_sim.cli safety validate configs/safety/prototype_safety.yaml`
- `PYTHONPATH=. python3 -m kart_sim.cli safety campaign --output-dir /tmp/phase7-safety-campaign`
- `PYTHONPATH=. python3 -m kart_sim.cli report /tmp/phase7-evidence/vcu_command_timeout --safety`

TEST RESULTS:
- Full locally executable regression: 98 passed, 0 failed, 0 skipped.
- Focused VCU: 10 passed.
- Focused controlled stop: 9 passed.
- Focused emergency stop: 5 passed.
- Focused safety envelope: 4 passed.
- Critical smoke campaign: 3/3 cases passed, 3/3 catalog faults covered.
- Mandatory software checklist: 79/79 PASS.
- Hardware-only checklist: 3 `NOT_EXECUTED_EXTERNAL_DEPENDENCY`.

PHYSICAL SIMULATION EVIDENCE:
- Nominal VCU drive: final VCU output reaches dynamic steering, brake and motor.
- Command timeout: detected at 0.205 s; torque zero at 0.215 s; brake response
  at 0.213 s; measured stop at 0.880 s; 0.821 m after detection.
- Steering jam: absent shaft/road motion produces `STEER-001`, drive zero and
  physical pressure rise.
- Broken linkage: shaft motion with absent linkage response produces
  `STEER-004` and controlled stop.
- Brake cable break: actuator travel continues; cable force, pressure and brake
  torque collapse; `BRAKE-003` and stopping-capability loss escalate.
- Brake release failure: residual pressure produces lockout and propulsion
  inhibit.
- Motor stuck torque: `MOTOR-003` at 0.253 s; isolation response; actual torque
  below threshold at 0.320 s; emergency pressure response at 0.261 s.
- VCU freeze: independent watchdog latch overrides the frozen output, isolates
  motor and physically applies the brake.
- Physical E-stop simulation: during acceleration, action at 0.251 s, torque
  zero at 0.263 s, pressure response at 0.259 s and stop at 0.696 s.
- Wet-surface stop: envelope limit falls and stopping distance increases; at
  the 1.5 s horizon speed remains 0.402 m/s at x=3.891 m, honestly not complete.

WORST-CASE RESULTS:
- Maximum command-timeout distance: 0.821 m after detection in the executed
  command-timeout case.
- Maximum fault-detection time: 0.253 s from scenario start among the reported
  critical reference cases; physical-observable onset is model-dependent.
- Maximum torque-removal time: 0.067 s after stuck-torque detection.
- Maximum brake-response time: 0.008 s after reported safety action.
- Maximum stopping distance: 0.821 m after detection among cases that completed
  within their horizon; wet/cable-break cases did not complete and are not
  misreported as successful stops.
- Invariant violations: 0 in full tests and critical campaign.

REGRESSION RESULTS:
- Previous tests: 31/31 baseline passed.
- New/current tests: 98/98 passed.
- Regressions: none.

ASSUMPTIONS:
- All safety thresholds, friction, brake pressure, torque and timing values are
  provisional simulation values.
- Cable force and brake torque are internal diagnostic signals unless physical
  sensors are installed; operational cable diagnosis uses actuator encoder,
  hydraulic pressure and motion evidence.
- Independent safety is logically separate in simulation, not proven
  electrically independent hardware.

REAL-HARDWARE MEASUREMENTS REQUIRED:
- VCU/safety-task WCET, jitter, reset and watchdog timing.
- Steering onset/tracking/jam current/linkage and power-loss response.
- Brake actuator/pressure/deceleration/release/hold/fade and split-friction
  stopping performance.
- Motor disable/torque decay, contactor opening/weld feedback and BMS behavior.
- Physical E-stop, independent watchdog, power defaults and sensor failure
  timing.

SAFETY RISKS:
- Provisional thresholds may be unsafe on the physical kart.
- Reduced-order tire, brake, steering and power models omit hardware failure
  detail.
- Simulator agreement cannot substitute for independent bench/vehicle tests.

KNOWN LIMITATIONS:
- No safety certification or production VCU firmware claim.
- No physical hardware was energized or tested.
- Phase 5/6 model limitations constrain environment/sensor fidelity.

SAFETY-CRITICAL BLOCKERS:
- Software/simulation Phase 7 gate: none.
- Real-vehicle deployment: physical E-stop, actuator, isolation, watchdog, HIL
  and progressive vehicle-test evidence remain absent.

REPOSITORY COMMIT:
- NOT AVAILABLE — INVALID GIT METADATA

READY FOR PHASE 8:
YES — for software research work only; not for vehicle deployment.

NEXT PROMPT FILE:
autokart_codex_prompts/08_PHASE_ROS2_AUTONOMY_SIL.txt
