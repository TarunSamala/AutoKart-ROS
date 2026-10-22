# Phase 7 Remediation Audit

Baseline: 31/31 tests passed. Git metadata is an empty invalid `.git` directory, so no user worktree status or commit is available.

Existing bypass: `kart_sim.scenario.run_scenario()` constructed `VehicleCommand` directly from scenario YAML and passed it to `DynamicFourWheelKart.step()`. That command drove steering, brake, and motor actuator chains without a VCU. `FourWheelKart` and `PlanarBicycleKart` retain direct commands only for explicit low-level `direct_test` physics/component testing.

Replacement policy: `control.mode: vcu` schedules a VCU at a configurable rate, builds typed feedback inputs from the dynamic kart, sends only VCU outputs to `DynamicFourWheelKart`, and refreshes the independent-safety pulse separately. `control.mode: direct_test` logs `VCU_BYPASS_ACTIVE`.
