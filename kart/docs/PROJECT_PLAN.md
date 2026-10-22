# Project Plan

## Technology decision

**Decision: Python 3.10 headless core with Pydantic/YAML and pytest; optional adapters remain separate. Verdict: APPROVED_WITH_CONDITIONS.**

| Evaluation item | Decision record |
| --- | --- |
| Purpose | Make a deterministic, inspectable foundation executable in the detected environment. |
| Problem solved | Enables physics, safety logic, test automation, and schema validation without GPU, Docker, Chrono, or CARLA. |
| Assumptions | Python, PyYAML, Pydantic, pytest, NumPy, and Matplotlib are locally available. |
| Advantages | Small dependency footprint; rapid testability; stable headless operation. |
| Limitations | No real-time guarantee, multibody contact, rendered sensor images, or HIL timing fidelity. |
| Failure modes | Numerical instability, missing dependency, model misuse as physical validation. |
| Detection | finite-state checks, timestep validation, tests, versioned configuration and telemetry. |
| Safe response | Stop run on invalid state; do not authorize vehicle deployment. |
| Alternatives | C++20 core, Project Chrono, CARLA/Unreal, ROS-first architecture. |
| Verification | automated baseline tests and later comparison with measurements/backends. |

## Sequential delivery plan

1. **Phase 0 — audit and engineering records:** complete.
2. **Phase 1 — foundation:** package, config validation, scheduler, CLI, logging, smoke test: complete baseline.
3. **Phase 2 — basic mathematical kart:** planar bicycle dynamics and core tests: complete baseline; plotting is pending.
4. **Phase 3 — four-wheel dynamics:** individual wheel/contact/tire interfaces, loads, perfect rigid solid rear axle, and a quasi-static chassis-twist proxy: prototype complete; compliance and validation remain pending.
5. **Phase 4 — actuators and electrical:** steering/brake chains, BLDC, battery and thermal models.
6. **Phases 5–10:** track/surfaces, sensors, VCU/safety, ROS, dashboard, fault benchmark.
7. **Phases 11–15:** optional Chrono/CARLA adapters, SIL/HIL, calibration, research demonstration.

Each phase requires an executable test gate, updated assumptions/status/limitations, and a Git checkpoint if a writable Git repository is supplied. The workspace currently contains an empty non-repository `.git` directory, so a checkpoint cannot be made without replacing user-owned repository metadata.
