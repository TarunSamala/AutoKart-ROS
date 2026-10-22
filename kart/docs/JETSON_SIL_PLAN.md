# Jetson SIL Plan

Jetson performance was not measured:

```text
JETSON PERFORMANCE: NOT_EXECUTED_EXTERNAL_DEPENDENCY
```

Preparation scripts:

```bash
./scripts/check_jetson_environment.sh
./scripts/deploy_jetson_sil.sh             # dry-run instructions only
./scripts/deploy_jetson_sil.sh --execute user@jetson-host
./scripts/profile_jetson_sil.sh
```

Qualification must record JetPack/L4T, power mode, clocks, CPU/GPU temperature,
EGL backend, RAM, throttling, per-process CPU/RSS, RTF windows, VCU/safety
misses, queues and sensor drops. The workstation CPU-affinity lists must not be
copied to Jetson.

No deployment or remote process was started in Phase 16. HIL and physical
actuator outputs remain disabled.
