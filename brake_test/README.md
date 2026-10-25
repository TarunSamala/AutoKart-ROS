# AutoKart Phase 1 brake test

This folder is the operator entry point for the complete, fail-closed braking
qualification workflow:

`HP60C perception -> mapping -> BRAKING_TEST planner -> controller candidate -> safety supervisor -> VCU bridge -> measured braking -> integrated stop trigger`

## Start

```bash
brake_test/run.sh
```

Show the current non-actuating readiness snapshot:

```bash
brake_test/run.sh --status
```

Run one stage:

```bash
brake_test/run.sh --stage 0
```

Use `brake_test/run.sh --help` for the complete stage list.

## Safety boundary

The workflow does not flash firmware, set verification flags, change
commissioning values, request ARM, or release the brake automatically. Stage 5
is the direct physical braking measurement and requires the ROS VCU bridge to
be stopped. Stage 8 observes the integrated perception-to-VCU stop chain with
the wheels off the ground. Hardware motion remains blocked until all measured
gates pass and an operator reviews the evidence.

Evidence is stored under `results/braking_session_<timestamp>/`.

## Portable one-click blue-cone demonstrator

The proven HP60C + ESP32 blue-cone brake dashboard can be packaged for Ubuntu
22.04 x86_64 and Ubuntu 22.04 aarch64 (Raspberry Pi 4B):

```bash
bash brake_test/portable/build_package.sh
```

Extract the resulting `dist/AutoKart-Brake-Test-Ubuntu22.04-Pi4B.tar.gz` and
double-click `AutoKart Brake Test.desktop`. The first launch installs the
runtime and HP60C USB rule; subsequent launches open the UI directly. See
`brake_test/portable/PACKAGE_README.md` for the hardware and safety preflight.

For the Arch development laptop, build the Distrobox-backed one-click package:

```bash
bash brake_test/portable/build_arch_package.sh
```

## Implementation locations

- Orchestrator: `brake_test/braking_test_session.sh`
- Live graph: `kart/ros2_ws/src/autokart_bringup/launch/phase1_vehicle.launch.py`
- Direct braking measurement: `tools/braking_test.py`
- Integrated stop observer: `tools/stop_trigger_test.py`
- Final gate: `tools/readiness_report.py`
- Detailed runbook: `docs/TEST_RUNBOOK.md`
