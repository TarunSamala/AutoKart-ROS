# VCU Scenario Integration

In `control.mode: vcu`, the scenario runner creates a typed `MotionCommand`, invokes the VCU at a deterministic configurable period (default 250 Hz), passes `VcuInputs` built from dynamic actuator/vehicle feedback, and applies only `VcuOutputs.command` to the dynamic kart. Motor enable and contactor-open outputs are passed independently to the powertrain/battery boundary.

`direct_test` is retained exclusively for low-level tests and writes `VCU_BYPASS_ACTIVE` into telemetry. It is not evidence of integrated safety behavior.

```text
scenario command -> VCU validation/state/envelope -> VCU outputs -> dynamic actuators -> vehicle
                                           independent watchdog -> emergency override
```
