# Autonomy Robustness Status

The preserved baseline includes Pure Pursuit, Stanley, Follow-the-Gap,
longitudinal PID, emergency stop, deterministic fast laps, cone interfaces,
wet-surface faults, obstacles, and localization/perception contracts.

Phase 16 revalidated the Pure Pursuit full lap and controlled stop under the
native real-time follower. Existing unit and integration tests cover the
controller contracts and VCU routing.

The following claims are intentionally not made:

- rendered-sensor cone detection/classification success;
- camera glare or low-light perception qualification;
- recovery from real localization loss;
- dynamic pedestrian or moving-kart avoidance at real-time load;
- complete comparative lap qualification for every controller;
- native multi-lap Gazebo thermal robustness.

Ground-truth trajectory operation in the qualifying lap is labelled
`ground_truth_debug`; it is not relabelled as rendered perception. Completing
the rendered-sensor cone pipeline is the next autonomy qualification step.

| Test | Phase 16 result |
|---|---|
| A1 nominal lap | PASS: Pure Pursuit native lap and controlled stop |
| A2 cone track | PARTIAL: interfaces/ground-truth-debug only |
| A3 wet track | PASS internally; native rendered real-time run not executed |
| A4 obstacle | baseline contracts preserved; native dynamic matrix not executed |
| A5 degraded perception | noise/drop hooks preserved; rendered qualification not executed |
| A6 localization uncertainty | interfaces preserved; loss/recovery run not executed |
| A7 multi-lap thermal | 233-lap core soak passed; native full-stack run not executed |
