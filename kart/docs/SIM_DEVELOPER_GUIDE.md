# AutoKart Sim Studio Developer Guide

Keep the deterministic Python core independent of ROS and Gazebo imports.
External packages translate messages and state but do not own safety or plant
logic.

Change flow:

```text
schema/config -> transport-neutral implementation -> unit test
              -> integration/safety/determinism test
              -> optional native adapter -> native evidence/status
```

Rules:

- acquire one `AuthorityLease` before adding an integrator;
- add high-level sources to VCU authorization explicitly;
- include measurement, delivery and reset-epoch semantics;
- never expose dashboard or visual-follower actuator methods;
- use standard ROS messages when semantically correct;
- namespace every multi-kart topic/frame;
- mark provisional physical values `SIMULATION USE ONLY — MEASUREMENT REQUIRED`;
- report missing native execution, never fabricate it.

Track changes must pass all converters. Robot changes must pass Xacro,
`check_urdf`, dependency-free validation and native `gz sdf -k` when possible.
ROS messages require `./scripts/build_ros2.sh` and native smoke.

The existing 107-test baseline is regression authority. New tests live in
unit, integration, safety and determinism groups. Do not move established plant
modules merely to imitate the target directory sketch.

The empty `.git` directory is invalid metadata; see `docs/GIT_RECOVERY.md`.
Use filesystem/test artifacts until repository history is repaired.

