# Independent validation — 2026-10-22

Role: independent VALIDATOR for node autokart_mission. No implementation edits.
Software fixtures only; no camera/serial/IMU physical acquisition or actuation.

Command from worktree/autokart_ros_nodes:

```bash
source /opt/ros/humble/setup.bash
source ../../validation_ws/install/local_setup.bash
PYTHONPATH="$PWD/autokart_mission:$PYTHONPATH" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=93 python3 -m pytest -q autokart_mission/test -rs
```

Exit code: 0

```text
.............................                                            [100%]
29 passed in 1.75s

```

Result: PASS (see skip details if any; skipped checks are NOT EXECUTED).
Physical qualification and vehicle readiness: NOT EXECUTED / NO-GO.
