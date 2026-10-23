# Independent validation — 2026-10-22

Validator role: sensors workstream author; no planner/controller implementation
was authored or edited by this validator. Tested the actual isolated source
against ROS 2 Humble and newly built validation_ws autokart_msgs.

Initial independent execution: **80 passed in 1.93s**. A separate hand-written
probe (/tmp/autokart_independent_0809.py) found malformed string validity flags
accepted by both canonical node adapters. Controller accepted valid="false"
into a true envelope; planner accepted false strings in mission/coverage and
reached its geometry engine. Reported these defects to the owning developer;
no validator implementation edits were made. Source inspection also found
coverage expiry type was unchecked; owning developer corrected that parser.

After owning developer corrections: **96 passed in 2.86s**, no failures/skips.
Independent fault probe repeated: controller rejects malformed validity with
PLANNING_STATUS_MALFORMED; planner rejects with
MISSION_MISSING_INVALID_OR_EXPIRED and does not reach geometry evaluation.
Controller wrong-provenance and expired-odometry vetoes PASS; planner mismatched
odometry sample stamp veto PASS. No physical endpoints or actuation were used.

Reproduction from worktree/autokart_ros_nodes:
```
source /opt/ros/humble/setup.bash
source ../../validation_ws/install/local_setup.bash
export PYTHONPATH="$PWD/autokart_planner:$PWD/autokart_controller:$PYTHONPATH"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q autokart_planner/test autokart_controller/test --import-mode=importlib
```

Disposition: PASS for the executed software tests and concrete fault probes.
This does not establish physical limits, live camera obstacle completeness,
firmware/session correctness, actual steering/braking or vehicle release.
Live autonomous motion remains NO-GO under current unqualified inputs.
