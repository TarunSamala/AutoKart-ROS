# Codex developer report — 2026-10-22

Role: DEVELOPER only. Independent review and validation pending; no self approval.
Scope: nodes 08 and 09 in isolated takeover worktree. Original library and symlink
algorithm dependencies were read only. No firmware, device open, flash or actuation.

Implemented canonical /mission/state -> /planning/status/path/profile/stop_reason ->
/control/request/status contracts while retaining legacy compatibility topics.
Mission path selection remains upstream. Explicit matched obstacle coverage and
odometry status, sample timestamps, frames, source and expiry gates fail closed.
Planner adds continuous conservative full-body disk sweep, first-contact clearance,
curvature steering feasibility, terminal stop, latency-aware speed envelope and
acceleration/braking passes. Controller defaults feedback invalid, computes 50 Hz
expiring requests, rejects stale feedback/status and reports geometric tracking
errors and actual steering tracking. Yaw-rate error explicitly unavailable.
Optional reused adapters call existing curvature/braking and Pure Pursuit classes;
selected controller backend import checks reject stale/wrong cached source trees.

Commands executed from worktree/autokart_ros_nodes:

```bash
python3 -m compileall -q autokart_planner/autokart_planner autokart_controller/autokart_controller
# PASS
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q autokart_planner/test/test_planner_core.py autokart_planner/test/test_algorithms.py autokart_controller/test/test_controller_core.py autokart_controller/test/test_pursuit_backends.py
# PASS: 71 passed in 0.76s (before final ROS-status regression additions)
source /opt/ros/humble/setup.bash
source ../../validation_ws/install/local_setup.bash
PYTHONPATH="$PWD/autokart_planner:$PWD/autokart_controller:$PYTHONPATH" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=88 python3 -m pytest -q autokart_planner/test autokart_controller/test
# PASS: 80 passed in 1.54s, zero skips
colcon build --base-paths autokart_planner autokart_controller --build-base /tmp/autokart_planning_build_20261022 --install-base /tmp/autokart_planning_install_20261022 --packages-select autokart_planner autokart_controller --event-handlers console_direct+
# PASS: 2 packages finished [2.63s]; setuptools tests_require warning subsequently removed.
./autokart_planner/export_node.sh ../../evidence/exports
./autokart_controller/export_node.sh ../../evidence/exports
# PASS: both source+interface tarballs created. Root final packaging should refresh after reports/review.
```

Environment findings: unsourced system pytest auto-load fails in host anyio plugin
(ModuleNotFoundError _pytest.scope), resolved through explicit plugin-autoload
isolation. Historical kart/ros2_ws install lacks MissionDebug; fresh isolated
validation_ws interface build is mandatory. No dependency substitution performed.

Tests include actual ROS planner-to-controller pub/sub followed by expired STOP,
missing feedback, missing obstacle coverage, invalid fresh odometry status,
stale estimator status, malformed mission, persistent fault statuses, terminal and
speed inequalities, swept corner collision and curvature feasibility.

Limitations: conservative enclosing-disk geometry can reject feasible narrow paths;
no jerk-limited profile; no rectangular exact swept volume claim; no measured
physical tracking, calibration, latency or braking proof. Obstacle extraction must
supply physical extent+uncertainty and qualified corridor coverage. Default live
parameters fail closed. output_inhibited=true and vehicle_authorized=false always.
Vehicle physical readiness remains NO-GO. Review and validation belong to other roles.


## Review repair addendum

Fixed independently reported malformed truthy boolean admission in mission,
planning and coverage JSON; parsers now require actual booleans and the gates
require `is True`. Coverage rejects non-integer expiry and wrong source/frame.
Both nodes enforce the configured odometry source age at evaluation time, so
recent receipt cannot extend a delayed pose's life. Planner core now explicitly
rejects malformed point arity/types and zero-length segments; max curvature must
be finite positive (default 0 means unqualified).

Relevant regression rerun (fresh ROS/interface environment as above):

```bash
PYTHONPATH="$PWD/autokart_planner:$PWD/autokart_controller:$PYTHONPATH" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=88 python3 -m pytest -q autokart_planner/test/test_planner_core.py autokart_planner/test/test_node_integration.py
# PASS: 40 passed in 1.32s
PYTHONPATH="$PWD/autokart_planner:$PWD/autokart_controller:$PYTHONPATH" python3 /tmp/autokart_independent_0809.py
# Both invalid_boolean_rejected=True; controller wrong-source and expired-odom,
# planner mismatched odom stamp veto PASS; geometry engine not reached on bad flags.
```

Sensor developer acting as independent validator has been asked to rerun its
probe and full suites. This developer addendum is not independent approval.


## Integration closure addendum

Production LIVE/LIVE_PHYSICAL rejects builtin algorithm/pursuit backends explicitly.
SIMULATED and SYNTHETIC estimator statuses normalize only within the nonphysical
simulation regime. Planner now requires capture-matched `/mapping/status`, and
includes every `geometry.observation_cones_odom` center as a footprint collision
obstacle. Radius and positional uncertainty are explicit parameters (defaults
unqualified); LIVE requires separate cone-footprint verification. Finite center,
strict validity, frame, source, stamp and expiry checks apply to the companion.

Relevant node integration tests: 18 passed in 1.32s, including path-through-cone
STOP, missing mapping veto, source regime normalization and LIVE builtin rejection.
No physical acquisition/actuation, and no independent approval claimed.
