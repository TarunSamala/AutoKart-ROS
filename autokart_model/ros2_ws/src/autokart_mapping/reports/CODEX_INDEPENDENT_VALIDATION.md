# Independent validation — 2026-10-22

Role: independent VALIDATOR for node autokart_mapping. No implementation edits.
Software fixtures only; no camera/serial/IMU physical acquisition or actuation.

Command from worktree/autokart_ros_nodes:

```bash
source /opt/ros/humble/setup.bash
source ../../validation_ws/install/local_setup.bash
PYTHONPATH="$PWD/autokart_mapping:$PYTHONPATH" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=93 python3 -m pytest -q autokart_mapping/test -rs
```

Exit code: 1

```text
.....F.....                                                              [100%]
=================================== FAILURES ===================================
___________ test_mapping_rejects_unqualified_frame_and_ground_truth ____________

algorithm = <autokart_mapping.algorithm.MappingAlgorithm object at 0x7177bd6d5540>

    def test_mapping_rejects_unqualified_frame_and_ground_truth(algorithm):
        engine = MappingEngine(algorithm)
        status = {"sample_time_ns": 1_000_000_000, "valid": True, "source": "LIVE",
                  "frame_id": "camera_optical", "ground_truth": False}
        result = engine.update(cones(), status, algorithm.PoseEstimate(0, 0, 0, 0, 1_000_000_000, 0.05))
        assert not result["valid"] and not result["tracked_cones"]
        status = {"sample_time_ns": 1_000_000_000, "valid": True, "source": "SIMULATED",
                  "frame_id": "base_link", "ground_truth": True}
        result = engine.update(cones(), status, algorithm.PoseEstimate(0, 0, 0, 0, 1_000_000_000, 0.05))
>       assert not result["valid"]
E       assert not True

autokart_mapping/test/test_engine.py:64: AssertionError
1 failed, 10 passed in 1.73s

```

Result: FAIL / BLOCKED; see exact failure above.
Physical qualification and vehicle readiness: NOT EXECUTED / NO-GO.
