# Independent validation — 2026-10-22

Role: independent VALIDATOR for node autokart_perception_adapter. No implementation edits.
Software fixtures only; no camera/serial/IMU physical acquisition or actuation.

Command from worktree/autokart_ros_nodes:

```bash
source /opt/ros/humble/setup.bash
source ../../validation_ws/install/local_setup.bash
PYTHONPATH="$PWD/autokart_perception_adapter:$PYTHONPATH" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=93 python3 -m pytest -q autokart_perception_adapter/test -rs
```

Exit code: 1

```text
.........F.                                                              [100%]
=================================== FAILURES ===================================
________________ test_missing_dependency_keeps_reporting_fault _________________

ros_context = None
tmp_path = PosixPath('/tmp/pytest-of-mark1/pytest-101/test_missing_dependency_keeps_0')

    def test_missing_dependency_keeps_reporting_fault(ros_context, tmp_path):
        from rclpy.parameter import Parameter
        missing_path = str(tmp_path / "does_not_exist")
        node = PerceptionAdapterNode(parameter_overrides=[
            Parameter('backend', Parameter.Type.STRING, 'algorithm'),
            Parameter('algorithm_package_path', Parameter.Type.STRING, missing_path),
            Parameter('input_mode', Parameter.Type.STRING, 'standalone_timer'),
            Parameter('standalone_period_s', Parameter.Type.DOUBLE, 0.2),
        ])
        assert node.backend is None
        assert "DEPENDENCY_PATH_NOT_FOUND" in node.startup_error
    
        received_list = []
    
        def check(received):
            sub = received  # unused, we track via the list captured below
            assert _wait_for(lambda: len(received_list) >= 2, timeout_s=5.0)
            assert all(m.quality == "INVALID" for m in received_list)
            assert all("DEPENDENCY_PATH_NOT_FOUND" in m.fault_state for m in received_list)
    
        # Re-subscribe manually to collect every message (not just the latest).
        sub_node = rclpy.create_node("test_subscriber_repeat")
        sub_node.create_subscription(
            SensorDeliveryMetadata, "/perception/status", lambda m: received_list.append(m), 10)
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.add_node(sub_node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        try:
>           check({})

autokart_perception_adapter/test/test_fault_injection.py:128: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

received = {}

    def check(received):
        sub = received  # unused, we track via the list captured below
        assert _wait_for(lambda: len(received_list) >= 2, timeout_s=5.0)
        assert all(m.quality == "INVALID" for m in received_list)
>       assert all("DEPENDENCY_PATH_NOT_FOUND" in m.fault_state for m in received_list)
E       assert False
E        +  where False = all(<generator object test_missing_dependency_keeps_reporting_fault.<locals>.check.<locals>.<genexpr> at 0x79948c94f370>)

autokart_perception_adapter/test/test_fault_injection.py:116: AssertionError
----------------------------- Captured stderr call -----------------------------
[ERROR] [1792622336.686771259] [autokart_perception_adapter]: autokart_perception_adapter startup failed: DEPENDENCY_PATH_NOT_FOUND:/tmp/pytest-of-mark1/pytest-101/test_missing_dependency_keeps_0/does_not_exist
[WARN] [1792622337.191951342] [autokart_perception_adapter]: autokart_perception_adapter fault: CAMERA_INPUT_TIMEOUT
1 failed, 10 passed in 2.48s

```

Result: FAIL / BLOCKED; see exact failure above.
Physical qualification and vehicle readiness: NOT EXECUTED / NO-GO.
