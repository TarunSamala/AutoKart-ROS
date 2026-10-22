# VCU Stage 0 test record — 2026-10-17

## Scope and result

This record covers the new host-only command gate in `vcu/`. It is **not** a
firmware build, a flashed image, an electrical test, or evidence of stopping
the kart. **Vehicle release: NO-GO.**

| Check | Result | Evidence |
|---|---|---|
| C11 build, strict warnings, adversarial host assertions | PASS | [`evidence/host_guard.log`](evidence/host_guard.log) |
| AddressSanitizer + UndefinedBehaviorSanitizer host run | PASS | [`evidence/sanitizer.log`](evidence/sanitizer.log) |
| Existing Python VCU unit/integration cases | 15 passed | [`evidence/existing_vcu_tests.log`](evidence/existing_vcu_tests.log) |
| Legacy ESP-IDF RC firmware static cases | 9 passed | [`evidence/rc_firmware_static.log`](evidence/rc_firmware_static.log) |
| Cppcheck warning/style/performance/portability pass | PASS, no findings | [`evidence/cppcheck.log`](evidence/cppcheck.log) |
| ESP-IDF production image build | NOT EXECUTED | No production adapter or local `idf.py` is present. |
| Connected ESP32/physical stop/actuator test | NOT EXECUTED | Local `lsusb` showed no Espressif device; no serial port was opened. |

Python pytest initially failed before collecting tests because the host's
autoloaded `anyio` plugin expects a newer `_pytest.scope` than the installed
pytest provides. The recorded Python runs use
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`; the tests themselves then passed. This
environment workaround does not change project source.

## Host scenarios covered

- Default SAFE, missing brake qualification, and absent physical start.
- Fresh zero-speed/zero-steering prerequisite before autonomous entry.
- Command expiry at the local monotonic deadline, stale hardware snapshot,
  clock rollback, and Agent loss.
- Duplicate sequence, wrong boot/session, nonfinite value, excess speed,
  excessive TTL, and removed independent-stop/physical authorization.
- STOP clears the drive target and enters non-armable `STOP_PENDING`; it does
  not report a measured physical stop or silently return to READY.

The C tests feed **synthetic qualification bits**. They establish software
logic behavior only. A future board adapter must derive every bit from actual
fresh signals and measured configuration; ROS/browser data cannot set them.

## Open release blockers

1. Installed board, electrical drawing, active firmware checksum and pin
   polarities have not been established for this VCU candidate.
2. AS5047P electrical interface, shaft location and counts/revolution;
   steering AS5600 location and road-wheel map; brake position/force feedback;
   and independent emergency stop/traction isolation remain unverified.
3. Pi↔VCU wire serialization, integrity checks, session handshake, ROS mapping,
   measured timing and independent watchdog response are not implemented here.
4. No ESP-IDF platform adapter, GPIO-safe boot proof, actuator control loop,
   physical brake fallback, bench measurements, or end-to-end fault injection
   has been completed.

The previously saved USB micro-ROS bench endpoint at
`/home/mark1/Desktop/autokart-remote/firmware` remains NOOP-only and was not
modified. The older RC firmware under `AutoKart_RC_Dynamics_ESP32S3_v1/` was
not modified or flashed. The present `kart/ros2_ws/src/autokart_msgs/msg/MotionCommand.msg`
and the bench `UInt64MultiArray` packet are different contracts; neither is
silently connected to this gate.

## Commands run

```text
make -C vcu test
cc -std=c11 -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -Wall -Wextra -Werror -pedantic -Ivcu vcu/guard.c vcu/tests/test_guard.c -lm -o /tmp/autokart_vcu_guard_san
/tmp/autokart_vcu_guard_san
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=kart python3 -m pytest -q kart/tests/unit/test_vcu.py kart/tests/unit/test_vcu_state_guards.py kart/tests/unit/test_vcu_command_envelope.py kart/tests/integration/test_vcu_scenario.py kart/tests/integration/test_vcu_bypass_prevention.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q AutoKart_RC_Dynamics_ESP32S3_v1/tests/test_static.py
cppcheck --enable=warning,style,performance,portability --error-exitcode=1 --quiet vcu/guard.c vcu/tests/test_guard.c
```

Espressif's [ESP32-S3 watchdog documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32s3/api-reference/system/wdts.html)
distinguishes task watchdog observation from reset configuration. The future
adapter must test a fail-safe response rather than assume that a task watchdog
warning alone inhibits outputs. The [PCNT documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32s3/api-reference/peripherals/pcnt.html)
is relevant if the AS5047P is actually wired as ABI; its wired interface has
not yet been verified.
