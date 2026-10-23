# Senior reviewer report — autokart_odometry

A sequential, adversarial second pass over `node.py`/`backends.py` and the docs, started *because* the
suite passed 20/20 on its first run, which is a reason to look harder.

## Defects found and fixed

1. **Covariance parameters were published unvalidated.** A `0`, negative or `NaN` value in any of the four
   variance parameters went straight into `pose.covariance`/`twist.covariance`. `docs/NODE_CONTRACTS.md`
   requires finite covariance, and a `0` variance claims perfect certainty to any downstream fusion or
   mapping, which is the dangerous direction. **Fixed:** all four must be finite and > 0, otherwise the
   node fails closed with `INVALID_CONFIG:<name> ...`. **Regression test:**
   `test_nonpositive_or_nonfinite_covariance_is_refused[0.0|-1.0|nan]`. **Reproduced pre-fix:** with the
   check disabled, all 3 parametrized cases failed; restored, they pass.

2. **Factual error in the docs about which node is the conflicting TF writer.** The first draft of
   CONTRACT.md and the `node.py` docstring said running alongside `autokart_localization` creates a
   duplicate `odom -> base_link` writer. A grep showed `autokart_localization` broadcasts no TF; the actual
   existing writer is `autokart_gazebo`'s follower plugin. **Fixed** in both places. A wrong hazard
   statement is a defect in a safety document even when the code is right: it would send an operator to
   disable the wrong thing.

3. **Intermittent test-teardown race (all three packages' suites, found while soak-testing this one).**
   Tests called `executor.shutdown()` and then `destroy_node()` without joining the spin thread, so a
   callback could still be in flight during destruction. The symptom was a sporadic
   `cannot use Destroyable because destruction was requested` at interpreter exit. **Fixed** by joining
   the spin thread before destroying nodes, in every suite of this batch. A related IMU-only flake (an
   instant cross-topic assert) is documented in that node's reports.

## Documented hazard, not fixable here

**Speed mode cannot represent reversing.** `WheelOdometry.update()` ignores `encoder_forward_sign`. The
VCU `TEL.speed_mmps` is treated as unsigned upstream (`< 0` means unavailable in both the firmware protocol
and `core.py`). So with `source: actuator_feedback`, a reversing kart is either integrated as forward
motion or rejected. Fixing this needs a direction signal that does not exist in the current protocol or
messages. It is written prominently into CONTRACT.md rather than papered over.

## Checked and found correct

- **No reimplementation.** The only kinematics in the package are calls to the reused `update()` /
  `update_counts()`. Quaternion construction from `yaw_rad` is message packing, not odometry.
- **Re-anchoring after every rejection.** In counts mode, without this, a delta spanning an invalid
  interval would be integrated over a small `dt` as a huge speed. After any reject `prev` is `None`, so
  the next sample only anchors. Covered by the gap, non-monotonic and speed-loss tests.
- **No pose is republished while INVALID.** `/odom` and TF are emitted only on an accepted update. The
  speed-loss and invalid-feedback tests assert the odometry count stops growing (allowing one message in
  flight).
- **Steering loss freezes yaw, not translation.** The test asserts yaw is constant to 1e-9 across the tail
  while x/y keep changing.
- **Counts mode with `ActuatorFeedback` is refused**, not silently downgraded to speed mode.
- **`wheelbase_m` has no guessed default.** At `0.0` the reused class's own `ValueError` becomes
  `INVALID_CONFIG`.
- **Stamp domains.** Input stamps are used only for `dt`; output stamps use the ROS clock. This is right
  because node 04's `measurement_time` comes from a monotonic clock.
- **Signature use:** `update(dt_s, speed_mps, steering_rad)`, `update_counts(dt_s, delta_counts:int,
  steering_rad)`. The `int(...)` cast on the counts delta matters because the reused method rejects
  non-`int` deltas. It was checked, since numpy or float counts would silently mark every sample INVALID.

## Not independently verified

- Physical correctness of any parameter, and behaviour on a real kart (lifted axle, push tests, turning
  radius against the bicycle model). There is no hardware.
