# AutoKart VCU command gate — Stage 0

This is the first, host-testable part of the proposed ESP32-S3 VCU. It is a
platform-independent command and arming gate. It has **no GPIO, PWM, DAC,
stepper, brake, contactor, ROS publisher/subscriber, or flash target**. The
existing RC firmware and the NOOP-only micro-ROS bench endpoint remain
separate and untouched. This library must not be interpreted as a deployable
vehicle controller.

The source of requirements is `/home/mark1/Desktop/vcu.md` (revision 0.1),
especially its command expiry, feedback, and independent stop requirements.
`kart/kart_sim/vcu.py` remains the existing simulation VCU; this C module is
the proposed embedded boundary, not a replacement for the simulator.
The ROS workspace separately contains a read-only NOOP bench status adapter
in `autokart_diagnostics`; it is not connected to this C gate or any actuator.

## Implemented boundary

- Power-up and reset initialize `SAFE`, with no command and no authorization.
- The platform must supply a nonzero boot ID and an unpredictable nonzero
  session ID. Every command must match both, carry protocol version 1, use a
  strictly increasing sequence number, and arrive with a bounded TTL.
- Received time is the VCU's monotonic time. A Pi wall-clock timestamp cannot
  prolong validity. A command expires at the earlier of its TTL and the
  100 ms local ceiling.
- Nonfinite or out-of-range target values, stale/replayed commands, clock
  rollback, a lost Agent, or lost/stale hardware qualification latch a fault
  and clear the command. The platform must refresh its qualified hardware
  snapshot at least every 25 ms. A heartbeat cannot extend a motion command.
- `READY` requires *all* hardware qualifications, measured bounds, and a
  physical authorization input supplied by the eventual platform adapter.
  `READY` rejects nonzero speed or steering commands. `AUTONOMOUS`
  additionally requires a fresh zero-speed, zero-steering command and a
  distinct physical start **edge**. Both transitions are impossible with the
  default empty qualification.
- A stop request clears the accepted drive target and enters non-armable
  `STOP_PENDING`. This **does not claim a physical stop**; no brake output or
  measured-stop completion path exists here. `SAFE` cannot be re-entered
  without a fresh boot of this Stage 0 module.

The `hardware_ok` bitmask and `physical_authorized`/`physical_start` booleans
are trusted **only** when produced by a verified hardware adapter. They must
never be populated from ROS, a browser, or simulated telemetry in a vehicle
build. The physical start parameter must be supplied only on a new physical
input edge after READY, never as a held level. Checksums/authentication,
actual sensor freshness, control loops,
independent traction isolation, braking, fault recovery, and physical testing
are still open. No arming API is connected to the existing bench transport.

## Host test

`make -C vcu test` compiles only the pure C gate and runs its adversarial
software checks. It does not build or flash ESP-IDF firmware and cannot test
electrical boot states, sensor timing, brake force, or independent E-stop.

## Next integration gates

1. Record the installed board, flash identity, as-wired schematic and pin
   functions; reconcile the competing RC firmware mappings.
2. Confirm AS5047P shaft/interface/count scale, steering AS5600 location and
   road-wheel calibration, actual brake feedback, and independent stop path.
3. Freeze a versioned Pi↔VCU packet/ROS contract and add packet integrity and
   transport tests. The current `autokart_msgs/MotionCommand` and the
   NOOP-only bench packet are **not** interchangeable.
4. Add an ESP-IDF adapter with electrically verified safe boot/output states,
   bounded tasks, watchdog response, measured telemetry and actual brake
   feedback. Build/flash only after isolated hardware and a restorable image
   backup are confirmed.

**Vehicle release: NO-GO.** Passing the host test is software evidence only.
