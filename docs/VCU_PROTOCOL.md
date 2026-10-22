# AutoKart laptop <-> VCU serial protocol (v1.1)

Owner: `vcu_firmware/` (ESP32-S3 side) and `kart/ros2_ws/src/autokart_vehicle_bridge` (laptop side). This is a
**new** protocol for the **new** `vcu_firmware/` firmware; it is not the NOOP-only micro-ROS bench packet
documented in `docs/NODE_CONTRACTS.md` (`autokart_diagnostics/bench_vcu_status`), and the two are not
interchangeable. Reused, with minor renames, from the equivalent document built against the same hardware facts
in `autokart/docs/PROTOCOL.md` during an earlier scoping pass of this workspace.

USB CDC serial, 115200 (ignored by native USB), 8N1. One ASCII frame per line:

    $<TYPE>,<field>,<field>,...*<CRC16>\n

- CRC16-CCITT-FALSE (poly `0x1021`, init `0xFFFF`) over the bytes between `$` and `*` (exclusive), written as 4
  upper-case hex digits.
- Frames with a bad CRC, bad length or unknown type are ignored.
- All numbers are integers (no floats on the wire).

## Laptop -> VCU

| Frame | Fields | Meaning |
|---|---|---|
| `CMD` | `seq,throttle_mv,steer_cdeg,brake_pct` | Sent at 20 Hz. Doubles as the heartbeat. `throttle_mv` 0..2000 (hard cap). `steer_cdeg` (centi-degrees) is parsed and **ignored** — steering is observed, never actuated, in this release. `brake_pct` >= 50 asserts the BLD-750 BRK line. |
| `ARM` | `seq` | Request SAFE -> ARMED. Rejected unless every precondition holds (see NAK codes). |
| `DIS` | `seq` | Disarm immediately -> SAFE (drive inhibited). |
| `RST` | `seq` | Clear a latched FAULT -> SAFE, only if its cause is gone. |
| `OBS` | `seq,frame_ms,cone_id,forward_mm,gap_mm,decision` | Phase 1 camera/algorithm observation only. `decision`: 0 STOP, 1 OBSERVE, 2 AT_TARGET. It never arms or requests throttle; STOP/AT_TARGET disarm. Missing geometry uses -1 for ID/distance. |

`seq` is a 32-bit counter that must increase; duplicates and old values are ignored.

## VCU -> laptop

| Frame | Fields |
|---|---|
| `TEL` (50 Hz) | `fwseq,t_ms,state,fault,flags,en,brk,dac_mv,counts,speed_mmps,steer_raw,brake_raw,ack_seq,link_age_ms` |
| `HELLO` (every 5 s and at boot) | `fw_version,proto_version` |
| `NAK` | `seq,code` |
| `OBSACK` | `seq,decision` — acknowledges accepted observation syntax, not physical motion or braking. |

The default no-encoder, arming-locked source identifies itself as `0.2.2-no-estop-input-locked` and sends `HELLO` revision `2` (protocol v1.1). An optional fitted-input build identifies as `0.2.2-estop-fitted-locked`. The previously flashed `0.2.1` / revision `1` board is not updated by a source build; the bridge rejects ARM until matching firmware is explicitly installed in a separate authorized session. In its `TEL`, `counts=-1` and
`speed_mmps=-1` mean **unavailable**, never zero motion. The bridge publishes invalid speed feedback
and inhibits nonzero throttle for this firmware until an independently qualified motion estimator is
integrated. Bit3 (`axle_motion_seen`) remains clear.

`state`: 0 SAFE, 1 ARMED, 2 FAULT.

`fault` bit mask: 1 `LINK_TIMEOUT`, 2 `ESTOP`, 4 `DAC_FAIL`, 8 `CMD_RANGE`.

`flags` bit mask: bit0 `estop_ok` (meaningful only with bit5), bit1 `steer_pwm_ok`, bit2 `brake_pwm_ok`, bit3 `axle_motion_seen`, bit4 `dac_ok`, bit5 `estop_input_fitted`. With `ESTOP_INPUT_FITTED=0` (default), both bit0 and bit5 are clear. This explicitly means **NOT FITTED - MANUAL POWER CUT ONLY**; bit0 clear must not be displayed as a pressed switch. With `ESTOP_INPUT_FITTED=1`, bit5 is set and bit0 reports the filtered GPIO7 level (HIGH healthy, LOW pressed after three consecutive bad samples). `TEL` field count, framing and CRC are unchanged.

`steer_raw` / `brake_raw`: AS5600 12-bit reading 0..4095, or -1 if invalid. The laptop applies zero offset and
scale from its own configuration; the firmware does not interpret these values.

`NAK` codes for `ARM`/`RST`: 1 wrong state, 2 E-stop active, 3 DAC not ready, 4 command link not fresh, 5 throttle
not zero, 6 fault cause still present, 7 output wiring unverified. The source firmware ships with
`VCU_OUTPUT_WIRING_VERIFIED=0`, so an ARM always returns code 7 until the actual GPIO-to-BLD-750
levels have been measured and deliberately configured. The manufacturer documents BRK **open** as quick stop;
the source's active-high PN2222 candidate therefore sets `GPIO_BRK_ASSERT_LEVEL LOW`, still marked VERIFY.

## VCU safety behaviour (implemented in `vcu_firmware/src/vcu_logic.cpp`, host-tested)

- Boots into SAFE with the drive inhibited and throttle 0 V.
- Throttle is clamped to 2000 mV; a command above the cap while ARMED is treated as a bug and latches FAULT
  (`CMD_RANGE`) rather than being silently clamped.
- No valid `CMD` for 300 ms while ARMED -> FAULT (`LINK_TIMEOUT`).
- Only if `ESTOP_INPUT_FITTED=1`: GPIO7 not OK while ARMED -> FAULT (`ESTOP`). With the default `0`, GPIO7 is neither configured nor read and cannot fault or block ARM.
- FAULT: drive inhibited, throttle 0 V, BRK asserted, latched until `RST` (and `RST` itself is refused while the
  fault's cause — fitted E-stop input, DAC, or link — is still present).
- Throttle ramps up at a limited rate (1000 mV/s, ~2 s from 0 to the 2 V cap); it drops instantly on brake, fault
  or disarm.
- On any stop, the DAC is written to 0 V before EN is de-asserted / BRK is asserted — see the stop-sequence note
  in `vcu_firmware/README.md`.

## Mapping onto the ROS vehicle bridge

`autokart_vehicle_bridge` is the single owner of this serial link and the single writer of `CMD`/`ARM`/`DIS`/`RST`.
It:
- Subscribes to `autokart_msgs/MotionCommand` on `/autokart/controller/motion_command_candidate` (the existing,
  disarmed Pure Pursuit candidate — see `docs/NODE_CONTRACTS.md`) and converts `target_speed_mps` into a throttle
  voltage: 0 V, or a value in `[throttle_run_v, throttle_v_max]` (never in between — the motor does not move below
  roughly `throttle_run_v`), via its own command gate (`core.py`). It never actuates `steering_angle_rad`.
- Only forwards a nonzero throttle when the upstream safety state is ARMED, the VCU itself reports ARMED, and the
  candidate command is fresh (see `CommandGate` in `core.py`) — this is the "safety state ARMED AND VCU ARMED AND
  fresh command" gate, independent of and in addition to the firmware's own CMD_RANGE/LINK_TIMEOUT checks.
- Subscribes to a bounded `arm`/`stop`/`reset_fault` request topic published only by the VCU safety supervisor
  (never directly by the dashboard) and translates it to `ARM`/`DIS`/`RST`.
- Publishes `autokart_msgs/ActuatorFeedback` on `/vehicle/actuator_feedback` and `autokart_msgs/VcuState` on
  `/vcu/state` from the `TEL` stream. Any field this protocol does not measure (brake force, motor current/torque,
  battery voltage) is published as `0.0`/`NaN` with `valid` reflecting whether the *measured* fields are trustworthy
  — never a fabricated plausible-looking number.
