# AutoKart laptop ↔ VCU serial protocol (v1)

USB CDC serial, 115200 (ignored by native USB), 8N1. One ASCII frame per line:

    $<TYPE>,<field>,<field>,...*<CRC16>\n

- CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF) over the bytes between `$` and `*` (exclusive), written as 4 upper-case hex digits.
- Frames with a bad CRC, bad length or unknown type are ignored.
- All numbers are integers (no floats on the wire).

## Laptop → VCU

| Frame | Fields | Meaning |
|---|---|---|
| `CMD` | `seq,throttle_mv,steer_cdeg,brake_pct` | Sent at 20 Hz. Doubles as the heartbeat. `throttle_mv` 0..2000 (hard cap). `steer_cdeg` (centi-degrees) and the steering part of the command are parsed and **ignored** in this release. `brake_pct` ≥ 50 asserts the BLD-750 BRK line. |
| `ARM` | `seq` | Request SAFE → ARMED. Rejected unless every precondition holds (see NAK codes). |
| `DIS` | `seq` | Disarm immediately → SAFE (drive inhibited). |
| `RST` | `seq` | Clear a latched FAULT → SAFE, only if its cause is gone. |

`seq` is a 32-bit counter that must increase; duplicates and old values are ignored.

## VCU → laptop

| Frame | Fields |
|---|---|
| `TEL` (50 Hz) | `fwseq,t_ms,state,fault,flags,en,brk,dac_mv,counts,speed_mmps,steer_raw,brake_raw,ack_seq,link_age_ms` |
| `HELLO` (every 5 s and at boot) | `fw_version,proto_version` |
| `NAK` | `seq,code` |

`state`: 0 SAFE, 1 ARMED, 2 FAULT.

`fault` bit mask: 1 LINK_TIMEOUT, 2 ESTOP, 4 DAC_FAIL, 8 CMD_RANGE.

`flags` bit mask: bit0 estop_ok, bit1 steer_pwm_ok, bit2 brake_pwm_ok, bit3 axle_motion_seen, bit4 dac_ok.

`steer_raw` / `brake_raw`: AS5600 12-bit reading 0..4095, or -1 if invalid. The laptop applies zero offset and scale (config/vehicle/vehicle.yaml).

`NAK` codes for `ARM`/`RST`: 1 wrong state, 2 E-stop active, 3 DAC not ready, 4 command link not fresh, 5 throttle not zero, 6 fault cause still present.

## VCU safety behaviour (implemented in `vcu_logic.cpp`, host-tested)

- Boots into SAFE with the drive inhibited and throttle 0 V.
- Throttle is clamped to 2000 mV; a command above the cap while ARMED is treated as a bug and latches FAULT (CMD_RANGE).
- No valid `CMD` for 300 ms while ARMED → FAULT (LINK_TIMEOUT).
- E-stop status line not OK while ARMED → FAULT (ESTOP).
- FAULT: drive inhibited, throttle 0 V, BRK asserted, latched until `RST`.
- Throttle ramps up at a limited rate; it drops instantly on brake, fault or disarm.
