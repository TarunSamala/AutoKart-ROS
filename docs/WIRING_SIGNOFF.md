# VCU output wiring sign-off

Gates `VCU_OUTPUT_WIRING_VERIFIED` in `vcu_firmware/include/config.h`. This document does not set
it; the operator does, by editing that macro to `1` and reflashing, only after every row below is
measured and matches. Until then every `ARM` returns `NAK 7`.

## Pin assignment (frozen, confirmed 2026-10-19)

| Signal | GPIO | Source |
|---|---|---|
| BLDC EN | 1 | `vcu_firmware/include/config.h` `PIN_BLDC_EN`; matches `AutoKart_S3_Pin_and_Power_Budget_RevA.xlsx` row 5 |
| BLDC BRK | 2 | `PIN_BLDC_BRK`; budget row 6 |
| BLDC F/R | 39 | `PIN_BLDC_FR`; budget row 24 |
| E-stop aux status (not fitted) | 7 | `PIN_ESTOP_STATUS`; budget row 10 |
| Steering feedback (AS5600) | 13 | budget row 16 |
| Brake feedback (AS5600) | 14 | budget row 17 |

**`bench/PINOUT.txt`, `bench/bldc/PINOUT.txt` and `bench/full_setup/PINOUT.txt` disagree**: they
list GPIO7 as BLD-750 F/R. That is stale/wrong against the frozen pin budget (which assigns GPIO7
to E-stop status and GPIO39 to F/R) and against `vcu_firmware`'s actual, already-flashed-candidate
source. Not corrected in this pass; flagged for a separate fix to the bench docs.

## Polarity table (measure, do not infer)

| Signal | Macro | Claimed level | Measured level (V, relative to COM) | Matches? | Date | Name |
|---|---|---|---|---|---|---|
| EN | `GPIO_EN_ENABLE_LEVEL` | HIGH = run | | | | |
| BRK | `GPIO_BRK_ASSERT_LEVEL` | LOW = release | | | | |
| F/R | `GPIO_FR_FORWARD_LEVEL` | LOW = forward | | | | |

## Bench evidence referenced

- `bench/bldc/PINOUT.txt`: "The NPN interface assumed by the code makes ESP GPIO HIGH pull the BLD
  input to COM." (consistent with the EN row above; BRK/F-R active-low per the macros, still VERIFY).
- `docs/HANDOVER_STATUS.md` (2026-10-19): operator-reported terminal measurements --
  "GPIO1 HIGH -> EN shorted to COM, GPIO2 LOW -> BRK open, GPIO39 LOW -> F/R open" -- reported, not
  independently observed here, and not sufficient on its own to flip the verified flag per the
  existing hard rule (measure at the driver terminal, see `docs/BENCH_TESTS.md` section (c)).
- `docs/BENCH_TESTS.md` section (c) ("EN / BRK / F-R polarity -- measure, do not infer") is the
  full physical procedure; this table is its summary sign-off record, not a replacement for it.

## Sign-off

Only the operator completes this section, after running `docs/BENCH_TESTS.md` section (c) in full
on the actual board.

**Result (PASS / FAIL):** ______________________
**Date:** ______________________
**Name:** ______________________
**Signature:** ______________________

Setting `VCU_OUTPUT_WIRING_VERIFIED=1` without a completed row above, for every signal, is not
authorized by this document.
