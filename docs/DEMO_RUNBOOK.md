# DEMO DRIVE runbook

DEMO DRIVE is real motion, not autonomy. It still requires every existing gate: `system_check.sh`
GO, `readiness_report.py` either READY or the operator accepting a partial state, ARM (refused by
firmware NAK 7 until `docs/WIRING_SIGNOFF.md` is signed and `VCU_OUTPUT_WIRING_VERIFIED=1` is set),
and the manual power cut confirmed. Nothing in this runbook or in the software changes this turn
sets any verified flag.

## 1. Wheels-off rehearsal (mandatory first)

1. Kart on a stand, drive wheels clear of the ground.
2. `scripts/start_phase1.sh`, then open `http://127.0.0.1:5173/ops`.
3. `tools/readiness_report.py` -- review the READINESS GATES panel on `/ops`; NOT_RUN gates are
   expected pre-commissioning.
4. Tick "Area clear, physical E-stop reachable" only once a second person is holding the physical
   battery/supply disconnect within reach.
5. "Arm demo drive" (sends `demo_drive_start` -- refused by `autokart_vcu_safety` unless already
   ARMED through the normal arm flow and every startup gate holds).
6. HOLD TO DRIVE for 1-2 s. Confirm: wheels turn forward only, release stops within the firmware's
   300 ms link timeout, SV never exceeds 2.0 V (multimeter on SV).
7. Run the three fault drills below with wheels still off the ground before touching the ground.

## 2. Fault drills (wheels off, x3 each, record pass/fail)

| Drill | How | Expected |
|---|---|---|
| Heartbeat loss | Hold, then close the browser tab (or kill the tab's JS) mid-drive | SV -> 0 V, BRK applied within ~500 ms (gateway side) and 300 ms (firmware CMD_TIMEOUT_MS) |
| E-stop mid-ramp | Trip the physical power cut while holding | Traction removed independently of software; VCU reports FAULT/LINK_TIMEOUT once power returns |
| Serial unplug | Unplug the VCU USB cable while holding | Bridge's `vcu_armed` goes false within ~300 ms; CommandGate forces 0 V/100% brake |
| Over-cap request | (software test only, `tools/test_*`, not a bench step) 5 V request | Firmware clamps/faults at 2000 mV; never measured above 2.0 V on SV |
| Direction change while moving | Attempt a second `drive` request with a different direction mid-hold | Refused (`DIRECTION_CHANGE_WHILE_MOVING_REFUSED`); reverse itself is always refused, not wired |
| Boot state | Power cycle VCU, do nothing | Boots SAFE, EN stop, BRK applied; demo drive cannot be active (requires ARMED first) |

Record each as PASS/FAIL below. 3x3 required before considering ground testing.

**Run 1 (date, initials):** Heartbeat ___ E-stop ___ Serial ___ Direction ___ Boot ___
**Run 2 (date, initials):** Heartbeat ___ E-stop ___ Serial ___ Direction ___ Boot ___
**Run 3 (date, initials):** Heartbeat ___ E-stop ___ Serial ___ Direction ___ Boot ___

## 3. Ground test (only after 3/3 clean wheels-off runs)

Same steps as section 1, kart on the ground, clear 20 m area, second observer, physical power cut
in reach. Start with the shortest timed hold. Do not raise speed -- the firmware cap is fixed at
2.0 V and is not exposed as a tunable in this build.

## 4. What is real vs. simulated

| Shown on `/ops` | Real or simulated | Source |
|---|---|---|
| Camera RGB/depth + YOLO boxes | Real (LIVE) when the camera/perception node is running; otherwise explicit NO DATA | `/mission/api/camera` |
| VCU state, faults, EN/BRK | Real | `/mission/api/status` -> `vcu` channel |
| Readiness gates | Real, from the last `tools/readiness_report.py` run; NOT RUN if never run | `/mission/api/readiness` |
| Front safety envelope | Real topic, but DIAGNOSTIC_ONLY / schematic -- never pixel-accurate; labelled UNVERIFIED unless `data_source: LIVE` | `/safety/envelope` |
| Autonomy path/geometry (Delaunay, gates, centerline) | Only ever real perception/mapping output when qualified, or explicitly labelled SIMULATED on `/demo`; `/ops` never substitutes one for the other | perception/mapping stages |
| DEMO DRIVE throttle/brake | Real command, capped in firmware | `/autokart/demo_drive/control` -> bridge -> serial |

## 5. Launch

```
scripts/start_phase1.sh
```
Then open `http://127.0.0.1:5173/ops`. Stop with `scripts/stop_phase1.sh`.
