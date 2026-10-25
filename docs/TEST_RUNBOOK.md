# AutoKart test runbook

One page: the stages, pass/abort criteria, who holds the manual power cut, and what to paste where.
Run the whole sequence with `brake_test/run.sh` (guided, resumable, logs to
`results/braking_session_<timestamp>/`), or any stage directly with its own tool. The older
`scripts/braking_test_session.sh` and `scripts/test_session.sh` commands are retained as aliases.
**This kart has no E-stop
input (GPIO7 has no status wire): the physical battery/supply disconnect is the only emergency stop.**
A second person holds it, within reach, for every stage from 3 onward. Nothing in this runbook sets
`calibration_verified`, `vehicle_config_verified`, or `braking_verified` -- only the operator does, after
reading the evidence.

| # | Stage | Command | Pass criteria | Abort criteria |
|---|---|---|---|---|
| 0 | Software self-test | `brake_test/run.sh --stage 0` (or run each suite directly) | All touched suites pass, 0 failures | Any failure: stop, fix or report, do not continue |
| 1 | Stationary camera rate + rejections | `tools/camera_rate_check.py --duration 20` (graph running) | Reports real numbers; compare against the 9 Hz target | Camera busy/absent: stop, do not force a second owner |
| 2 | Depth fit | `tools/depth_check.py --fit-pitch` (tape stations 1.2, 2, 3, 4 m) | `PROVISIONAL_FIT_REVIEW`, residuals within budget | `NO_CONFIG_CHANGE_*`: more/better station data needed, not a reason to guess |
| 3 | Encoder calibration | `tools/encoder_calibration.py` (hand push, 3 trials, VCU bridge stopped first) | 3+ trials, consistent sign, VCU SAFE throughout | Direction flips between trials, or VCU not SAFE: stop |
| 4 | Latency | `tools/latency_report.py --live --seconds 30` | A real p50/p95/max from a complete frame chain | `NO_MEASUREMENT`: no qualified chain yet -- expected until stages 1-2 pass live |
| 5 | Braking test | `tools/braking_test.py /dev/ttyACM0` (typed CLEAR20/POWERCUT/OBSERVER, open 20 m strip, shortest run time first) | 5 final runs at the longest held time, stopping distance < 3 m | Any VCU fault, link loss, E-stop, or speed not zero within 6 s: tool aborts itself |
| 6 | Readiness report | `tools/readiness_report.py` | Every gate PASS | Any gate FAIL/NOT RUN: follow the printed command for that gate |
| 7 | Runway check | `tools/runway_check.py --v-run .. --a-verified .. --t-latency .. --margin ..` (measured values only) | Runway >= 0.5 m at every commissioning gap tested | Negative or < 0.5 m runway at the active gap: do not proceed |
| 8 | Stop-trigger test | `tools/stop_trigger_test.py` (wheels OFF the ground, typed `WHEELS OFF`) | Planner STOP_CANDIDATE -> bridge 0 V TX -> VCU TEL braked, all within timeout | Timeout, fault, or no event: stop; verify SV/BRK with a meter either way |
| 9 | First run | `scripts/first_run.sh` (requires `system_check.sh` GO, typed `CHECKED`) | Checklist reviewed, graph healthy | NO-GO from preflight: do not override |

## Who holds the manual power cut
A second person, physically at the battery/supply disconnect, for every stage from 3 onward (anything
that opens the VCU serial port or could request motion). They watch the kart, not the laptop screen.

## What to paste where (always printed by the tool, never written by it)
- `tools/depth_check.py --fit-pitch` / `tools/calibration_candidate.py` -> `height_m`, `pitch_down_deg`
  lines for `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/data/perception.yaml`.
- `tools/encoder_calibration.py` -> `counts_per_m`, `encoder_forward_sign` for
  `kart/ros2_ws/src/autokart_bringup/config/commissioning.yaml` (`autokart_localization`).
- `tools/latency_report.py` -> `t_latency` for the same file's `autokart_planner` block.
- `tools/braking_test.py` -> `brake_min_mps2` (the `a_verified` suggestion) for the same file.
- After reviewing all of the above: the operator (never a script) sets `calibration_verified`,
  `vehicle_config_verified`, `braking_verified` in
  `kart/ros2_ws/src/autokart_vcu_safety/config/vcu_safety.yaml`.

## Final gate
`tools/readiness_report.py` prints `READY FOR FIRST RUN` only when every gate is PASS, including the
three verified flags. Until then it prints `NOT READY` with the exact reasons and the exact command to
address each one. **Nothing in this runbook, or a passing run of it, authorizes vehicle motion by
itself** -- it only tells you what evidence exists and what is still missing.
