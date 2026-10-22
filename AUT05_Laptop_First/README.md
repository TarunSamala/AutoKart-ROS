# AUT-05 Laptop-First Kit (Jarvis)

Scope: develop and validate the camera→AUT-03→dashboard application **on the Ubuntu laptop first**; Pi deployment and VCU control are out of scope. The repository `TarunSamala/YDLIDARHP60C` is the native camera viewer + build wrapper, not already an integrated AutoKart camera API.

1. Extract this folder anywhere, e.g. `~/Desktop/autonomous_kart/AUT05_Laptop_First/`.
2. Run `bash AUT05_Laptop_First/laptop_preflight.sh` from a Jarvis terminal. Camera missing from laptop is not an error for replay/synthetic development.
3. Run `bash AUT05_Laptop_First/baseline_tests.sh` to repeat AUT-03 tests. It does not install packages or modify files.
4. From `~/Desktop/autonomous_kart`, run:
   ```bash
   codex exec --sandbox workspace-write --skip-git-repo-check - < AUT05_Laptop_First/CODEX_LAPTOP_FIRST.md
   ```
   Review each change, keep vehicle outputs disabled, and do not claim hardware PASS without connecting the HP60C to Jarvis.
5. To check the HP60C **if physically connected to Jarvis**, use the existing wrapper at `autonomy_stack_phase1/` if its `run_viewer.sh` exists: `cd ~/Desktop/autonomous_kart/autonomy_stack_phase1 && ./check_camera.sh && ./build.sh && ./run_viewer.sh`. Press `d` for RGB+depth. `./build.sh` downloads the checksum-pinned vendor SDK on first use, which requires network and build prerequisites. Do not run viewer alongside your live camera process.

The native SDK wrapper README: https://github.com/TarunSamala/YDLIDARHP60C

No changes have been applied to your actual Jarvis files or Pi by this kit. The files in this package are implementation instructions and read-only preflight/regression scripts; they are not a finished live dashboard.
