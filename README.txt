AUTONOMOUS KART WORKSPACE
=========================

NO-GO FOR VEHICLE MOTION. E-STOP INPUT NOT FITTED: GPIO7 has no status wire.
Software STOP depends on the laptop, USB link and ESP32. The manual battery/
supply disconnect is the only emergency stop. Keep traction isolated for this
software work; the three verified flags remain false.

NO-GO until tools/readiness_report.py prints READY FOR FIRST RUN and the
operator has set calibration_verified, vehicle_config_verified and
braking_verified. See docs/TEST_RUNBOOK.md for the full stage sequence and
scripts/test_session.sh for a guided, resumable runner.

Optional monitored upgrade: a normally closed switch from GPIO7 to 3.3 V with
an external pull-down makes a wire break read LOW; then build with
ESTOP_INPUT_FITTED=1 and verify the fitted input on the actual board.

Repository layout
-----------------

bench/
  MicroPython hardware bench firmware, separated by subsystem. Start with
  bench/README.txt and always read the local README.txt and PINOUT.txt before
  connecting or energizing hardware. All older originals are preserved under
  bench/archive/; nothing from the original hardware folders was discarded.
  The new recovery manifest is indexed at bench/archive/recovery_index/.

phase_sim/
  React/TypeScript Delaunay autonomy visual simulator. This is the directory
  previously named phase1. See phase_sim/README.md and PROJECT_INFO.txt.

kart/
  Full Python/ROS 2/Gazebo autonomous-kart simulation and research platform.
  See kart/README.md and kart/docs/.

vcu_firmware/
  New, separate ESP32-S3 VCU firmware (an earlier version flashed; the current
  source is not flashed) for the real
  USB-serial bridge at kart/ros2_ws/src/autokart_vehicle_bridge. See
  vcu_firmware/README.md, docs/VCU_PROTOCOL.md and docs/BENCH_TESTS.md before
  touching hardware. NO-GO: see PROJECT_INDEX.md.

tools/
  Bench console/ramp/braking test scripts for vcu_firmware/ (no ROS). See
  docs/BENCH_TESTS.md.

Validation
----------

Bench static checks:
  python3 bench/validate_bench.py

Phase simulator:
  cd phase_sim
  npm install
  npm run check

Full simulator:
  cd kart
  ./scripts/test.sh

Safety boundary
---------------

The bench firmware is prototype test software. It is not a certified or
independent emergency-stop system. Vehicle testing requires a hardwired,
normally-safe circuit that removes traction energy independently of the
ESP32, Wi-Fi, LoRa, I2C, and application software.
