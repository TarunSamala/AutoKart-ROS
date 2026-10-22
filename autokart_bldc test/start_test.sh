#!/usr/bin/env bash
# Starts the experimental AutoKart BLDC test harness.
# Does not touch production ROS 2, VCU firmware, or safety config.
# This is a passive Pico serial monitor: it never sends an actuator command.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
exec python3 app.py "$@"
