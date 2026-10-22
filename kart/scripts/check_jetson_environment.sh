#!/usr/bin/env bash
set -euo pipefail
printf 'AutoKart Jetson SIL environment check (read-only)\n'
uname -a
if [[ -r /etc/nv_tegra_release ]]; then
  sed -n '1,4p' /etc/nv_tegra_release
else
  printf 'JETSON PERFORMANCE: NOT_EXECUTED_EXTERNAL_DEPENDENCY\n'
  printf '/etc/nv_tegra_release is absent; this is not a detected Jetson target.\n'
  exit 2
fi
for command in nvidia-smi tegrastats ros2 gz colcon; do
  if command -v "$command" >/dev/null 2>&1; then
    printf '%-12s %s\n' "$command" "$(command -v "$command")"
  else
    printf '%-12s MISSING\n' "$command"
  fi
done
