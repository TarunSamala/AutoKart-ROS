#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" != "--execute" || -z "${2:-}" ]]; then
  cat <<EOF
No deployment performed.

Usage:
  $0 --execute user@jetson-host

The explicit form copies the current workspace (excluding generated artifacts,
build products and VCS data) to ~/autokart-sim-studio on the named target.
Run scripts/check_jetson_environment.sh on the target before qualification.
EOF
  exit 0
fi
target="$2"
rsync -az --delete-delay \
  --exclude .git --exclude artifacts --exclude build --exclude install \
  --exclude log --exclude __pycache__ \
  "$project_root/" "$target:autokart-sim-studio/"
printf 'Deployment copied to %s:autokart-sim-studio; no simulator started.\n' "$target"
