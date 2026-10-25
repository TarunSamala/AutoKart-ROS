#!/usr/bin/env bash
# Backward-compatible alias for the packaged Phase-1 braking workflow.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
exec "$ROOT/brake_test/run.sh" "$@"
