#!/usr/bin/env bash
# Canonical entry point for the packaged AutoKart Phase-1 braking workflow.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
exec "$ROOT/brake_test/braking_test_session.sh" "$@"
