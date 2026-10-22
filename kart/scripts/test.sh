#!/usr/bin/env bash
set -euo pipefail

# Keep the project test environment deterministic. User-level pytest plugins
# can otherwise be auto-loaded even when they are not project dependencies.
PYTEST_DISABLE_PLUGIN_AUTOLOAD="${PYTEST_DISABLE_PLUGIN_AUTOLOAD:-1}" \
  python3 -m pytest
