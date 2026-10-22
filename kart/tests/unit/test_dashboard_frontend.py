from __future__ import annotations

import subprocess
from pathlib import Path


ROOT = Path(__file__).parents[2]


def test_dashboard_frontend_state_store():
    result = subprocess.run(
        ["node", "--test", "tests/frontend/dashboard-state.test.mjs"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
