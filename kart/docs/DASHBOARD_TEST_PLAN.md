# Dashboard test plan

Automated tests cover normalized parsing, finite pose values, track projection,
kart dimensions, deterministic decimation, bounded buffers, interpolation,
measured/display time, stale/backward timestamps, fault/state event mapping,
read-only route inventory, real trajectory movement, steering sign changes,
wheel rotation input, wet/dry wheel surfaces, and VCU mode.

Commands:

```bash
python3 -m pytest tests/unit/test_dashboard.py \
  tests/unit/test_dashboard_rebuild.py \
  tests/unit/test_dashboard_frontend.py -q
python3 -m pytest tests/integration/test_dashboard_visual_scenarios.py -q
node --test tests/frontend/dashboard-state.test.mjs
./scripts/test.sh
```

Manual endpoint checks verify manifest/track/replay responses, WebSocket pose
movement, and HTTP 405 for mutation. The generic visual replay generator is:

```bash
bash scripts/generate_dashboard_runs.sh runs/dashboard-visual
```

It covers nominal straight, constant-radius, slalom, wet surface, steering jam,
brake cable break, stuck torque, VCU freeze, physical E-stop, controlled stop,
fault-lockout/timeout, and E-stop-at-rest replay inputs.

Native headless Firefox screenshot capture is not reliable in the current
container because its software compositor cannot map a framebuffer. Browser
rendering is therefore verified interactively at localhost and via static/JS
and endpoint tests; this limitation is not labelled as a visual pass.

