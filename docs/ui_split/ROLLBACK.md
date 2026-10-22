# AutoKart UI split — Rollback (non-destructive)

Goal: return to the pre-split layout (single Vite app on 5173 + `autokart_apps/run_*.sh`) without
deleting anything. Every step below copies or moves; nothing is `rm`-ed.

Backup taken before the split: `_rollback/ui_split_20261018T150757Z/` (2026-10-18 15:07:57 UTC).

## 1. Stop the split stacks

```bash
cd /home/mark1/Desktop/autonomous_kart
scripts/run_dashboard.sh stop
scripts/run_lab.sh stop
scripts/status.sh            # every row ALIVE=no; LISTEN=FOREIGN rows are not ours — leave them
# if the systemd examples were installed by hand:
#   sudo systemctl stop autokart-dashboard autokart-lab autokart-perception
```

## 2. Legacy entry points still work as-is (often enough on its own)

`autokart_apps/run_live.sh`, `run_lab.sh`, `run_core.sh`, `run_tests.sh` were not modified by the split
(byte-identical to the backup as of this writing). The split only *added* files, plus the three
modified files in §3. So in most cases you can simply run:

```bash
autokart_apps/run_live.sh --mode live      # disarmed HP60C dashboard, http://127.0.0.1:8768/live
autokart_apps/run_lab.sh                   # lab_api 8767 + legacy Vite on 5173
```

## 3. Restore the pre-split source files (only if the legacy app itself is broken)

First keep a copy of the current versions next to the backup, then copy the backup back.

| Backup file | Restore to | Differs from current? (checked 2026-10-18) |
|---|---|---|
| `_rollback/ui_split_20261018T150757Z/phase_sim/package.json` | `phase_sim/package.json` | **yes** (split added `dev:/build:/preview:{dashboard,lab}` scripts) |
| `_rollback/ui_split_20261018T150757Z/phase_sim/tsconfig.node.json` | `phase_sim/tsconfig.node.json` | **yes** (includes the new vite configs) |
| `_rollback/ui_split_20261018T150757Z/perception/live_mission.py` | `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/live_mission.py` | **yes** (pipeline diagnostics added) |
| `_rollback/.../phase_sim/vite.config.ts`, `index.html`, `tsconfig.json`, `tsconfig.app.json` | same names under `phase_sim/` | no |
| `_rollback/.../phase_sim/src/**` (all files in the backup) | `phase_sim/src/**` | no (existing files unchanged; the split added new ones, see §4) |
| `_rollback/.../autokart_apps/run_{core,lab,live,tests}.sh` | `autokart_apps/` | no |

Re-check before restoring (the tree is being edited in parallel):

```bash
B=_rollback/ui_split_20261018T150757Z
for f in $(cd $B/phase_sim && find . -type f); do cmp -s $B/phase_sim/$f phase_sim/$f || echo "differs: phase_sim/${f#./}"; done
cmp -s $B/perception/live_mission.py autonomy_stack_phase1/phase_01_perception/src/autokart_perception/live_mission.py || echo "differs: live_mission.py"
```

Restore (keeps the current versions in a timestamped folder):

```bash
B=_rollback/ui_split_20261018T150757Z
K=_rollback/pre_restore_$(date -u +%Y%m%dT%H%M%SZ); mkdir -p "$K/phase_sim" "$K/perception"
cp -p phase_sim/package.json phase_sim/tsconfig.node.json "$K/phase_sim/"
cp -p autonomy_stack_phase1/phase_01_perception/src/autokart_perception/live_mission.py "$K/perception/"
cp -p $B/phase_sim/package.json $B/phase_sim/tsconfig.node.json phase_sim/
cp -p $B/perception/live_mission.py autonomy_stack_phase1/phase_01_perception/src/autokart_perception/live_mission.py
# only if other files show as "differs" in the check above:
#   cp -p $B/phase_sim/<file> phase_sim/<file>
```

## 4. Files the split added (leave in place, or move aside)

They are inert for the legacy app, so leaving them is fine. If they get in the way (e.g. `tsc -b`
after restoring `tsconfig.node.json`), move them — do not delete:

- `phase_sim/apps/` (`dashboard/index.html`, `lab/index.html`)
- `phase_sim/src/apps/` (dashboard + lab sources) and `phase_sim/src/test/labApp.test.tsx`
- `phase_sim/vite.dashboard.config.ts`, `phase_sim/vite.lab.config.ts`
- `phase_sim/dist-dashboard/`, `phase_sim/dist-lab/` (build output)
- `scripts/`, `deploy/systemd/*.service.example`, `docs/ui_split/{OPERATIONS,ROLLBACK}.md`
- `.run/` (pids/logs), `.ros_build/` (repo-local ROS build; harmless, also usable by legacy flows)

```bash
K=_rollback/split_aside_$(date -u +%Y%m%dT%H%M%SZ); mkdir -p "$K/phase_sim/src"
mv phase_sim/apps phase_sim/vite.dashboard.config.ts phase_sim/vite.lab.config.ts "$K/phase_sim/"
mv phase_sim/src/apps "$K/phase_sim/src/"
```

## 5. Verify the rollback

```bash
cd phase_sim && npm run test && npm run build && cd ..
autokart_apps/run_live.sh --mode sim       # then open http://127.0.0.1:8768/live, Ctrl-C to stop
```

Roll *forward* again by reversing §3/§4 from the `pre_restore_*` / `split_aside_*` folders.
