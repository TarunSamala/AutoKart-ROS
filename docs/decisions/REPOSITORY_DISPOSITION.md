# Repository disposition (RC-01)

This is a non-destructive classification. Active source paths stay in place. No archive move, worktree removal, deletion, reset or `git clean` was performed. The machine-readable row-level register is [`manifest.csv`](manifest.csv).

## Keep active

- `autokart_apps/`, `autonomy_stack_phase1/phase_01_perception/`, `kart/kart_sim/`, `kart/ros2_ws/`, `kart/configs/`, `kart/tests/` and `phase_sim/`: active source, entry points, contracts and current integration work. Keep their current paths.
- `phase_sim/node_modules/`: currently available dependency tree used by the verified LAB tests/build. No install/update was run.
- `phase_sim/dist/`: generated output that `autokart_apps/run_live.sh` checks and can rebuild; current launcher consumes it when serving. Keep for this frozen release.

## Keep as reference

- `_autokart_audit/`, `_autokart_full_audit/`, `docs/` and `AUT05_Laptop_First/`: audits, saved measurements, runbooks and read-only preflight/regression scripts.
- `bench/`, `models/`, `training_runs/`, `datasets/`, perception `recordings/`, `kart/artifacts/`, firmware binaries and backup binaries, firmware ZIPs, pin/power spreadsheet and supplied specifications: preserve source and provenance even when not part of the runtime.
- `kart/ros2_ws/log/` and `kart/log/`: existing `latest*` symlinks point to dated test/build records. Do not clear or rewrite them.
- `AutoKart_RC_Dynamics_ESP32S3_v1/` and `XIAO_C3_RA02_ESTOP_PRODUCTION/`: **UNKNOWN_DO_NOT_TOUCH**. Installed image, wiring and stop-path status are unresolved; similarly named binaries/builds are not interchangeable.
- Root `README.txt`, project prompt/spec files and ZIP archives: preserve as supplied references.

## Archive candidates: approval required, not moved

All of the following are linked worktrees in `autonomy_stack_phase1/.git`, not independent repositories. They have branches and Git history; some contain modified tracked files and unique untracked artifacts. Source comparison found, for example, 4–7 changed Phase 1 modules in the `_camera_a*` family versus the current local source, extra calibration code in `_camera_prod_a1`, two modified modules in `_camera_prod_a2`, three modified browser files and generated output in `_camera_prod_a4`, and a modified live mission plus model files in `_camera_team_base`. The production branches share 15 commits ahead of current `main`; the A branches have 3–6 commits ahead; team base has 16. These differences and untracked content rule out automatic cleanup.

Proposed archive destination: `/home/mark1/Desktop/autokart_archives/2026-10-16/`. Exact candidate paths:

- `/home/mark1/Desktop/autonomous_kart/_camera_a1/`
- `/home/mark1/Desktop/autonomous_kart/_camera_a2/`
- `/home/mark1/Desktop/autonomous_kart/_camera_a3/`
- `/home/mark1/Desktop/autonomous_kart/_camera_a4/`
- `/home/mark1/Desktop/autonomous_kart/_camera_a5/`
- `/home/mark1/Desktop/autonomous_kart/_camera_prod_a1/`
- `/home/mark1/Desktop/autonomous_kart/_camera_prod_a2/`
- `/home/mark1/Desktop/autonomous_kart/_camera_prod_a3/`
- `/home/mark1/Desktop/autonomous_kart/_camera_prod_a4/`
- `/home/mark1/Desktop/autonomous_kart/_camera_prod_a5/`
- `/home/mark1/Desktop/autonomous_kart/_camera_team_base/`

### Snapshot worktree heads and observed local changes

| Worktree path | Branch / HEAD | Observed status |
|---|---|---|
| `_camera_a1/` | `camera-a1` / `d3d5307723e239912c4a6007e5703334358dbdb2` | Untracked Python bytecode cache files. |
| `_camera_a2/` | `camera-a2` / `7493e0bd7de82ec7ca8a08fc8cda153a7a68f374` | Clean worktree. |
| `_camera_a3/` | `camera-a3` / `b3b0fb6ecabd69574b05e324c87bd1b804574f49` | Untracked Python bytecode cache files. |
| `_camera_a4/` | `camera-a4` / `07714c46cbc15d0465a382ff108e9ebb707f7306` | Clean worktree. |
| `_camera_a5/` | `camera-a5` / `f3408cc7583c453a7e8c2eb7c797bf3d815b44de` | Clean worktree. |
| `_camera_prod_a1/` | `camera-prod-a1` / `eeaf8bb2e0456f8473d1af7499fd36178a6b1226` | Untracked `calibration_runtime.py`, its test and Python bytecode caches. |
| `_camera_prod_a2/` | `camera-prod-a2` / `eeaf8bb2e0456f8473d1af7499fd36178a6b1226` | Modified depth/pipeline modules and tests; untracked A2 report and caches. |
| `_camera_prod_a3/` | `camera-prod-a3` / `eeaf8bb2e0456f8473d1af7499fd36178a6b1226` | Clean worktree. |
| `_camera_prod_a4/` | `camera-prod-a4` / `eeaf8bb2e0456f8473d1af7499fd36178a6b1226` | Modified live UI/CSS/test; untracked build output and TypeScript metadata. |
| `_camera_prod_a5/` | `camera-prod-a5` / `eeaf8bb2e0456f8473d1af7499fd36178a6b1226` | Clean worktree. |
| `_camera_team_base/` | `camera-team-base` / `29c2ab47c0fc1a38e33df6a1619b050bbd5c83f2` | Modified live mission; untracked model paths, build output and caches. |

The main worktree is `main` at `87f2e288cb1505a9eb233e077603f19689cc3114`; its tracked `.gitignore`, `README.md` and `setup.sh` are modified and its Phase 1 source, docs and runs are untracked. All of this remains as found. The external Git bundle preserves all branch pointers.

Before any archive execution, obtain approval for this exact path list and destination. Because these are Git linked worktrees, a later operation must use a Git-aware reversible sequence and retain the external branch bundle plus modified/untracked content. Do not use `mv` on a live worktree or edit any `.git` pointer directly.

An independent-to-the-workspace, readable backup copy was created at `/home/mark1/Desktop/autokart_backups/RC-01-prearchive-2026-10-16/` before documentation changes. It contains an all-refs Git bundle and a tar of the snapshot worktree files; `git bundle verify`, `tar -tzf` and `sha256sum -c SHA256SUMS` succeeded. The backup is on the same physical filesystem, so it is a recoverable local copy, not off-device disaster recovery.

## Generated rebuildable candidates

- `phase_sim/tsconfig.app.tsbuildinfo` and `phase_sim/tsconfig.node.tsbuildinfo`: TypeScript incremental metadata; generated by the successful frontend build. They are not runtime source. They remain in place; the manifest records removal as approval-gated.
- `__pycache__/` and `.pytest_cache/` under historical worktrees: generated test/interpreter artifacts. Existing snapshot worktrees may have untracked bytecode; preserve the complete snapshot archive until approval rather than removing these paths separately.

No cache was cleaned. Root and Phase 1 virtual environments, firmware `build/` outputs (which include previous binary backups), ROS build/install/log trees and model artifacts remain kept or unknown; their use/evidence content prevents a safe blanket cache classification.

## Approval boundary

No source or generated file has been moved or removed. Archival of the exact 11 linked worktree paths above requires approval of that exact list and destination. Removal of the two TypeScript build metadata files is a separate, optional approval item; the successful frontend build can recreate them. Ambiguous firmware, E-stop, environment, build, model, dataset and evidence paths are not cleanup targets. If approval is not given, all remain exactly where they are.

## Rollback reference

For any later approved worktree archive, restore working files from `/home/mark1/Desktop/autokart_backups/RC-01-prearchive-2026-10-16/camera-worktrees-working-copy.tar.gz` and refs from `autonomy_stack_phase1-all-refs.bundle`, then re-register each branch with `git worktree add`. Validate the restored directory tree against its manifest SHA-256 and confirm `git worktree list --porcelain`. For approved TypeScript metadata removal, run `npm run build` in `phase_sim/`. No rollback is needed for this plan itself because it has not moved or removed project files.
