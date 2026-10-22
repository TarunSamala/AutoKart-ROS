# RC-01 cleanup diff

## Scope and result

Repository hygiene only. No autonomy, control, model, firmware, safety, pin map, ROS message or vehicle-authorization behavior was changed. No workspace path was moved or deleted, no Git worktree was removed, and no repository reset/clean was run. This is a documentation/index addition plus a verified external copy of historical worktrees for preservation.

## Files created or copied in the workspace

- `PROJECT_INDEX.md`
- `docs/architecture/REPOSITORY_MAP.md`
- `docs/decisions/CANONICAL_COMPONENTS.md`
- `docs/decisions/REPOSITORY_DISPOSITION.md`
- `docs/decisions/manifest.csv`
- `docs/runbooks/VERIFY_REPOSITORY.md`
- `docs/audits/2026-10-16/01_SYSTEM_INVENTORY.md`
- `docs/audits/2026-10-16/02_END_TO_END_ARCHITECTURE.md`
- `docs/audits/2026-10-16/03_DEFECT_REGISTER.md`
- `docs/audits/2026-10-16/04_TEST_EVIDENCE.md`
- `docs/audits/2026-10-16/05_COMPLETION_PLAN.md`
- `docs/audits/2026-10-16/CLEANUP_DIFF.md`
- `docs/audits/2026-10-16/SYMLINK_INVENTORY.csv`

The copied audit originals under `_autokart_full_audit/` remain in place. No existing project source/config/test file was edited.

## Baseline-generated outputs

The requested pre-change `npm run build` created/updated `phase_sim/dist/` and TypeScript incremental files under `phase_sim/`. These are build outputs from the baseline verification, not application source changes. `autokart_apps/run_live.sh` references `phase_sim/dist/index.html` and rebuilds if it is absent/stale; no such output was deleted. ROS build/install/log outputs for RC-01 were directed to `/tmp/autokart_rc01_ros_*`.

The workspace was 10,050,320,925 bytes immediately before the documentation additions (state reconstructed by excluding the new root `PROJECT_INDEX.md` and root `docs/` from the after scan; it includes the build outputs created by the requested baseline run). At this report update it is 10,050,480,773 bytes: an increase of 159,848 bytes. These are apparent workspace bytes from `du -sb`; symlinks are not followed. The backup occupies 3.7 MiB. At final check, the root filesystem had 7.5 GiB free.

## Historical-worktree backup and checksums

Backup directory: `/home/mark1/Desktop/autokart_backups/RC-01-prearchive-2026-10-16/` (outside the project root, on the same physical filesystem).

- `autonomy_stack_phase1-all-refs.bundle`: includes all local branches and refs; `git bundle verify` reported a complete history and verified all 11 camera branch refs plus `main`.
- `camera-worktrees-working-copy.tar.gz`: copies the 11 linked worktree directories, including working-tree changes/untracked files and symlinks; `tar -tzf` succeeded.
- `SHA256SUMS`: checksum list; `sha256sum -c` passed for both backup artifacts.
- Bundle SHA-256: `18b18d2b657b5d219f826e2994f06e2111da5ffe04f0dde20d57d0125559a15c`.
- Worktree tar SHA-256: `e78e3c0a82b67bcc5a4fdd1bae780a9b2cf3a2da4fb2e0b73e5723ebd1e7df56`.
- A separate temporary clone from the bundle read a committed worktree file; extraction recovered the untracked `calibration_runtime.py` from `_camera_prod_a1` with the same SHA-256 as its source.

This backup is readable and recoverable on the current filesystem. It is not an off-device backup. The source worktrees are unchanged.

## Verification results

Baseline and post-documentation checks both passed: perception 139; kart 149; LAB 32; ROS direct contracts 10; ROS build 13 packages; ESP32 static 9; nested WebStop static checks printed PASS. Post-documentation runtimes/counts and exact commands are in `../../runbooks/VERIFY_REPOSITORY.md`. The frontend build succeeded both times with the same large-bundle advisory. Before documentation, ROS `colcon test` completed 13 packages but only the sim bridge ran 2 tests; dashboard gateway ran 0 and Gazebo pytest was unavailable. No firmware build was attempted because `idf.py` is absent. These are before/after documentation checks; because no cleanup move or cache removal has been executed, they are not a post-cleanup comparison. Any approved archive/cache operation still requires its listed verification afterward.

## Disposition and blocked decisions

- The 11 `_camera_*` paths are linked worktrees and contain distinct commit histories; some have untracked or modified implementation files. They are archive candidates, not proven redundant. No move is authorized by this report alone.
- Proposed destination if approved: `/home/mark1/Desktop/autokart_archives/2026-10-16/`.
- Approval is required for each exact path listed in `docs/decisions/REPOSITORY_DISPOSITION.md`, as well as the destination, before any Git worktree removal or archive action.
- Firmware and E-stop paths remain `UNKNOWN_DO_NOT_TOUCH`; built firmware images include previous-image backup binaries. No build tree containing those files is proposed for cleanup.
- `.venv*`, source data, datasets, training outputs, calibration, recordings, safety evidence and ROS log trees remain preserved. Only TypeScript build metadata is marked as a regenerable-cache candidate; nothing was cleaned.
- Current canonical deployed firmware, Pi checkout, IMU, sensor wiring, physical E-stop and brake behavior remain unknown/unverified.
- The host showed no active AutoKart user service or application/ROS process at the inspection point. This does not describe a separate Raspberry Pi.

No vehicle operation or hardware motion test was attempted. Vehicle authority remains false.

## Rollback instructions

- **No cleanup rollback is currently needed:** the active source paths, snapshot worktrees and generated cache files remain in place. This change adds navigation and planning documents only.
- **If the 11 linked worktrees are later archived with approval:** restore their working-tree contents from `/home/mark1/Desktop/autokart_backups/RC-01-prearchive-2026-10-16/camera-worktrees-working-copy.tar.gz` and Git refs from `autonomy_stack_phase1-all-refs.bundle`; re-register each branch with `git worktree add`. Check each restored tree against the matching `checksum_sha256` row in `docs/decisions/manifest.csv`, then inspect `git worktree list --porcelain` and rerun the Phase 1 suite.
- **If the two TypeScript incremental files are later removed with approval:** run `npm run build` from `phase_sim/` to recreate them, then run the LAB tests and build commands in `docs/runbooks/VERIFY_REPOSITORY.md`.
- **To revert the documentation addition itself:** restore the pre-change copies from the external backup if separately captured, or remove only the exact RC-01 files listed in the “Files created or copied” section above plus `PROJECT_INDEX.md`. Do not remove the original reports under `_autokart_full_audit/` or the external backup.

## Proposed minimal follow-up, ordered by path/import/service risk

1. **Highest Git/source-of-truth risk:** capture the active `autonomy_stack_phase1/phase_01_perception/` working source in a reviewed commit or equivalent immutable source snapshot. Current Git `main` does not track that directory, so a commit hash alone cannot identify the tested source.
2. **High launcher/model risk:** decide and document the per-mode diagnostic model selection without changing defaults by assumption. The current `run_live.sh` and example service resolve to ConeNet v1; the frozen YOLO26s model is explicit and experimental. Verify `PYTHONPATH`, `phase_sim/dist` and model-manifest paths before any service deployment.
3. **High Git metadata risk:** after explicit approval for the 11 exact linked-worktree paths, archive them with Git-aware handling, preserving the branch bundle and all modified/untracked files. Do not use plain `mv` on live worktrees.
4. **Medium path risk:** review root project docs and service examples for stale paths after any separately approved archive; keep imports, launch files, model paths and UI assets at their current locations for this freeze.
5. **Later technical integration, outside RC-01:** separately identify the Pi image, IMU, camera calibration/mount, installed ESP32 image and wiring; then plan ROS telemetry/planning/control integration under physical safety gates. Do not implement the algorithms or VCU bridge as part of repository consolidation.
