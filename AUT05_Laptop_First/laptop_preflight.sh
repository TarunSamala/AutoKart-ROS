#!/usr/bin/env bash
# Read-only workstation inspection: no installation, USB capture, Pi access, or actuation.
set -uo pipefail
ROOT="${AUTOKART_ROOT:-$HOME/Desktop/autonomous_kart}"
printf 'AUT-05 LAPTOP FIRST PREFLIGHT\nroot=%s\n' "$ROOT"
if [[ ! -d "$ROOT" ]]; then echo "BLOCKED: project root missing: $ROOT"; exit 2; fi
printf '\n===== MACHINE =====\n'
uname -srm
printf 'DISPLAY=%s WAYLAND_DISPLAY=%s\n' "${DISPLAY:-}" "${WAYLAND_DISPLAY:-}"
for cmd in python3 g++ cmake pkg-config readelf git lsusb v4l2-ctl node npm codex; do
 if command -v "$cmd" >/dev/null 2>&1; then printf 'FOUND %-14s %s\n' "$cmd" "$(command -v "$cmd")"; else printf 'MISSING %s\n' "$cmd"; fi
done
printf '\n===== AUT-03 FILES =====\n'
for f in \
 "$ROOT/autokart_apps/run_core.sh" \
 "$ROOT/autokart_apps/run_lab.sh" \
 "$ROOT/autonomy_stack_phase1/phase_01_perception/pyproject.toml" \
 "$ROOT/autonomy_stack_phase1/phase_01_perception/src/autokart_perception/sdk.py" \
 "$ROOT/autonomy_stack_phase1/phase_01_perception/src/autokart_perception/camera.py" \
 "$ROOT/autonomy_stack_phase1/phase_01_perception/SDK.md" \
 "$ROOT/phase_sim/package.json"; do
 [[ -f "$f" ]] && echo "FOUND $f" || echo "MISSING $f"
done
printf '\n===== CAMERA WRAPPER CANDIDATES =====\n'
for c in "$ROOT/autonomy_stack_phase1" "$ROOT/YDLIDARHP60C" "$ROOT/external/YDLIDARHP60C"; do
 if [[ -f "$c/run_viewer.sh" && -f "$c/setup.sh" ]]; then
    echo "FOUND_CAMERA_REPO=$c"
    bash -n "$c/run_viewer.sh" "$c/setup.sh" "$c/build.sh" && echo 'camera shell syntax PASS' || echo 'camera shell syntax FAIL'
    if [[ -f "$c/vendor/linux_ros/linux/CMakeLists.txt" ]]; then echo 'VENDOR SOURCE PRESENT'; else echo 'VENDOR SOURCE NOT PREPARED'; fi
 fi
done
printf '\n===== HP60C USB =====\n'
if command -v lsusb >/dev/null 2>&1; then
 lsusb -d 3482:6723 || true
 if lsusb -d 3482:6723 | grep -q '3482:6723'; then echo 'CAMERA ENUMERATED'; else echo 'CAMERA NOT CONNECTED TO LAPTOP (replay development allowed)'; fi
fi
printf '\n===== GIT WORKTREE =====\n'
if git -C "$ROOT" rev-parse --show-toplevel >/dev/null 2>&1; then
 git -C "$ROOT" rev-parse --short HEAD
 git -C "$ROOT" status --short | head -60
else
 echo 'Project root is not a Git repository; check nested repositories before edits'
fi
printf '\nSAFE: inspection only. No Pi operations or vehicle commands performed.\n'
