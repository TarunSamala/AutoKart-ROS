#!/usr/bin/env bash
# Start the phase-1 ROS graph and Mission Control. All verification gates ship false.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export AUTOKART_ROOT="$ROOT"
OVERLAY="${AUTOKART_PHASE1_OVERLAY:-$ROOT/.ros_phase1_overlay/install}"
RUN="$ROOT/.run/phase1"
mkdir -p "$RUN"
[[ -f "$OVERLAY/setup.bash" ]] || { echo "NO-GO: separate phase-1 overlay missing: $OVERLAY" >&2; exit 1; }
[[ -f "$ROOT/models/yolo26s/best.onnx" && -f "$ROOT/models/yolo26s/best.json" ]] || { echo "NO-GO: YOLO26s model/manifest missing" >&2; exit 1; }
[[ -f "$ROOT/autonomy_stack_phase1/phase_01_perception/native/build-x86_64-linux-gnu/libhp60c_bridge.so" ]] || { echo "NO-GO: HP60C native bridge missing" >&2; exit 1; }
[[ -e "${AUTOKART_VCU_PORT:-/dev/ttyACM0}" ]] || { echo "NO-GO: VCU serial device missing" >&2; exit 1; }
# Idempotent: calling start when the graph is already up is a safe no-op success (exit 0),
# not an error -- the desired end state (graph running) already holds either way.
if [[ -f "$RUN/ros.pid" ]] && kill -0 "$(cat "$RUN/ros.pid")" 2>/dev/null; then
  echo "Phase-1 ROS graph already running (PID $(cat "$RUN/ros.pid")); nothing to do."
  exit 0
fi
# Rotate, don't silently overwrite, the previous run's logs; keep the last 10 per name so a long
# sequence of restarts cannot grow the logs directory without bound.
for name in ros ui; do
  if [[ -f "$RUN/$name.log" ]]; then
    mv "$RUN/$name.log" "$RUN/$name.log.$(date -u +%Y%m%dT%H%M%SZ)"
  fi
  mapfile -t rotated < <(ls -1t "$RUN/$name.log."* 2>/dev/null || true)
  if (( ${#rotated[@]} > 10 )); then
    printf '%s\n' "${rotated[@]:10}" | xargs -r rm -f
  fi
done
if pgrep -af 'autokart_perception[.]live_mission.*--mode live' >/dev/null; then
  echo "NO-GO: another HP60C owner is running" >&2; exit 1
fi
python3 - <<'PY'
import socket
for port in (8765, 5173):
    with socket.socket() as sock:
        try:
            sock.bind(("127.0.0.1", port))
        except OSError as exc:
            raise SystemExit(f"NO-GO: local port {port} unavailable: {exc}")
PY
# The workspace source is required by the ROS adapters; no copy of perception is started.
set +u
source /opt/ros/humble/setup.bash
source "$OVERLAY/setup.bash"
set -u
export PYTHONPATH="$ROOT/autonomy_stack_phase1/phase_01_perception/src:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${AUTOKART_LIVE_ROS_DOMAIN_ID:-0}"
setsid ros2 launch autokart_bringup phase1_vehicle.launch.py >"$RUN/ros.log" 2>&1 &
ROS_PID=$!
echo "$ROS_PID" >"$RUN/ros.pid"
if ! kill -0 "$ROS_PID" 2>/dev/null; then echo "NO-GO: ROS launch exited; inspect $RUN/ros.log" >&2; exit 1; fi
pushd "$ROOT/phase_sim" >/dev/null
setsid npm run dev -- --host 127.0.0.1 --port 5173 >"$RUN/ui.log" 2>&1 &
UI_PID=$!
popd >/dev/null
echo "$UI_PID" >"$RUN/ui.pid"
if ! kill -0 "$UI_PID" 2>/dev/null; then
  echo "Mission Control exited; inspect $RUN/ui.log" >&2
  "$ROOT/scripts/stop_phase1.sh"
  exit 1
fi
echo "Started phase-1 graph (PID $ROS_PID) and Mission Control at http://127.0.0.1:5173/ops. Vehicle remains disarmed by default."
echo "Run scripts/system_check.sh for a live NO-GO/GO report before any operator arming request."
