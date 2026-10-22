#!/usr/bin/env bash
# AutoKart Algorithm Lab stack (simulation/replay only; never touches the live vehicle).
#   scripts/run_lab.sh [start|stop|status|restart] [--prod]
#   lab-core     autokart_perception.lab_api (port 8767)
#   lab-gateway  ROS_DOMAIN_ID=42 dashboard_gateway --source SIMULATION (port 8775)
#   lab-ui       npm run dev:lab   (or --prod: build:lab + preview:lab), port 5181
# Never starts anything on the live ports (8765/8768) or in the live ROS domain.
# Env: AUTOKART_SKIP_BUILD=1 with --prod reuses phase_sim/dist-lab without rebuilding.
set -euo pipefail
# shellcheck source=autokart_env.sh
source "$(dirname "${BASH_SOURCE[0]}")/autokart_env.sh"

ACTION=start PROD=0
for arg in "$@"; do
  case "$arg" in
    start|stop|status|restart) ACTION=$arg ;;
    --prod) PROD=1 ;;
    -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
    *) ak_err "unknown argument: $arg"; exit 2 ;;
  esac
done

COMPONENTS=(lab-core lab-gateway lab-ui)

guard_isolation() {
  local p
  for p in "$AUTOKART_LAB_CORE_PORT" "$AUTOKART_LAB_GATEWAY_PORT" "$AUTOKART_LAB_PORT"; do
    if [[ "$p" == 8765 || "$p" == 8768 || "$p" == "$AUTOKART_PERCEPTION_PORT" || "$p" == "$AUTOKART_LIVE_GATEWAY_PORT" || "$p" == "$AUTOKART_DASHBOARD_PORT" ]]; then
      ak_err "Lab port $p collides with a live/dashboard port; refusing."; exit 2
    fi
  done
  if [[ "$AUTOKART_LAB_ROS_DOMAIN_ID" == "$AUTOKART_LIVE_ROS_DOMAIN_ID" ]]; then
    ak_err "Lab ROS domain ($AUTOKART_LAB_ROS_DOMAIN_ID) equals live domain; refusing (Lab must never touch live topics)."; exit 2
  fi
}

do_start() {
  local rc=0
  guard_isolation
  ak_start lab-core "$AUTOKART_LAB_CORE_PORT" 30 \
    "$AUTOKART_PYTHON" -m autokart_perception.lab_api --host 127.0.0.1 --port "$AUTOKART_LAB_CORE_PORT" || rc=1
  if ak_source_ros; then
    ak_info "lab ROS domain: ROS_DOMAIN_ID=$AUTOKART_LAB_ROS_DOMAIN_ID (isolated from live domain $AUTOKART_LIVE_ROS_DOMAIN_ID)"
    ak_start lab-gateway "$AUTOKART_LAB_GATEWAY_PORT" 30 \
      env ROS_DOMAIN_ID="$AUTOKART_LAB_ROS_DOMAIN_ID" \
      ros2 run autokart_dashboard_gateway dashboard_gateway --source SIMULATION \
      --host "$AUTOKART_BIND_HOST" --port "$AUTOKART_LAB_GATEWAY_PORT" || rc=1
  else
    rc=1
  fi
  cd "$ROOT/phase_sim"
  if ((PROD)); then
    if [[ "${AUTOKART_SKIP_BUILD:-0}" != 1 ]] && ! ak_is_running lab-ui; then
      ak_info "building lab (npm run build:lab)..."
      npm run build:lab >> "$(ak_logfile lab-ui-build)" 2>&1 || {
        ak_err "build:lab failed; see $(ak_logfile lab-ui-build)"; tail -n 20 "$(ak_logfile lab-ui-build)" >&2; rc=1; }
    fi
    if [[ -f "$ROOT/phase_sim/dist-lab/index.html" ]]; then
      ak_start lab-ui "$AUTOKART_LAB_PORT" 30 npm run preview:lab || rc=1
    else
      ak_err "phase_sim/dist-lab/index.html missing; cannot preview"; rc=1
    fi
  else
    ak_start lab-ui "$AUTOKART_LAB_PORT" 30 npm run dev:lab || rc=1
  fi
  echo
  ak_info "Algorithm Lab: http://$(ak_host_ip):${AUTOKART_LAB_PORT}/home   (local: http://127.0.0.1:${AUTOKART_LAB_PORT}/home)"
  ak_info "SIMULATION / replay only -- no live vehicle data, no vehicle authority"
  return "$rc"
}

do_stop() {
  local i
  for ((i = ${#COMPONENTS[@]} - 1; i >= 0; i--)); do ak_stop "${COMPONENTS[i]}"; done
}

case "$ACTION" in
  start) do_start ;;
  stop) do_stop ;;
  restart) do_stop; do_start ;;
  status) exec "$(dirname "${BASH_SOURCE[0]}")/status.sh" --lab ;;
esac
