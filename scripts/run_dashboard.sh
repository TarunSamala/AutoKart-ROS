#!/usr/bin/env bash
# AutoKart Vehicle Dashboard stack (read-only diagnostic; vehicle authority OFF).
#   scripts/run_dashboard.sh [start|stop|status|restart] [--dev-sim] [--prod]
# Starts, each as its own session with its own pid file + log under $ROOT/.run:
#   dash-perception  autokart_perception.live_mission --mode live --camera hp60c  (HP60C single SDK owner)
#   dash-gateway     ros2 run autokart_dashboard_gateway dashboard_gateway --source REAL_KART
#   dash-safety      ros2 run autokart_safety_bridge front_safety_envelope_monitor (DIAGNOSTIC_ONLY, data_source LIVE)
#   dash-ui          npm run dev:dashboard   (or --prod: build:dashboard + preview:dashboard)
# --dev-sim : BENCH ONLY. perception runs --mode sim; the dashboard will show SIMULATED data.
# There is no automatic fallback to sim when the camera is missing -- by design.
# Env: AUTOKART_PERCEPTION_EXTRA_ARGS='...' extra live_mission args (never --mode).
# Env: AUTOKART_SKIP_BUILD=1 with --prod reuses phase_sim/dist-dashboard without rebuilding.
set -euo pipefail
# shellcheck source=autokart_env.sh
source "$(dirname "${BASH_SOURCE[0]}")/autokart_env.sh"

ACTION=start DEV_SIM=0 PROD=0
for arg in "$@"; do
  case "$arg" in
    start|stop|status|restart) ACTION=$arg ;;
    --dev-sim) DEV_SIM=1 ;;
    --prod) PROD=1 ;;
    -h|--help) sed -n '2,14p' "$0"; exit 0 ;;
    *) ak_err "unknown argument: $arg"; exit 2 ;;
  esac
done

COMPONENTS=(dash-perception dash-gateway dash-safety dash-ui)

dev_sim_banner() {
  cat >&2 <<'BANNER'
##########################################################################
##  WARNING: --dev-sim  (BENCH ONLY)                                    ##
##  Perception is running in --mode sim. The Vehicle Dashboard will    ##
##  show SIMULATED camera/perception data, NOT the HP60C.              ##
##  Do NOT use this mode to judge the real vehicle.                    ##
##########################################################################
BANNER
}

# Refuse a second HP60C owner anywhere on this machine (any port), not just on our port.
check_camera_owner() {
  local others
  others="$(pgrep -af 'autokart_perception[.]live_mission' 2>/dev/null | grep -E -- '--mode[ =]live' || true)"
  if [[ -n "$others" ]]; then
    ak_err "another HP60C owner (live_mission --mode live) is already running:"
    printf '         %s\n' "$others" >&2
    ak_err "refusing to start a second camera owner. Stop it first (scripts/run_dashboard.sh stop, or autokart_apps/run_live.sh's terminal)."
    return 1
  fi
  if pgrep -af 'autokart_perception[.]live_mission' >/dev/null 2>&1; then
    ak_warn "a non-live live_mission (sim/replay) is running elsewhere; it does not own the camera:"
    pgrep -af 'autokart_perception[.]live_mission' | sed 's/^/         /' >&2
  fi
}

start_perception() {
  if ak_is_running dash-perception; then
    ak_info "dash-perception already running (pid $(ak_pid_of dash-perception)); not starting a second camera owner"
    return 0
  fi
  if ak_port_listening "$AUTOKART_PERCEPTION_PORT"; then
    ak_err "dash-perception: port $AUTOKART_PERCEPTION_PORT already in use by a FOREIGN process (pid(s): $(ak_port_pids "$AUTOKART_PERCEPTION_PORT")) -- possibly another camera owner."
    ak_err "refusing to start a second camera owner."
    return 3
  fi
  local mode=live
  local -a extra=()
  # Optional extra live_mission args, e.g. "--detector conenet-v2" or "--token-file /etc/autokart/token".
  # --mode is owned by this script and may not be overridden here.
  if [[ -n "${AUTOKART_PERCEPTION_EXTRA_ARGS:-}" ]]; then
    read -r -a extra <<< "$AUTOKART_PERCEPTION_EXTRA_ARGS"
    if [[ " ${extra[*]} " == *" --mode"* ]]; then ak_err "AUTOKART_PERCEPTION_EXTRA_ARGS may not contain --mode (use --dev-sim)"; return 2; fi
  fi
  if ((DEV_SIM)); then
    mode=sim; dev_sim_banner
  else
    check_camera_owner || return 4
  fi
  [[ -f "$ROOT/phase_sim/dist/index.html" ]] || ak_warn "phase_sim/dist missing; live_mission requires it (cd phase_sim && npm run build)"
  ak_start dash-perception "$AUTOKART_PERCEPTION_PORT" 90 \
    "$AUTOKART_PYTHON" -m autokart_perception.live_mission --mode "$mode" --camera hp60c \
    --host "$AUTOKART_BIND_HOST" --port "$AUTOKART_PERCEPTION_PORT" ${extra[@]+"${extra[@]}"}
}

start_ros() {
  ak_source_ros || return 1
  export ROS_DOMAIN_ID="$AUTOKART_LIVE_ROS_DOMAIN_ID"
  ak_info "live ROS domain: ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  local rc=0
  ak_start dash-gateway "$AUTOKART_LIVE_GATEWAY_PORT" 30 \
    ros2 run autokart_dashboard_gateway dashboard_gateway --source REAL_KART \
    --host "$AUTOKART_BIND_HOST" --port "$AUTOKART_LIVE_GATEWAY_PORT" || rc=$?
  # Uses the package's DIAGNOSTIC_ONLY config; no authority parameter is ever overridden here.
  ak_start dash-safety - 3 \
    ros2 run autokart_safety_bridge front_safety_envelope_monitor --ros-args \
    --params-file "$AUTOKART_ROS_INSTALL/autokart_safety_bridge/share/autokart_safety_bridge/config/front_safety_envelope_monitor.yaml" \
    -p data_source:=$( ((DEV_SIM)) && echo SIMULATED || echo LIVE ) || rc=$?
  return "$rc"
}

start_ui() {
  cd "$ROOT/phase_sim"
  if ((PROD)); then
    if [[ "${AUTOKART_SKIP_BUILD:-0}" != 1 ]] && ! ak_is_running dash-ui; then
      ak_info "building dashboard (npm run build:dashboard)..."
      npm run build:dashboard >> "$(ak_logfile dash-ui-build)" 2>&1 || {
        ak_err "build:dashboard failed; see $(ak_logfile dash-ui-build)"; tail -n 20 "$(ak_logfile dash-ui-build)" >&2; return 1; }
    fi
    ak_start dash-ui "$AUTOKART_DASHBOARD_PORT" 30 npm run preview:dashboard
  else
    ak_start dash-ui "$AUTOKART_DASHBOARD_PORT" 30 npm run dev:dashboard
  fi
}

do_start() {
  local rc=0
  start_perception || rc=$?
  if ((rc != 0)); then
    # Abort: never let the dashboard UI come up proxied to some other (foreign/sim) perception service.
    ak_err "perception service (camera owner) not started (rc=$rc); aborting dashboard start. Nothing else was started."
    return "$rc"
  fi
  start_ros || { rc=1; ak_err "ROS part failed"; }
  start_ui || { rc=1; ak_err "dashboard UI failed"; }
  ((DEV_SIM)) && dev_sim_banner
  echo
  ak_info "Vehicle Dashboard: http://$(ak_host_ip):${AUTOKART_DASHBOARD_PORT}/overview   (local: http://127.0.0.1:${AUTOKART_DASHBOARD_PORT}/overview)"
  ak_info "vehicle authority: OFF (read-only diagnostic)"
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
  status) exec "$(dirname "${BASH_SOURCE[0]}")/status.sh" --dashboard ;;
esac
