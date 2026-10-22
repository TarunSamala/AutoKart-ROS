#!/usr/bin/env bash
# Table of every AutoKart dashboard/lab process: pid, alive, port listening, health.
#   scripts/status.sh [--dashboard|--lab]
# Informational: exits 0. Ports listening but "pid -" means a process NOT started by these scripts.
set -euo pipefail
# shellcheck source=autokart_env.sh
source "$(dirname "${BASH_SOURCE[0]}")/autokart_env.sh"

WHICH=all
case "${1:-}" in --dashboard) WHICH=dashboard ;; --lab) WHICH=lab ;; "") ;; *) ak_err "usage: $0 [--dashboard|--lab]"; exit 2 ;; esac

# name|port|health-url|stack
ROWS=(
  "dash-perception|$AUTOKART_PERCEPTION_PORT|http://127.0.0.1:$AUTOKART_PERCEPTION_PORT/health|dashboard"
  "dash-gateway|$AUTOKART_LIVE_GATEWAY_PORT|http://127.0.0.1:$AUTOKART_LIVE_GATEWAY_PORT/mission/api/status|dashboard"
  "dash-safety|-|-|dashboard"
  "dash-ui|$AUTOKART_DASHBOARD_PORT|http://127.0.0.1:$AUTOKART_DASHBOARD_PORT/|dashboard"
  "lab-core|$AUTOKART_LAB_CORE_PORT|http://127.0.0.1:$AUTOKART_LAB_CORE_PORT/api/core/status|lab"
  "lab-gateway|$AUTOKART_LAB_GATEWAY_PORT|http://127.0.0.1:$AUTOKART_LAB_GATEWAY_PORT/mission/api/status|lab"
  "lab-ui|$AUTOKART_LAB_PORT|http://127.0.0.1:$AUTOKART_LAB_PORT/|lab"
)

printf '%-16s %-8s %-6s %-6s %-10s %-6s %s\n' PROCESS PID ALIVE PORT LISTEN HTTP DETAIL
printf '%-16s %-8s %-6s %-6s %-10s %-6s %s\n' ---------------- -------- ------ ------ ---------- ------ ------
for row in "${ROWS[@]}"; do
  IFS='|' read -r name port url stack <<< "$row"
  [[ "$WHICH" == all || "$WHICH" == "$stack" ]] || continue
  pid="$(ak_pid_of "$name")"
  alive=no; [[ -n "$pid" ]] && alive=yes
  listen=-; code=-; detail=""
  if [[ "$port" != - ]]; then
    if ak_port_listening "$port"; then
      if [[ -n "$pid" ]] && ak_port_owned_by_group "$pid" "$port"; then listen=yes; else listen="FOREIGN"; detail="port held by pid(s) $(ak_port_pids "$port")"; fi
    else
      listen=no
    fi
    if [[ "$listen" != no ]]; then
      body="$(curl -s -m 3 -w '\n%{http_code}' "$url" 2>/dev/null || true)"
      code="${body##*$'\n'}"; body="${body%$'\n'*}"
      case "$name" in
        dash-perception) detail="${detail:+$detail; }$(printf '%s' "$body" | tr -d '\n' | head -c 90)" ;;
        *gateway) detail="${detail:+$detail; }$(printf '%s' "$body" | python3 -c 'import json,sys
try:
  d=json.load(sys.stdin); print("source=%s ros_connected=%s vehicle_authorized=%s"%(d.get("source"),d.get("ros_connected"),d.get("vehicle_authorized")))
except Exception: print("non-JSON")' 2>/dev/null)" ;;
        lab-core) detail="${detail:+$detail; }$(printf '%s' "$body" | tr -d '\n' | head -c 90)" ;;
      esac
    fi
  else
    [[ -n "$pid" ]] && detail="ROS node, DIAGNOSTIC_ONLY (no port; see scripts/audit_no_actuation.sh)"
  fi
  printf '%-16s %-8s %-6s %-6s %-10s %-6s %s\n' "$name" "${pid:--}" "$alive" "$port" "$listen" "$code" "$detail"
done
echo "pid files: $AUTOKART_PID_DIR   logs: $AUTOKART_LOG_DIR"
