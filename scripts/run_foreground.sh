#!/usr/bin/env bash
# Foreground runner for systemd (deploy/systemd/*.service.example). Not for interactive use --
# use run_dashboard.sh / run_lab.sh instead. Runs the group in the foreground; if ANY child exits,
# all are terminated and this exits non-zero so systemd's Restart=on-failure restarts the group.
#   scripts/run_foreground.sh perception   # HP60C owner, --mode live (never sim)
#   scripts/run_foreground.sh dashboard    # live gateway + safety monitor (DIAGNOSTIC_ONLY) + dashboard preview UI
#   scripts/run_foreground.sh lab          # lab core + lab SIMULATION gateway (domain 42) + lab preview UI
# UIs are served from the prebuilt dist-dashboard / dist-lab (npm run build:apps beforehand).
set -euo pipefail
# shellcheck source=autokart_env.sh
source "$(dirname "${BASH_SOURCE[0]}")/autokart_env.sh"

pids=()
# Each child runs in its own session (setsid; pid == pgid), so cleanup kills whole trees
# (ros2 run -> node process, npm -> sh -> node). systemd KillMode=control-group is the backstop.
cleanup() {
  trap - EXIT INT TERM
  local p
  for p in "${pids[@]}"; do kill -TERM -- "-$p" 2>/dev/null || true; done
  sleep 3
  for p in "${pids[@]}"; do kill -KILL -- "-$p" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
need_free() { if ak_port_listening "$1"; then ak_err "port $1 already in use (pid(s): $(ak_port_pids "$1")); refusing"; exit 3; fi; }

case "${1:-}" in
  perception)
    need_free "$AUTOKART_PERCEPTION_PORT"
    if pgrep -af 'autokart_perception[.]live_mission' | grep -qE -- '--mode[ =]live'; then
      ak_err "another HP60C owner is already running; refusing"; exit 4
    fi
    exec "$AUTOKART_PYTHON" -m autokart_perception.live_mission --mode live --camera hp60c \
      --host "$AUTOKART_BIND_HOST" --port "$AUTOKART_PERCEPTION_PORT"
    ;;
  dashboard)
    need_free "$AUTOKART_LIVE_GATEWAY_PORT"; need_free "$AUTOKART_DASHBOARD_PORT"
    ak_source_ros
    export ROS_DOMAIN_ID="$AUTOKART_LIVE_ROS_DOMAIN_ID"
    setsid ros2 run autokart_dashboard_gateway dashboard_gateway --source REAL_KART \
      --host "$AUTOKART_BIND_HOST" --port "$AUTOKART_LIVE_GATEWAY_PORT" & pids+=($!)
    setsid ros2 run autokart_safety_bridge front_safety_envelope_monitor --ros-args \
      --params-file "$AUTOKART_ROS_INSTALL/autokart_safety_bridge/share/autokart_safety_bridge/config/front_safety_envelope_monitor.yaml" \
      -p data_source:=LIVE & pids+=($!)
    setsid bash -c 'cd "$1" && exec npm run preview:dashboard' _ "$ROOT/phase_sim" & pids+=($!)
    ;;
  lab)
    for p in "$AUTOKART_LAB_CORE_PORT" "$AUTOKART_LAB_GATEWAY_PORT" "$AUTOKART_LAB_PORT"; do
      [[ "$p" != 8765 && "$p" != 8768 ]] || { ak_err "lab may not use live port $p"; exit 2; }
      need_free "$p"
    done
    [[ "$AUTOKART_LAB_ROS_DOMAIN_ID" != "$AUTOKART_LIVE_ROS_DOMAIN_ID" ]] || { ak_err "lab domain == live domain; refusing"; exit 2; }
    setsid "$AUTOKART_PYTHON" -m autokart_perception.lab_api --host 127.0.0.1 --port "$AUTOKART_LAB_CORE_PORT" & pids+=($!)
    ak_source_ros
    setsid env ROS_DOMAIN_ID="$AUTOKART_LAB_ROS_DOMAIN_ID" ros2 run autokart_dashboard_gateway dashboard_gateway \
      --source SIMULATION --host "$AUTOKART_BIND_HOST" --port "$AUTOKART_LAB_GATEWAY_PORT" & pids+=($!)
    setsid bash -c 'cd "$1" && exec npm run preview:lab' _ "$ROOT/phase_sim" & pids+=($!)
    ;;
  *) ak_err "usage: $0 {perception|dashboard|lab}"; exit 2 ;;
esac

ak_info "$1 group running: pids ${pids[*]}"
set +e
wait -n "${pids[@]}"
rc=$?
ak_err "a child of the $1 group exited (rc=$rc); stopping the group"
exit $(( rc == 0 ? 1 : rc ))
