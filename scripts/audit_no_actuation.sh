#!/usr/bin/env bash
# Audit the running live ROS graph for any actuation path. Run WITH the live stack running.
#   scripts/audit_no_actuation.sh [--domain N]     (default: AUTOKART_LIVE_ROS_DOMAIN_ID)
# FAILs if any node publishes to /control/motion_command, /cmd_vel, any /vcu/* topic, or any topic
# whose name contains command|brake|throttle|steer; if the safety monitor is missing; or if its
# vehicle_authority != OFF / safety_monitor_mode != DIAGNOSTIC_ONLY. Subscriptions are allowed
# (the gateway only displays /control/motion_command).
set -euo pipefail
# shellcheck source=autokart_env.sh
source "$(dirname "${BASH_SOURCE[0]}")/autokart_env.sh"

DOMAIN="$AUTOKART_LIVE_ROS_DOMAIN_ID"
if [[ "${1:-}" == --domain ]]; then DOMAIN="${2:?--domain needs a value}"; fi
ak_source_ros
export ROS_DOMAIN_ID="$DOMAIN"
SPIN=(--no-daemon --spin-time 3)
FORBIDDEN_RE='^/control/motion_command$|^/cmd_vel$|^/vcu/|command|brake|throttle|steer'

fails=0
pass() { printf 'PASS    %s\n' "$*"; }
fail() { printf 'FAIL    %s\n' "$*"; fails=$((fails + 1)); }

echo "Actuation audit, ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
mapfile -t nodes < <(ros2 node list --all "${SPIN[@]}" 2>/dev/null | sort -u)
if ((${#nodes[@]} == 0)); then fail "no ROS nodes visible in domain $DOMAIN -- is the live stack running?"; fi
printf 'nodes (%d): %s\n' "${#nodes[@]}" "${nodes[*]:-}"

for n in "${nodes[@]}"; do
  info="$(ros2 node info --include-hidden "$n" "${SPIN[@]}" 2>&1 || true)"
  pubs="$(printf '%s\n' "$info" | awk '/^  Publishers:/{p=1;next} /^  [A-Z][A-Za-z ]*:/{p=0} p && NF {print $1}' | tr -d ':')"
  bad="$(printf '%s\n' "$pubs" | grep -E "$FORBIDDEN_RE" || true)"
  srv="$(printf '%s\n' "$info" | awk '/^  (Service Clients|Action Clients):/{p=1;next} /^  [A-Z][A-Za-z ]*:/{p=0} p && NF {print $1}' | tr -d ':' | grep -E "$FORBIDDEN_RE" || true)"
  if [[ -n "$bad" ]]; then fail "$n PUBLISHES actuation-like topic(s): $(echo $bad)"
  else pass "$n publishes no actuation topic (publishers: $(echo $pubs))"; fi
  [[ -z "$srv" ]] || fail "$n has actuation-like service/action client(s): $(echo $srv)"
done

# Cross-check from the topic side: catches publishers the node view could miss.
while read -r t; do
  [[ -n "$t" ]] || continue
  cnt="$(ros2 topic info "$t" "${SPIN[@]}" 2>/dev/null | awk -F': ' '/Publisher count/{print $2}')"
  if [[ "${cnt:-0}" != 0 ]]; then fail "topic $t has $cnt publisher(s)"; else pass "topic $t exists with 0 publishers (subscribers only)"; fi
done < <(ros2 topic list "${SPIN[@]}" 2>/dev/null | grep -E "$FORBIDDEN_RE" || true)

SM=/front_safety_envelope_monitor
if printf '%s\n' "${nodes[@]}" | grep -qx "$SM"; then
  va="$(ros2 param get "$SM" vehicle_authority "${SPIN[@]}" 2>&1 || true)"
  mode="$(ros2 param get "$SM" safety_monitor_mode "${SPIN[@]}" 2>&1 || true)"
  ds="$(ros2 param get "$SM" data_source "${SPIN[@]}" 2>&1 || true)"
  [[ "$va" == *"String value is: OFF" ]] && pass "$SM vehicle_authority=OFF" || fail "$SM vehicle_authority: $va"
  [[ "$mode" == *"String value is: DIAGNOSTIC_ONLY" ]] && pass "$SM safety_monitor_mode=DIAGNOSTIC_ONLY" || fail "$SM safety_monitor_mode: $mode"
  printf 'INFO    %s data_source -> %s\n' "$SM" "${ds##*: }"
else
  fail "$SM not running in domain $DOMAIN"
fi

echo
if ((fails)); then echo "ACTUATION AUDIT: FAIL ($fails)"; exit 1; fi
echo "ACTUATION AUDIT: PASS (no node publishes actuation topics; vehicle_authority OFF)"
