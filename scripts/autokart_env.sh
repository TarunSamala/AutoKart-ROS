#!/usr/bin/env bash
# AutoKart shared environment + process helpers. SOURCE this file; do not execute it.
#   source "$(dirname "${BASH_SOURCE[0]}")/autokart_env.sh"
# Every value below is overridable from the caller's environment.
# Read-only diagnostic tooling: nothing here starts anything that can actuate the vehicle.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "autokart_env.sh must be sourced, not executed" >&2
  exit 2
fi

AUTOKART_ROOT="${AUTOKART_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
ROOT="$AUTOKART_ROOT"
export AUTOKART_ROOT ROOT

# ---- Port / process contract (docs/ui_split/OPERATIONS.md) ----
export AUTOKART_PERCEPTION_PORT="${AUTOKART_PERCEPTION_PORT:-8768}"     # HP60C single SDK owner
export AUTOKART_LIVE_GATEWAY_PORT="${AUTOKART_LIVE_GATEWAY_PORT:-8765}" # live ROS gateway
export AUTOKART_DASHBOARD_PORT="${AUTOKART_DASHBOARD_PORT:-5180}"       # Vehicle Dashboard UI (read by vite.dashboard.config.ts)
export AUTOKART_LAB_CORE_PORT="${AUTOKART_LAB_CORE_PORT:-8767}"         # Lab CORE API
export AUTOKART_LAB_GATEWAY_PORT="${AUTOKART_LAB_GATEWAY_PORT:-8775}"   # Lab ROS SIMULATION gateway
export AUTOKART_LAB_PORT="${AUTOKART_LAB_PORT:-5181}"                   # Algorithm Lab UI (read by vite.lab.config.ts)
export AUTOKART_BIND_HOST="${AUTOKART_BIND_HOST:-127.0.0.1}"            # backends stay loopback-only

# ROS domains: live stack inherits ROS_DOMAIN_ID (default 0); the Lab is isolated on 42.
export AUTOKART_LIVE_ROS_DOMAIN_ID="${AUTOKART_LIVE_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-0}}"
export AUTOKART_LAB_ROS_DOMAIN_ID="${AUTOKART_LAB_ROS_DOMAIN_ID:-42}"

# Vite proxy targets (consumed by phase_sim/vite.{dashboard,lab}.config.ts)
export AUTOKART_PERCEPTION_URL="http://127.0.0.1:${AUTOKART_PERCEPTION_PORT}"
export AUTOKART_ROS_GATEWAY_URL="http://127.0.0.1:${AUTOKART_LIVE_GATEWAY_PORT}"
export AUTOKART_LAB_CORE_URL="http://127.0.0.1:${AUTOKART_LAB_CORE_PORT}"
export AUTOKART_LAB_ROS_GATEWAY_URL="http://127.0.0.1:${AUTOKART_LAB_GATEWAY_PORT}"

# ---- Python ----
export AUTOKART_PYTHON="${AUTOKART_PYTHON:-python3}"
export PYTHONPATH="$ROOT/autonomy_stack_phase1/phase_01_perception/src:$ROOT/kart${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONUNBUFFERED=1

# ---- ROS (repo-local isolated build from scripts/build_ros.sh) ----
export AUTOKART_ROS_DISTRO_SETUP="${AUTOKART_ROS_DISTRO_SETUP:-/opt/ros/humble/setup.bash}"
export AUTOKART_ROS_BUILD_DIR="${AUTOKART_ROS_BUILD_DIR:-$ROOT/.ros_build}"
export AUTOKART_ROS_INSTALL="$AUTOKART_ROS_BUILD_DIR/install"

# ---- Run state ----
export AUTOKART_RUN_DIR="${AUTOKART_RUN_DIR:-$ROOT/.run}"
export AUTOKART_PID_DIR="$AUTOKART_RUN_DIR/pids"
export AUTOKART_LOG_DIR="$AUTOKART_RUN_DIR/logs"
mkdir -p "$AUTOKART_PID_DIR" "$AUTOKART_LOG_DIR"

ak_info() { printf '[autokart] %s\n' "$*"; }
ak_warn() { printf '[autokart] WARNING: %s\n' "$*" >&2; }
ak_err()  { printf '[autokart] ERROR: %s\n' "$*" >&2; }

# Source ROS distro + the repo-local install. setup.bash is not `set -u` clean.
ak_source_ros() {
  if [[ ! -f "$AUTOKART_ROS_DISTRO_SETUP" ]]; then
    ak_err "ROS setup not found: $AUTOKART_ROS_DISTRO_SETUP"; return 1
  fi
  if [[ ! -f "$AUTOKART_ROS_INSTALL/setup.bash" ]]; then
    ak_err "ROS workspace not built at $AUTOKART_ROS_INSTALL -- run scripts/build_ros.sh first"; return 1
  fi
  local had_u=0; [[ $- == *u* ]] && had_u=1
  set +u
  # shellcheck disable=SC1090
  source "$AUTOKART_ROS_DISTRO_SETUP"
  # shellcheck disable=SC1090
  source "$AUTOKART_ROS_INSTALL/setup.bash"
  ((had_u)) && set -u
  return 0
}

ak_host_ip() {
  local ip
  ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
  printf '%s' "${ip:-$(hostname)}"
}

# ---- Port helpers ----
ak_port_listening() { ss -Hltn "sport = :$1" 2>/dev/null | grep -q .; }
# Pids that own a listening socket on $1 (needs same-user processes; may be empty otherwise).
ak_port_pids() {
  ss -Hltnp "sport = :$1" 2>/dev/null | grep -o 'pid=[0-9]*' | cut -d= -f2 | sort -u | tr '\n' ' '
}
ak_wait_port() { # port timeout_s pid
  local port=$1 timeout=$2 pid=${3:-} i
  for ((i = 0; i < timeout * 4; i++)); do
    ak_port_listening "$port" && return 0
    if [[ -n "$pid" ]] && ! kill -0 "$pid" 2>/dev/null; then return 1; fi
    sleep 0.25
  done
  return 1
}

# ---- Pid-file helpers. A pid file holds "pid starttime"; starttime defeats pid reuse. ----
ak_pidfile() { printf '%s/%s.pid' "$AUTOKART_PID_DIR" "$1"; }
ak_logfile() { printf '%s/%s.log' "$AUTOKART_LOG_DIR" "$1"; }
# starttime = field 22 of /proc/PID/stat; parse after the last ')' because comm may contain spaces
# (npm retitles itself "npm run dev:dashboard").
ak_proc_start() { local s; s="$(cat "/proc/$1/stat" 2>/dev/null)" || return 0; s="${s##*) }"; awk '{print $20}' <<< "$s"; }
# Prints the pid if the named process is alive and is the one we started; else nothing.
ak_pid_of() {
  local f pid st
  f="$(ak_pidfile "$1")"
  [[ -f "$f" ]] || return 0
  read -r pid st < "$f" || true
  [[ -n "${pid:-}" ]] || return 0
  if kill -0 "$pid" 2>/dev/null && [[ "$(ak_proc_start "$pid")" == "${st:-}" ]]; then
    printf '%s' "$pid"
  fi
}
ak_is_running() { [[ -n "$(ak_pid_of "$1")" ]]; }

# Does any pid in the process group/session of $1 own port $2?
ak_port_owned_by_group() {
  local pid=$1 port=$2 p sid
  for p in $(ak_port_pids "$port"); do
    sid="$(ps -o sid= -p "$p" 2>/dev/null | tr -d ' ')"
    [[ "$sid" == "$pid" ]] && return 0
  done
  return 1
}

# ak_start NAME PORT(or -) WAIT_S CMD...   -- starts CMD in its own session (pid == pgid == sid)
# Idempotent: returns 0 without starting if our process is already alive.
# Returns 3 if PORT is held by a foreign process.
ak_start() {
  local name=$1 port=$2 wait_s=$3; shift 3
  local pid log
  log="$(ak_logfile "$name")"
  pid="$(ak_pid_of "$name")"
  if [[ -n "$pid" ]]; then
    ak_info "$name already running (pid $pid) -- not starting a second copy"
    return 0
  fi
  rm -f "$(ak_pidfile "$name")"
  if [[ "$port" != "-" ]] && ak_port_listening "$port"; then
    ak_err "$name: port $port is already in use by a process NOT started by these scripts (pid(s): $(ak_port_pids "$port"))."
    ak_err "$name: refusing to start. Stop that process or choose another port via env override."
    return 3
  fi
  {
    printf '\n===== %s start %s =====\n' "$name" "$(date -Is)"
    printf 'cmd:'; printf ' %q' "$@"; printf '\n'
  } >> "$log"
  setsid "$@" >> "$log" 2>&1 < /dev/null &
  pid=$!
  sleep 0.2
  printf '%s %s\n' "$pid" "$(ak_proc_start "$pid")" > "$(ak_pidfile "$name")"
  if [[ "$port" != "-" ]]; then
    if ! ak_wait_port "$port" "$wait_s" "$pid"; then
      ak_err "$name did not open port $port within ${wait_s}s (pid $pid). Last log lines ($log):"
      tail -n 15 "$log" >&2 || true
      ak_stop "$name" >/dev/null 2>&1 || true
      return 1
    fi
  else
    sleep "$wait_s"
    if ! kill -0 "$pid" 2>/dev/null; then
      ak_err "$name exited during startup. Last log lines ($log):"
      tail -n 15 "$log" >&2 || true
      rm -f "$(ak_pidfile "$name")"
      return 1
    fi
  fi
  ak_info "$name started (pid $pid${port:+, port $port}) log: $log"
}

# ak_stop NAME -- SIGTERM the whole session/process group, SIGKILL after 10 s. Idempotent.
ak_stop() {
  local name=$1 pid i
  pid="$(ak_pid_of "$name")"
  if [[ -z "$pid" ]]; then
    rm -f "$(ak_pidfile "$name")"
    ak_info "$name not running"
    return 0
  fi
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  for ((i = 0; i < 40; i++)); do
    pgrep -s "$pid" >/dev/null 2>&1 || break
    sleep 0.25
  done
  if pgrep -s "$pid" >/dev/null 2>&1; then
    ak_warn "$name (session $pid) ignored SIGTERM; sending SIGKILL"
    pkill -KILL -s "$pid" 2>/dev/null || true
  fi
  rm -f "$(ak_pidfile "$name")"
  ak_info "$name stopped (was pid $pid)"
}

# Basic HTTP probe: prints HTTP code (000 on connection failure).
ak_http_code() { curl -s -o /dev/null -m "${2:-3}" -w '%{http_code}' "$1" 2>/dev/null || true; }
