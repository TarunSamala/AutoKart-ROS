#!/usr/bin/env bash
# Curl-level smoke test of the running Vehicle Dashboard stack (through the UI's own proxy).
# PASS/FAIL/BLOCKED/INFO lines; exits non-zero on any FAIL. BLOCKED (no camera hardware) is not a failure.
set -euo pipefail
# shellcheck source=autokart_env.sh
source "$(dirname "${BASH_SOURCE[0]}")/autokart_env.sh"

UI="http://127.0.0.1:${AUTOKART_DASHBOARD_PORT}"
fails=0
pass() { printf 'PASS    %s\n' "$*"; }
fail() { printf 'FAIL    %s\n' "$*"; fails=$((fails + 1)); }
blocked() { printf 'BLOCKED %s\n' "$*"; }
info() { printf 'INFO    %s\n' "$*"; }
json_get() { python3 -c 'import json,sys
d=json.loads(sys.argv[1])
for k in sys.argv[2].split("."):
    d=d[k] if isinstance(d,dict) else None
print(json.dumps(d) if not isinstance(d,str) else d)' "$1" "$2" 2>/dev/null || echo __ERR__; }

echo "Dashboard smoke against $UI (perception :$AUTOKART_PERCEPTION_PORT, gateway :$AUTOKART_LIVE_GATEWAY_PORT)"

c="$(ak_http_code "$UI/")";         [[ "$c" == 200 ]] && pass "UI / -> 200"          || fail "UI / -> $c"
c="$(ak_http_code "$UI/overview")"; [[ "$c" == 200 ]] && pass "UI /overview deep link -> 200" || fail "UI /overview -> $c"

# /api/snapshot via the UI proxy must be JSON from the perception service (not Vite's SPA index.html).
resp="$(curl -s -m 5 -w '\n%{http_code} %{content_type}' "$UI/api/snapshot" || true)"
meta="${resp##*$'\n'}"; body="${resp%$'\n'*}"
src="$(json_get "$body" source)"
if [[ "$meta" == 200\ application/json* && "$src" != __ERR__ && "$src" != null ]]; then
  pass "/api/snapshot via UI proxy -> JSON, source=$src"
  [[ "$src" == SIMULATED ]] && info "perception is SIMULATED (--dev-sim bench mode) -- dashboard shows SIMULATED, not the HP60C"
else
  fail "/api/snapshot via UI proxy -> '$meta' (no JSON 'source'; proxy to :$AUTOKART_PERCEPTION_PORT missing or service down -- the 'Failed to fetch' signature)"
fi

c="$(ak_http_code "$UI/health")"; [[ "$c" == 200 ]] && pass "/health via UI proxy -> 200" || fail "/health via UI proxy -> $c"

# Gateway status via UI proxy: vehicle_authorized must be false.
body="$(curl -s -m 5 "$UI/mission/api/status" || true)"
va="$(json_get "$body" vehicle_authorized)"
if [[ "$va" == false ]]; then
  pass "/mission/api/status via UI proxy -> vehicle_authorized:false (source=$(json_get "$body" source), ros_connected=$(json_get "$body" ros_connected))"
else
  fail "/mission/api/status via UI proxy -> vehicle_authorized=$va"
fi

# Camera hop: reported honestly. LIVE SDK error / no frame = BLOCKED-no-hardware, not FAIL.
state=""; reason=""
for _ in 1 2 3 4 5 6 7 8; do
  body="$(curl -s -m 5 "$UI/api/v1/diagnostics/pipeline" || true)"
  hops="$(python3 -c 'import json,sys
d=json.loads(sys.argv[1]); h=d.get("hops")
h = {x.get("name"):x for x in h} if isinstance(h,list) else (h or {})
c=h.get("sdk_capture") or {}
print(c.get("state") or "", c.get("reason") or c.get("last_error") or "", c.get("source") or "", d.get("overall") or "", sep="|")' "$body" 2>/dev/null || true)"
  IFS='|' read -r state reason hsrc overall <<< "${hops:-|||}"
  [[ "$state" == STARTING ]] || break
  sleep 1
done
if [[ -z "$state" ]]; then
  # Older live_mission without the diagnostics endpoint: fall back to the snapshot.
  body="$(curl -s -m 5 "$UI/api/snapshot" || true)"
  cs="$(json_get "$body" camera_streaming)"; st="$(json_get "$body" status)"
  if [[ "$st" == __ERR__ ]]; then fail "camera hop: perception unreachable through the UI proxy (cannot classify)"
  elif [[ "$cs" == true ]]; then pass "camera hop: streaming (snapshot status=$st)"
  else blocked "camera hop: not streaming (snapshot status=$st, reasons=$(json_get "$body" reasons)) -- BLOCKED-no-hardware"; fi
elif [[ "$state" == OK ]]; then
  if [[ "$hsrc" == LIVE ]]; then pass "camera hop sdk_capture=OK source=LIVE (HP60C SDK streaming); pipeline overall=$overall"
  else info "camera hop sdk_capture=OK source=$hsrc (not LIVE hardware); overall=$overall"; fi
elif [[ "$state" == SDK_ERROR || "$state" == NO_FRAME || "$state" == STALE || "$state" == STARTING ]]; then
  blocked "camera hop sdk_capture=$state reason=$reason -- BLOCKED-no-hardware (expected without an HP60C; service is up and reporting it honestly)"
else
  info "camera hop sdk_capture=$state reason=$reason overall=$overall"
fi

echo
if ((fails)); then echo "DASHBOARD SMOKE: FAIL ($fails failing check(s))"; exit 1; fi
echo "DASHBOARD SMOKE: PASS"
