#!/usr/bin/env bash
# Curl-level smoke test of the running Algorithm Lab stack (through the Lab UI's own proxy).
# PASS/FAIL lines; exits non-zero on any FAIL.
set -euo pipefail
# shellcheck source=autokart_env.sh
source "$(dirname "${BASH_SOURCE[0]}")/autokart_env.sh"

UI="http://127.0.0.1:${AUTOKART_LAB_PORT}"
fails=0
pass() { printf 'PASS    %s\n' "$*"; }
fail() { printf 'FAIL    %s\n' "$*"; fails=$((fails + 1)); }
info() { printf 'INFO    %s\n' "$*"; }

echo "Lab smoke against $UI (core :$AUTOKART_LAB_CORE_PORT, sim gateway :$AUTOKART_LAB_GATEWAY_PORT, domain $AUTOKART_LAB_ROS_DOMAIN_ID)"

c="$(ak_http_code "$UI/")";     [[ "$c" == 200 ]] && pass "UI / -> 200"              || fail "UI / -> $c"
c="$(ak_http_code "$UI/home")"; [[ "$c" == 200 ]] && pass "UI /home deep link -> 200" || fail "UI /home -> $c"

resp="$(curl -s -m 5 -w '\n%{http_code} %{content_type}' "$UI/api/core/status" || true)"
meta="${resp##*$'\n'}"; body="${resp%$'\n'*}"
if [[ "$meta" == 200\ application/json* ]]; then pass "/api/core/status via UI proxy -> 200 JSON $(printf '%s' "$body" | head -c 80)"
else fail "/api/core/status via UI proxy -> '$meta'"; fi
va="$(python3 -c 'import json,sys;print(json.loads(sys.argv[1]).get("vehicle_authorized"))' "$body" 2>/dev/null || echo ?)"
[[ "$va" == False ]] && pass "lab core vehicle_authorized:false" || fail "lab core vehicle_authorized=$va"

# The Lab UI must NOT proxy /api/snapshot to the live perception service.
resp="$(curl -s -m 5 -w '\n%{http_code} %{content_type}' "$UI/api/snapshot" || true)"
meta="${resp##*$'\n'}"; body="${resp%$'\n'*}"
is_snapshot="$(python3 -c 'import json,sys
try: d=json.loads(sys.argv[1]); print("yes" if isinstance(d,dict) and ("frame_id" in d or "source" in d) else "no")
except Exception: print("no")' "$body")"
if [[ "$is_snapshot" == no ]]; then pass "lab UI does NOT proxy /api/snapshot to live perception (got '$meta', not a perception snapshot)"
else fail "lab UI returned a perception snapshot for /api/snapshot -- Lab is wired to the LIVE camera service"; fi
for p in /api/image/x/rgb /health; do
  b="$(curl -s -m 5 "$UI$p" || true)"
  if printf '%s' "$b" | grep -q '"vehicle_authorized"\|SNAPSHOT_EXPIRED\|"server"'; then fail "lab UI proxies $p to a live service"; else pass "lab UI does not proxy $p to live perception"; fi
done

# Lab mission gateway via proxy must be SIMULATION, never REAL_KART.
body="$(curl -s -m 5 "$UI/mission/api/status" || true)"
src="$(python3 -c 'import json,sys;d=json.loads(sys.argv[1]);print(d.get("source"),d.get("vehicle_authorized"))' "$body" 2>/dev/null || echo "? ?")"
if [[ "$src" == "SIMULATION False" ]]; then pass "/mission/api/status via lab proxy -> source=SIMULATION vehicle_authorized:false"
else fail "/mission/api/status via lab proxy -> '$src' (expected 'SIMULATION False')"; fi

echo
if ((fails)); then echo "LAB SMOKE: FAIL ($fails failing check(s))"; exit 1; fi
echo "LAB SMOKE: PASS"
