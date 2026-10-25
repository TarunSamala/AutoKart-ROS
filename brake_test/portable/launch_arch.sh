#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
CONTAINER=${AUTOKART_DISTROBOX:-kart-ros2-humble}
UI=http://127.0.0.1:8781/

fail() {
    printf '\nAutoKart Brake Test: %s\n' "$*" >&2
    read -r -p "Press Enter to close..." _ || true
    exit 1
}

[[ -f /etc/arch-release ]] || fail "This launcher is for Arch Linux. Use launch.sh on Ubuntu 22.04 or Raspberry Pi."
command -v distrobox >/dev/null 2>&1 || \
    fail "Distrobox is missing. Install it with: sudo pacman -S --needed distrobox podman"
command -v curl >/dev/null 2>&1 || fail "curl is missing. Install it with: sudo pacman -S --needed curl"

if ! distrobox list 2>/dev/null | grep -Fq "$CONTAINER"; then
    printf '%s\n' "Creating Ubuntu 22.04 container: $CONTAINER"
    distrobox create --yes --name "$CONTAINER" --image docker.io/library/ubuntu:22.04 || \
        fail "Could not create $CONTAINER. Connect to the internet and retry."
fi

CHILD_PID=
stop_child() {
    if [[ -n ${CHILD_PID:-} ]]; then
        kill "$CHILD_PID" >/dev/null 2>&1 || true
    fi
}
trap stop_child EXIT INT TERM HUP

distrobox enter "$CONTAINER" -- bash -lc \
    'root=$1; export AUTOKART_SKIP_DESKTOP=1 AUTOKART_SKIP_BROWSER=1; cd "$root"; exec bash brake_test/portable/launch.sh' \
    autokart "$ROOT" &
CHILD_PID=$!

for _ in $(seq 1 2400); do
    if curl --fail --silent --max-time 1 "$UI/api/status" >/dev/null 2>&1; then
        xdg-open "$UI" >/dev/null 2>&1 || true
        printf '%s\n' "AutoKart dashboard opened at $UI"
        wait "$CHILD_PID"
        exit $?
    fi
    if ! kill -0 "$CHILD_PID" >/dev/null 2>&1; then
        wait "$CHILD_PID" || true
        fail "The Ubuntu brake-test process exited before the dashboard became ready."
    fi
    sleep 0.25
done

fail "Dashboard startup timed out."
