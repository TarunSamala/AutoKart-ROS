#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FW="$ROOT/../esp32_firmware"
g++ -std=c++11 -Wall -Wextra -I"$FW/include" -o /tmp/vcu_test "$FW/test/test_vcu_logic.cpp" "$FW/src/protocol.cpp" "$FW/src/vcu_logic.cpp"
/tmp/vcu_test
python3 -m pytest -q -p no:anyio "$ROOT/tests"
