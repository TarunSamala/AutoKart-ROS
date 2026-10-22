#!/usr/bin/env bash
set -euo pipefail
python3 -m compileall -q kart_sim
echo 'Python headless simulator build check passed.'
