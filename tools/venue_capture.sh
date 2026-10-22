#!/usr/bin/env bash
# Guided venue recording session: starts the phase-1 graph if needed, then for each tape-measured
# station (distance x light condition) waits for the operator to place a cone and press Enter, saves
# a burst of stamp-matched RGB+depth frame pairs via tools/venue_capture_frames.py, and appends a row
# to a plain distance sheet (CSV). Read-only toward the vehicle: no ARM/CMD is sent, no verified flag
# is set. The saved frames and distance sheet are raw evidence for a human and for
# tools/calibration_candidate.py / tools/depth_check.py / tools/model_validation.py to use later --
# this script does not itself fit a pitch, validate a model, or write any config.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUN="$ROOT/.run/phase1"

DISTANCES=(1.2 2 3 4)
FRAMES_PER_STATION=10
TIMEOUT_S=30

while [[ $# -gt 0 ]]; do
  case "$1" in
    --distances) shift; DISTANCES=(); while [[ $# -gt 0 && "$1" != --* ]]; do DISTANCES+=("$1"); shift; done ;;
    --frames) FRAMES_PER_STATION="$2"; shift 2 ;;
    --timeout) TIMEOUT_S="$2"; shift 2 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

if [[ ! -f "$RUN/ros.pid" ]] || ! kill -0 "$(cat "$RUN/ros.pid" 2>/dev/null)" 2>/dev/null; then
  echo "Phase-1 graph not running: starting it with scripts/start_phase1.sh first."
  "$ROOT/scripts/start_phase1.sh"
  echo "Waiting for the graph and camera to come up..."
  sleep 8
fi

TS="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_ROOT="$ROOT/results/venue_capture_$TS"
mkdir -p "$OUT_ROOT"
SHEET="$OUT_ROOT/distance_sheet.csv"
echo "station_dir,tape_distance_m,light_condition,operator_notes,captured_utc,saved_pairs" > "$SHEET"

echo
echo "=== VENUE CAPTURE SESSION: $TS ==="
echo "Diagnostic data only. This script sends no ARM/CMD/DIS/RST and sets no verified flag."
echo "It only saves RGB+depth frame pairs and a distance sheet under results/venue_capture_$TS/."
echo "Stations (tape-measured camera-forward distance): ${DISTANCES[*]} m"
echo

read -r -p "Light condition label for this round (e.g. bright, dim, mixed, artificial): " LIGHT
read -r -p "Any operator notes for this round (optional): " NOTES

for DIST in "${DISTANCES[@]}"; do
  TAG="${DIST}m_${LIGHT// /_}"
  STATION_DIR="$OUT_ROOT/$TAG"
  echo
  read -r -p "Place one cone at ${DIST} m tape-measured camera-forward under '$LIGHT' light, then press Enter..." _
  echo "Capturing ${FRAMES_PER_STATION} paired frames (timeout ${TIMEOUT_S}s)..."
  SAVED=0
  if python3 "$ROOT/tools/venue_capture_frames.py" --station-dir "$STATION_DIR" \
      --frames "$FRAMES_PER_STATION" --timeout "$TIMEOUT_S"; then
    STATUS="OK"
  else
    STATUS="PARTIAL_OR_FAILED"
    echo "Warning: capture at ${DIST} m did not reach the requested frame count; see $STATION_DIR/meta.json" >&2
  fi
  if [[ -f "$STATION_DIR/meta.json" ]]; then
    SAVED="$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['saved_pairs'])" "$STATION_DIR/meta.json" 2>/dev/null || echo 0)"
  fi
  CAPTURED_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  printf '%s,%s,%s,%s,%s,%s\n' "$TAG" "$DIST" "$LIGHT" "${NOTES//,/;}" "$CAPTURED_UTC" "$SAVED" >> "$SHEET"
  echo "Station $TAG: $STATUS, $SAVED/$FRAMES_PER_STATION pairs saved."
done

echo
echo "Distance sheet: $SHEET"
echo "Recorded sessions: $OUT_ROOT/<distance>m_<light>/ (rgb_*.png, depth_*.npy, meta.json)"
echo "Nothing was written to config or any verified flag. Use this evidence with:"
echo "  tools/depth_check.py --fit-pitch   (its own live, interactive capture -- does not read these files)"
echo "  tools/calibration_candidate.py     (status layer over the pitch fit; writes CANDIDATE records only)"
echo "  tools/model_validation.py          (file-integrity/manifest check; does not yet consume recordings)"
echo "Run this script again with a different --light-condition answer to add another light round over the"
echo "same stations; each run gets its own results/venue_capture_<timestamp>/ directory and distance sheet."
