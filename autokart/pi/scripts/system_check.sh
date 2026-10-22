#!/usr/bin/env bash
# Quick pre-flight checks. Does not move anything.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ok(){ echo "  [ OK ] $1"; }; bad(){ echo "  [FAIL] $1"; FAILED=1; }
FAILED=0
command -v ros2 >/dev/null && ok "ros2 found" || bad "ros2 not found (source your ROS 2 setup)"
ls /dev/ttyACM* >/dev/null 2>&1 && ok "VCU serial device: $(ls /dev/ttyACM* | head -1)" || bad "no /dev/ttyACM* (ESP32-S3 not connected?)"
id -nG | grep -qw dialout && ok "user is in dialout group" || bad "user not in dialout group (sudo usermod -aG dialout \$USER)"
MODEL_PATH="/home/mark1/Desktop/autonomous_kart/models/yolo26s/best.onnx"
[ -f "$MODEL_PATH" ] && ok "YOLO26s model present ($MODEL_PATH)" || bad "$MODEL_PATH missing"
for m in onnxruntime cv2 serial websockets numpy; do python3 -c "import $m" 2>/dev/null && ok "python: $m" || bad "python module missing: $m"; done
if command -v ros2 >/dev/null; then
  ros2 topic list 2>/dev/null | grep -q "^/hp60c/color$" && ok "/hp60c/color is published" || bad "/hp60c/color not published (start the existing HP60C SDK owner)"
  ros2 topic list 2>/dev/null | grep -q "^/hp60c/depth$" && ok "/hp60c/depth is published" || bad "/hp60c/depth not published"
fi
grep -q "calibration_verified: true" "$ROOT/ros2_ws/src/autokart_bringup/config/autokart.yaml" && ok "camera calibration marked verified" || echo "  [NOTE] camera calibration not marked verified yet"
grep -q "braking_verified: true" "$ROOT/ros2_ws/src/autokart_bringup/config/autokart.yaml" && ok "braking marked verified" || echo "  [NOTE] braking not marked verified yet (arming stays blocked)"
exit $FAILED
