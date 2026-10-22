# Canonical component selection (RC-01)

## Selection state

There is **no verified deployed vehicle runtime** to designate canonical. For laptop diagnostic/replay work, retain the existing workspace path and wrapper; do not change its default as part of repository hygiene.

| Item | Current software selection | Identity and limits |
|---|---|---|
| Source checkout | `/home/mark1/Desktop/autonomous_kart/autonomy_stack_phase1/phase_01_perception/` | Parent Git HEAD is `87f2e288cb1505a9eb233e077603f19689cc3114` on `main`, but the perception source directory is untracked in that checkout. The active working-tree content is therefore not represented by that revision alone. |
| Diagnostic entry point | `/home/mark1/Desktop/autonomous_kart/autokart_apps/run_live.sh --mode live --host 127.0.0.1 --port 8768` | Resolves to `python -m autokart_perception.live_mission`; example service file uses this wrapper. Do not start a camera or service as part of RC-01. |
| Existing default detector | `conenet-v1` | `live_mission.py` defaults to this model. ONNX SHA-256: `0554f1fb28d19a4defac6b65b33e790ac2398e2f577fe353d01cbe28743aa103`. Not physically qualified. |
| Frozen evaluation model | `models/yolo26s/best.onnx` via explicit `--detector yolo26s` | SHA-256 `425a89cef2eda9db9c84fbf30cad187e510510b95d7b7e9904003fd22cec1158`, verified equal to `models/yolo26s/best.json`. Manifest: experimental, HP60C validation pending, vehicle authorization disabled. No default was changed. |
| Other detector choices | ConeNet v2, ConeSlayer | Experimental alternative and orange-only rollback respectively; see the repository map and audit evidence. |
| ROS / browser | `kart/ros2_ws/` and `phase_sim/` | Build/test sources only; not a complete, verified sensor-to-VCU runtime. |

## Unknowns that remain explicit

- Raspberry Pi deployed checkout/revision, Ubuntu image and running processes: **UNKNOWN**.
- HP60C serial/calibration, physical mount transform, depth scale and surveyed accuracy: **UNKNOWN / UNQUALIFIED**.
- MPU-6500 identity, connection, driver and calibration: **UNKNOWN**.
- Installed ESP32-S3 firmware image/hash, active pin map and protocol: **UNKNOWN**.
- AS5047P/AS5600 actual wiring and measured telemetry path: **UNKNOWN**.
- Independent physical E-stop, traction isolation and power-loss-safe braking: **UNKNOWN / NOT PROVEN**.
- Vehicle authorization: **false**. No physical command is part of this selection.

Keep the frozen skidpad rule (right two laps then left two laps, validated with state and winding/gate confirmation) and straight-course braking/obstruction-stop requirement as acceptance targets, not as claims about current implementation. Treat 10 km/h as out of scope until separately validated.
