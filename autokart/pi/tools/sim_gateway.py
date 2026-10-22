#!/usr/bin/env python3
"""SIMULATED data source for testing the dashboard UI without ROS or hardware.

It runs the REAL planner/controller/mapping logic against a toy kart model and pushes the results through the
same GatewayCore converters and WebSocket server as the real gateway. Everything it sends is marked
mode=SIMULATED, so the dashboard labels it SIMULATED and never LIVE.

    python3 tools/sim_gateway.py            # then open http://localhost:8080/index.html
"""
import base64
import http.server
import functools
import os
import sys
import threading
import time
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for pkg in ("mapping", "planner", "controller"):
    sys.path.insert(0, os.path.join(ROOT, "laptop", "ros2", f"autokart_{pkg}"))
sys.path.insert(0, os.path.join(ROOT, "dashboard", "ros_gateway"))

import cv2
import numpy as np
from autokart_controller.core import Controller, ControllerConfig
from autokart_mapping.core import ConeTracker, MapConfig, build_corridor
from autokart_planner.core import Planner, PlannerConfig
from gateway_core import GatewayCore, GatewayServer

STOP_X = 9.0


def pt(x, y):
    return NS(x=x, y=y)


def render(x, cones):
    img = np.full((480, 640, 3), (60, 70, 60), np.uint8)
    cv2.rectangle(img, (0, 300), (640, 480), (90, 90, 90), -1)
    boxes = []
    for cls, cx, cy in cones:
        d = cx - x
        if d < 0.8:
            continue
        u = int(320 - cy / d * 420)
        v = int(300 + 90 / d)
        h = int(max(10, 150 / d)); w = int(h * 0.6)
        col = {"blue": (230, 120, 40), "yellow": (40, 210, 240), "orange": (30, 120, 250)}[cls]
        cv2.rectangle(img, (u - w // 2, v - h), (u + w // 2, v), col, -1)
        boxes.append((cls, u - w // 2, v - h, u + w // 2, v, d))
    return img, boxes


def main():
    srv = GatewayServer("0.0.0.0", 8765, lambda a, m: print("request:", a))
    core = GatewayCore(srv.emit, mode="SIMULATED")
    srv.start()
    here = os.path.join(ROOT, "dashboard", "frontend")
    httpd = http.server.ThreadingHTTPServer(("0.0.0.0", 8080), functools.partial(http.server.SimpleHTTPRequestHandler, directory=here))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print("SIMULATED dashboard: http://localhost:8080/index.html  (ws://localhost:8765)")

    world = [("blue", 2 + 2 * i, 1.5) for i in range(5)] + [("yellow", 2 + 2 * i, -1.5) for i in range(5)] + [("orange", STOP_X, 0.0)]
    mcfg = MapConfig()
    stages = [(i, n, "READY" if i not in (6,) else "UNAVAILABLE", "simulated") for i, n in enumerate(
        ["Start ROS 2 runtime", "ESP32-S3 USB communication", "Firmware / protocol compatibility", "AS5047P axle encoder",
         "Steering + brake encoders", "MPU-6500 IMU", "HP60C SDK + camera", "RGB/depth synchronization", "YOLO26s ONNX model",
         "Camera calibration", "Localization + mapping", "Planner + controller", "Independent E-stop + interlocks",
         "Disarmed, mission-ready"], 1)]
    stage_objs = [NS(id=i, name=n, state=s, detail=d) for i, n, s, d in stages]

    while True:  # restart the mission forever
        tr, pl, ctl = ConeTracker(mcfg), Planner(PlannerConfig(front_bumper_x_m=0.6, a_verified_mps2=0.8, d_margin_m=0.02)), Controller(ControllerConfig())
        x, v, t, dt, fid = 0.0, 0.0, 0.0, 0.05, 0
        t_arm = 4.0
        armed = False
        stopped_at = None
        while True:
            time.sleep(dt)
            t += dt
            if not armed and t > t_arm:
                armed = True
            pl.set_active(armed)
            vis = [(c, cx - x, cy) for c, cx, cy in world if 0.9 < cx - x < 8.0]
            tr.update(vis, (x, 0.0, 0.0), t)
            cor = build_corridor(tr.confirmed(), (x, 0.0, 0.0), mcfg)
            r = pl.plan(t, cor.centerline, cor.valid, 0.0, [(q.cls, q.x, q.y) for q in tr.confirmed()], (x, 0.0, 0.0), v)
            o = ctl.step(t, armed, r.decision, r.target_speed_mps, cor.centerline, (x, 0.0, 0.0), v)
            if o.brake_pct >= 50:
                v = max(0.0, v - 0.8 * dt)
            else:
                v += (o.throttle_v - v) / 0.4 * dt
            x += v * dt
            fid += 1
            now = time.time()
            sec, ns = int(now), int((now % 1) * 1e9)
            hdr = NS(stamp=NS(sec=sec, nanosec=ns), frame_id="")
            img, boxes = render(x, world)
            ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            cones = [NS(cls=c, conf=0.9, x1=a, y1=b, x2=cc, y2=d_, dist_m=dist, x=dist, y=0.0, z=0.0, depth_ok=True)
                     for c, a, b, cc, d_, dist in boxes]
            core.on_detections(NS(header=hdr, frame_id=fid, cones=cones, inference_ms=18.0, depth_valid=True, calibration_verified=False))
            core.on_image(NS(header=NS(stamp=hdr.stamp, frame_id=str(fid)), data=jpg.tobytes()))
            core.on_ctrl(NS(steer_deg_cmd=o.steer_deg, target_speed_mps=r.target_speed_mps, brake_pct=o.brake_pct,
                            lookahead_valid=o.steer.valid, lookahead_x=o.steer.lookahead_xy[0], lookahead_y=o.steer.lookahead_xy[1],
                            wheelbase_m=1.0, lookahead_m=o.steer.lookahead_m, alpha_rad=o.steer.alpha_rad))
            core.on_plan(NS(header=hdr, frame_id=fid, path=[pt(a, b) for a, b in r.path], path_valid=r.path_valid,
                            stop_valid=r.stop_valid, stop_target=pt(*r.stop_target), nearest_cone_m=r.nearest_cone_m,
                            available_m=r.available_m, required_stop_m=r.required_stop_m, decision=r.decision,
                            target_speed_mps=r.target_speed_mps))
            core.on_map(NS(header=hdr, frame_id=fid, valid=cor.valid, status=cor.status, corridor_width_m=cor.width_m,
                           cones=[NS(cls=q.cls, x=q.x, y=q.y) for q in tr.confirmed()],
                           centerline=[pt(a, b) for a, b in cor.centerline]))
            core.on_odom(NS(header=hdr, x=x, y=0.0, yaw_rad=0.0, speed_mps=v, yaw_rate_dps=0.0, axle_counts=int(x * 4000), status="SIMULATED"))
            core.on_telemetry(NS(header=hdr, speed_mps=v, steer_deg=0.0, steer_valid=True, steer_pwm_ok=True, dac_v=o.throttle_v,
                                 state_name="ARMED" if armed else "SAFE", en=armed, brk=o.brake_pct >= 50))
            core.on_safety(NS(header=hdr, state="ARMED" if armed else "DISARMED", vcu_heartbeat_ok=True, arm_allowed=not armed,
                              stop_latched=False, reasons=["simulated data: nothing here is real"], startup_ready=True, stages=stage_objs))
            if o.stopped:
                stopped_at = stopped_at or t
                if t - stopped_at > 6:
                    core.log(f"simulated stop, gap to cone {(STOP_X - (x + 0.6)) * 100:.0f} cm (target 15 cm)")
                    break


if __name__ == "__main__":
    main()
