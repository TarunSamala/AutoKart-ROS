"""Local LAB API and deterministic simulated sensor/physics adapter.

This process is independent of CORE. It never opens the camera or VCU.
"""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
from math import atan2, cos, hypot, sin
from pathlib import Path

from .application import LabAdapter
from .shared_core import ALGORITHM_VERSION


class LabSimulation:
    def __init__(self):
        from kart_sim.config import load_kart_config
        from kart_sim.vehicle import PlanarBicycleKart
        config = load_kart_config(Path(__file__).resolve().parents[4] /
                                  "kart/configs/karts/prototype_dynamic.yaml")
        self.vehicle = PlanarBicycleKart(config)
        self.adapter = LabAdapter()
        self.world_cones = [
            {"id": 2 * i + side, "label": "blue" if side == 0 else "yellow",
             "confidence": 0.95, "x_m": 2.0 + 3.0 * i,
             "y_m": 1.5 if side == 0 else -1.5, "sigma_m": 0.05}
            for i in range(5) for side in range(2)]
        self.odometry = [0.0, 0.0, 0.0]
        self.tick = 0
        self.last = None

    def _payload(self) -> dict:
        state = self.vehicle.state
        x, y, yaw = self.odometry
        observations = []
        for item in self.world_cones:
            dx = item["x_m"] - state.x_m
            dy = item["y_m"] - state.y_m
            forward = cos(state.yaw_rad) * dx + sin(state.yaw_rad) * dy
            left = -sin(state.yaw_rad) * dx + cos(state.yaw_rad) * dy
            if forward < 0.2 or hypot(forward, left) > 12 or abs(atan2(left, forward)) > 1.1:
                continue
            observations.append({"id": item["id"], "label": item["label"],
                                 "confidence": item["confidence"], "xy": [forward, left],
                                 "sigma_m": item["sigma_m"], "distance_m": hypot(forward, left),
                                 "box": [0, 0, 1, 1]})
        now = 1_000_000_000 + self.tick * 100_000_000
        return {"timestamp_ns": now, "source": "SIMULATED", "mission": "STRAIGHT_TRACK",
                "cones": observations,
                "pose": {"x_m": x, "y_m": y, "yaw_rad": yaw,
                         "speed_mps": max(0.0, state.u_mps), "timestamp_ns": now,
                         "position_std_m": 0.05}}

    def step(self) -> dict:
        from kart_sim.types import VehicleCommand
        if self.last is None:
            self.last = self.adapter.evaluate(self._payload())
        candidate = self.last["control_candidate"]
        state = self.vehicle.state
        if candidate["valid"]:
            error = candidate["target_speed_mps"] - state.u_mps
            command = VehicleCommand(drive=max(0.0, min(0.3, 0.2 * error)),
                                     brake=max(0.0, min(1.0, -0.4 * error)),
                                     steering_rad=candidate["steering_angle_rad"])
        else:
            command = VehicleCommand(brake=1.0)
        for _ in range(5):
            previous = self.vehicle.state
            u = previous.u_mps
            yaw_rate = previous.yaw_rate_radps
            self.vehicle.step(command, 0.02)
            self.odometry[2] += yaw_rate * 0.02
            self.odometry[0] += u * cos(self.odometry[2]) * 0.02
            self.odometry[1] += u * sin(self.odometry[2]) * 0.02
        self.tick += 1
        self.last = self.adapter.evaluate(self._payload())
        return self.snapshot()

    def snapshot(self) -> dict:
        state = self.vehicle.state
        return {"source": "SIMULATED", "tick": self.tick, "cones": self.world_cones,
                "kart": {"x_m": state.x_m, "y_m": state.y_m,
                         "yaw_rad": state.yaw_rad, "speed_mps": state.u_mps},
                "odometry": {"x_m": self.odometry[0], "y_m": self.odometry[1],
                             "yaw_rad": self.odometry[2]},
                "diagnostic": self.last}

    def move_cone(self, cone_id: int, x_m: float, y_m: float) -> dict:
        if not all(-100 <= value <= 100 for value in (x_m, y_m)):
            raise ValueError("cone coordinate out of bounds")
        for item in self.world_cones:
            if item["id"] == cone_id:
                item["x_m"], item["y_m"] = x_m, y_m
                self.tick += 1
                self.last = self.adapter.evaluate(self._payload())
                return self.snapshot()
        raise ValueError("unknown cone")


class LabServer(HTTPServer):
    def __init__(self, address):
        super().__init__(address, LabHandler)
        self.simulation = LabSimulation()
        self.adapter = LabAdapter()


class LabHandler(BaseHTTPRequestHandler):
    def _reply(self, code: int, body: dict):
        data = json.dumps(body, allow_nan=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/api/core/status":
            self._reply(200, {"status": "SIMULATED", "vehicle_authorized": False,
                              "algorithm_version": ALGORITHM_VERSION,
                              "configuration_sha256": self.server.adapter.core.algorithms.configuration_sha256})
        elif self.path == "/api/core/snapshot":
            self._reply(200, self.server.simulation.snapshot())
        else:
            self._reply(404, {"error": "not found"})

    def do_POST(self):
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 <= length <= 1_000_000:
                raise ValueError("request too large")
            payload = json.loads(self.rfile.read(length) or b"{}")
            if self.path == "/api/core/evaluate":
                result = self.server.adapter.evaluate(payload)
            elif self.path == "/api/core/step":
                result = self.server.simulation.step()
            elif self.path == "/api/core/reset":
                self.server.simulation = LabSimulation()
                self.server.adapter.reset()
                result = self.server.simulation.snapshot()
            elif self.path == "/api/core/move-cone":
                result = self.server.simulation.move_cone(int(payload["id"]),
                                                          float(payload["x_m"]), float(payload["y_m"]))
            else:
                self._reply(404, {"error": "not found"})
                return
            self._reply(200, result)
        except (ValueError, TypeError, KeyError) as exc:
            self._reply(400, {"error": str(exc), "vehicle_authorized": False})


def main(argv=None):
    parser = argparse.ArgumentParser(description="AutoKart LAB diagnostic API")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8767)
    args = parser.parse_args(argv)
    server = LabServer((args.host, args.port))
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
