# AutoKart Mission Control

Mission Control opens in **LAB / SKIDPAD / READY**. It shows the complete physical cone course and kart immediately; press **RUN** to start a deterministic kinematic bicycle. LAB needs no ROS, gateway, or FSDS. It teaches and debugs the autonomy pipeline; it is not a vehicle-dynamics validation tool. FSDS remains the physics authority.

The skidpad is generated from a figure-eight centerline, a swept track corridor, extracted boundaries, and arc-length cone sampling. Its loop radius, width, cone spacing, straight entry and exit lengths, and clear junction radius are configurable in **BUILDER**. Before Run, there are no detections, graph edges, gates, or planned route. The generating centerline is never sent to the renderer or planner.

## Run and verify

```bash
cd /home/mark1/Desktop/autonomous_kart/phase_sim
npm install
npm run dev
```

Open the URL printed by Vite. `npm run check` runs the frontend tests, TypeScript build, and Vite build. `npm run build` writes `dist/` for the ROS gateway to serve.

## Data modes

| Mode | Source | Status |
| --- | --- | --- |
| LAB | Deterministic local sensor, planner, mission, and kinematic kart | Available on startup |
| SIM | Normalized ROS2 frames from the gateway | Requires FSDS, ROS2 publishers, and gateway |
| REPLAY | Recorded `WorldFrame` JSON, JSONL, or gateway replay | Available after loading or recording frames |
| REAL | Same normalized frame format from a physical kart | Integration point; requires real ROS2 publishers |

Every mode uses the same `MissionFrame` / `WorldFrame` renderer. Source-specific code produces frames; the world view, algorithm inspection, plots, and replay consume them. LAB **SAVE RUN** exports JSONL. The REPLAY view can open it and inspect any recorded frame and algorithm stage. The gateway's read-only replay API and WebSocket stream use the same schema.

## LAB scenarios and pipeline

- **SKIDPAD:** two symmetric lobes, straight entry and exit, physical blue/yellow boundaries and orange special cones. Winding estimates right and left loop progress, and mission decisions restrict branch choice until the required loops are complete.
- **AUTOCROSS:** seeded long course with straights, S bends, slaloms, hairpins, radius changes, and narrow/wide sections. Builder parameters change the generated world.
- **BRAKE:** straight cone corridor, detected obstacle, stopping-distance envelope, brake override, and actual deceleration to a stop.

While LAB runs, cones in the moving sensor FOV become detections. Delaunay triangulation uses those detections, Gabriel filtering tests the diameter disks, cross-color accepted edges become candidate gates, and scored local candidates provide the selected path. The Algorithm view freezes the current frame and reveals its stages. Ground truth is display-only and never enters those geometry functions. LAB's mission controller also uses known scenario geometry as a prior for loop selection and steering stabilization; it is a learning model, not a map-free autonomy claim.

The older Delaunay Kart Lab remains at `/demo` for its original presets. It is labeled DEMO and is separate from the Mission Control LAB source.

## ROS2 gateway

The gateway is at `../kart/ros2_ws/src/autokart_dashboard_gateway`. The topic and optional `/dashboard/debug` contract, ROS2 build commands, and replay commands are in `../kart/docs/mission_control_debug.md`. Vite proxies `/mission` to gateway port 8765. The gateway has no actuator endpoint. SIM and REAL show missing debug stages when their publishers do not provide them; Mission Control does not synthesize ROS algorithm decisions.

The checked-in ROS planning and perception packages do not yet contain runtime publishers for all debug stages, and this repository does not include a runnable FSDS installation. LAB and replay can be verified locally; FSDS physics and end-to-end SIM behavior require the external simulator and live ROS2 topics.

With an installed FSDS ROS2 bridge and `fs_msgs`, start the gateway with `--source SIMULATION --fsds --scenario SKIDPAD` to read FSDS testing-only odometry and the latched physical cone track. The dashboard still waits for actual autonomy publishers for detections and algorithm debug; it never sends simulator control commands.
