# Mission Control repair audit

Status: audit in progress. No implementation change should be called accepted until its gate has a test result and visual check.

| ID | Severity | Area | Evidence at audit start | Repair gate |
| --- | --- | --- | --- | --- |
| A-01 | P0 | Product navigation | The operating-mode selector exposes SIM and REAL, which lead to empty track states when no ROS source is connected. | 1 |
| A-02 | P0 | Critical choices | Scenario and camera controls are native selects; the user reports dropdown failure, and interaction reliability is unproven. | 1 |
| A-03 | P1 | Skidpad | `labScenarios.ts` defaults to radius 8 m and samples a swept corridor at 1 m spacing. The supplied 2026 rule values require 18.25 m centre separation and fixed 17/13 ring cone counts. | 2 |
| A-04 | P1 | Sensor | `LabSkidpad.ts` uses a 19 m, 2.55 rad flat FOV and calls it a camera. This is inconsistent with the specified HP60C depth sensor. | 3 |
| A-05 | P1 | Algorithm validity | Delaunay uses currently detected cones, but loop control uses scenario radius and centres as priors. It must be presented as LAB prior, never as a map-free estimate. | 4–5 |
| A-06 | P1 | Obstacle stop | Obstacle range is computed with 26 m sensing and cannot be moved; HP60C alone cannot validate the stopping envelope. | 7 |
| A-07 | P1 | Autocross | The generated route is a sinusoidal corridor with no sourced variants or rules metadata. | 6 |
| A-08 | P2 | Digital twin | `MissionScene.tsx` renders a white box/nose and box wheels. The repository has a provisional URDF/SDF and dimensions, but no CAD/GLB. | 2 |
| A-09 | P2 | World | No asphalt or bounded course surface; current render is cones over a generic grid and large empty regions. | 2, 8 |
| A-10 | P2 | Reuse and state | A single React component owns multiple views and controls; switching and reset need regression tests for stale geometry and stage state. | 1, 8 |
| A-11 | P2 | Replay | The frame contract and JSONL replay exist, but visible replay controls and error handling need interaction tests. | 8 |
| A-12 | P2 | Responsive UI | Desktop viewport sizes and all visible controls lack automated end-to-end coverage. | 8 |
| A-13 | P3 | Performance | Renderer pools graph lines and instanced cones, but repeated React rendering, camera framing, labels, and WebGL cleanup need profiling/inspection. | 8 |

The kart configuration at `../kart/configs/karts/prototype.yaml` explicitly marks all dimensions provisional and simulation-only. No physical-kart CAD was found in the repository scan. The current gateway is read-only and can remain an internal adapter; its existence does not justify the visible SIM/REAL selector.

## Gate status

| Gate | Scope | Status |
| --- | --- | --- |
| 1 | Interaction audit, remove SIM/REAL, scenario controls | Pending |
| 2 | Official skidpad, kart model, cones | Pending authoritative rules check |
| 3 | HP60C depth sensor | Pending official sensor check |
| 4 | Delaunay/Gabriel visualization | Pending |
| 5 | Winding/mission | Pending |
| 6 | Sourced autocross variants | Pending source research |
| 7 | Movable obstacle and stop | Pending |
| 8 | Visual, interaction, safety QA | Pending |
