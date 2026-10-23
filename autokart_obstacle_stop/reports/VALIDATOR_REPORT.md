# Validator report — autokart_obstacle_stop

Validation date: 2026-10-23.

## Environment

Host-side Python only. ROS 2 Humble, `autokart_msgs`, an HP60C camera, trained
cone model and physical VCU were not available in this environment.

## Checks actually run

1. Standard-library unit suite: `10 tests`, all passed.
2. Every Python source file compiled successfully.
3. `package.xml` parsed as valid XML.
4. `git diff --check` passed.
5. Topic and field usage was statically checked against the existing
   perception adapter, controller and VCU bridge source contracts.

## Acceptance matrix

| Capability | Status | Evidence |
|---|---|---|
| Forward cone inside corridor requests STOP | VALIDATED | unit test |
| Track-edge cones outside corridor pass | VALIDATED | unit test |
| Confidence and uncertainty handling | VALIDATED | unit tests |
| Stop hold and clear-frame hysteresis | VALIDATED | unit test |
| Missing/stale perception fails closed | VALIDATED | unit tests |
| Wrong frame and synthetic ground truth fail closed | VALIDATED | unit test |
| ROS package import/build/launch | NOT RUN | ROS 2 unavailable on host |
| Live HP60C cone detection | NOT RUN | camera/model/calibration unavailable |
| ROS-to-VCU command delivery | NOT RUN | VCU transport not connected |
| Physical BLDC stop | NOT RUN | requires restrained hardware test |

This is software decision-logic validation only. It is not a vehicle-readiness
or physical stopping claim.
