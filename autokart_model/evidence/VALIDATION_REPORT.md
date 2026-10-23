# Validation report — AutoKart Tarun handoff

Date: **23 October 2026**  
Host scope: x86_64 Ubuntu/ROS 2 Humble software validation  
Physical scope: **no camera test, no actuator power, no firmware operation, no vehicle movement**

## Results

| Check | Result | Evidence boundary |
|---|---|---|
| Bundle source inventory | PASS | 13 ROS packages present |
| Clean out-of-tree ROS build | PASS | 13 packages finished |
| ROS package pytest suites | PASS | 322 passed, 2 skipped across 12 Python-node packages |
| Phase 1 perception pytest | PASS | 178 passed, 2 skipped |
| YOLO26s manifest/SHA-256 | PASS | `425a89cef2eda9db9c84fbf30cad187e510510b95d7b7e9904003fd22cec1158` |
| YOLO26s ONNX checker/session/zero-input CPU inference | PASS | input `1x3x640x640`, output `1x300x6` |
| HP60C native bridge compile | PASS | x86_64 build only; target architecture must rebuild |
| No-ack launch guard | PASS | returned code 3 and refused launch |
| Full bundle SHA-256 manifest | PASS | generated after final assembly; verify with `scripts/verify_bundle.sh` |
| HP60C live delivery on Tarun computer | NOT RUN | camera hardware not exercised |
| Physical cone detection/metric position | NOT RUN | calibration remains unverified |
| Raspberry Pi rate/latency | NOT RUN | target device not exercised |
| Steering/BLDC/brake/E-stop/stopping | NOT RUN | outside stationary perception scope |
| Vehicle/autonomous motion | NO-GO | required measured gates incomplete |

## Important interpretation

The package is validated as a transferable software handoff. The model load result proves file identity and executable ONNX structure on this host; it does not prove accuracy on new HP60C scenes. The ROS tests prove the packaged contracts and fail-closed behavior; they do not qualify hardware, timing or braking.

The bundled camera calibration is `MOUNT_CONFIGURED_UNVERIFIED`, with a recorded pitch disagreement. `/perception/cones` is designed to remain empty for live input until metric calibration is genuinely qualified. This is a correct safety behavior.

The two ROS test skips are VCU serial/hardware-dependent cases. The two perception skips are optional/environment-specific cases. No skipped test was promoted to PASS.

## Source identity

- Parent repository recorded at packaging: `a7a824966549c00dbee5776422455295fde08629`
- Isolated 12-node baseline recorded in prior QA: `47686d603cdb623fe4fda546aa3750f97d573249`
- Phase 1 nested repository recorded at packaging: `87f2e288cb1505a9eb233e077603f19689cc3114` with uncommitted material; therefore exact file-level hashes in this bundle are authoritative.
- Prior clean-export hashes are retained under `evidence/source_qa/SOURCE_SHA256.json`.
- Final transferable hashes are in `MANIFEST.sha256`.

