# AUT-05 Jarvis diagnostic dashboard

This is a disarmed software prototype. `run_live.sh` does not open a VCU or publish vehicle commands. It reuses the AUT-03 `PerceptionGeometrySDK` and `Pipeline` for each current frame. The RGB image, RAW16 depth preview, and bird's-eye geometry are published under one frame ID. A fault or stale live frame clears the current geometry and controller candidate.

## Start

From `/home/mark1/Desktop/autonomous_kart`:

```bash
./autokart_apps/run_live.sh --mode live
```

This selects the checksum-pinned experimental ConeNet v1 at `training_runs/conenet_v1_20261014_114447/fit/weights/best.onnx`, using that run's `model.json` manifest. ConeNet maps raw output channels to `blue`, `large_orange`, `orange`, `unknown`, `yellow`, in that order. To select the original one-class detector, use `./autokart_apps/run_live.sh --mode live --detector coneslayer`.

To inspect the exported ConeNet v2 `last_resume.pt` checkpoint in the same disarmed live dashboard, use `./autokart_apps/run_live.sh --mode live --detector conenet-v2`. The launcher loads the checksum-pinned `models/conenet_v2/model.onnx` export. The 2026-10-14 Jarvis bench capture and its actual V2 boxes are saved in `phase_01_perception/recordings/conenet_v2_hp60c_live_20261014_224140` and `phase_01_perception/recordings/conenet_v2_hp60c_boxes_20261014_224140` under `autonomy_stack_phase1`. The visible orange bench cones were predominantly predicted as yellow, so V2 color labels remain unqualified. The runtime keeps vehicle authority disabled.

Open `http://127.0.0.1:8768/live`. The camera is opened by one native bridge owner. Stop with Ctrl-C. A second live process will contend for the same HP60C; close the first before changing modes. The launcher builds the React UI if its source entry changed. To rebuild after other frontend edits, run `cd phase_sim && npm run build`.

For the deterministic software demonstration:

```bash
./autokart_apps/run_live.sh --mode sim
```

The source badge reads `SIMULATED`. Simulated pose permits a diagnostic Pure Pursuit target and steering candidate. It never authorizes motion.

To capture at most 100 actual frames and then replay them through the same SDK:

```bash
./autokart_apps/run_live.sh --mode live \
  --record /home/mark1/Desktop/autonomous_kart/_autokart_audit/aut05-capture \
  --max-record-frames 100
./autokart_apps/run_live.sh --mode replay \
  --replay /home/mark1/Desktop/autonomous_kart/_autokart_audit/aut05-capture
```

Use a new recording directory for each run. Replay verifies SHA-256 hashes of recorded RGB and depth arrays and shows `REPLAY` in the source badge. Dashboard replay uses recorded detections and metric candidates, then runs the shared geometry pipeline; it does not rerun the newly selected detector. The separate ConeNet validation re-detected raw HP60C arrays explicitly. The server shows `REPLAY_COMPLETE` when the recording ends. The frame JSON has the same ID as its RGB and depth image URLs.

The default bind is localhost. To expose the diagnostic on a trusted LAN, create a private token file containing at least 24 random characters and launch with `--host <Jarvis-address> --token-file <path>`. The token gates all diagnostic API and image requests. Use TLS or a trusted tunnel for access across an untrusted network; the built-in HTTP server is not a public deployment endpoint. No write API is provided.

## Current qualifications and fault behavior

- ConeNet v1 exposes five classes, but its HP60C field classification and distance accuracy remain unqualified. A checksum and a working dashboard do not qualify its blue/yellow predictions as track boundaries. `unknown` stays `unknown`; it is never relabeled as a blue or yellow observation. The ConeSlayer rollback detects orange only.
- The current Jarvis live check found visible orange cones but no ConeNet detections in a measured 1,217-frame window. On one saved frame, ConeSlayer detected an orange cone while ConeNet's highest raw class score remained below its threshold. See `_autokart_audit/conenet-v1-live-metrics.json` and `_autokart_audit/conenet-v1-live-frame-comparison.json`; use the rollback when orange detections are needed for diagnosis.
- HP60C RAW16 zeros stay invalid. The depth preview uses black for invalid pixels and reports the valid fraction and raw-unit median. The configured metres-per-raw-unit value, RGB/depth registration, and camera mount are not field qualified. Distances and bird's-eye positions are diagnostic candidates. Projected paths are withheld from camera RGB until calibration and pose are qualified.
- Live mode has no validated pose source. Missing pose, invalid geometry, stale frame, and unknown stopping capability never authorize vehicle motion. `/ready` returns HTTP 503 while these qualifications are missing, even if `/health` reports that the web process is running.
- Frame-timeout and initial calibration delay keep the single camera owner alive while the stream starts. A disconnect clears the current snapshot, closes the source, and retries. Recording safety transitions are written to `events.jsonl`. The example service file is provided for review; it has not been installed or enabled.
- Browser `Pause display` marks its frame as historical. It does not stop the camera or turn old geometry into a current observation. The bird's-eye switches only draw edges, gates, boundaries, path, and controller target contained in the current SDK result.

## Inspect and rollback

`/health` is process liveness, `/ready` is qualified readiness, and `/api/snapshot` is the current diagnostic JSON. The `Export current diagnostic` button saves that JSON. Logs may be bounded with `--log <path> --max-log-frames <count>`; recording is bounded by `--max-record-frames`. Stop the foreground process with Ctrl-C. There is no firmware or Pi deployment in this stage.

The Jarvis demonstration also ran as a transient user service with `Restart=on-failure`. Its main process was deliberately killed once and the service opened a new live frame. The camera disconnect path and Pi service were not tested. For this running Jarvis unit, use `systemctl --user status autokart-live-diagnostic.service`, `systemctl --user restart autokart-live-diagnostic.service`, or `systemctl --user stop autokart-live-diagnostic.service`. The checked-in example unit has not been installed or enabled.

AUT-05 release evidence lives in `_autokart_audit/AUT05_ABC_RELEASE_EVIDENCE.json`. Software checks are distinguished from the unexecuted Pi, cone survey, 30-minute soak, and VCU gates.
