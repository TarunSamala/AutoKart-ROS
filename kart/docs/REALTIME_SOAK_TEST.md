# Real-Time Soak Test

The soak command is:

```bash
./scripts/run_realtime_soak.sh 1800
```

It retains the 1 kHz internal plant, VCU, independent safety, actuator,
electrical and thermal models. Timing samples are bounded at 200,000 values per
task and health samples are written every 10 simulated seconds.

The first attempt on the 2 m-wide research track exited its limits after one
lap and is retained as a failed probe. The qualification soak therefore uses
`configs/tracks/phase16_soak_circle.yaml`, an explicitly simulation-only,
8 m-wide durability course with unchanged centreline and vehicle physics.

The completed 2026-07-30 core run produced:

- 1,800.02 s simulated in 354.05 s wall time (`RTF 5.084`);
- 233 durability-course laps;
- zero physics, VCU, and independent-safety deadline misses;
- RSS 32.15 to 55.92 MiB (23.77 MiB growth, bounded after warm-up);
- zero queue growth;
- battery SOC 0.900 to 0.409;
- final motor temperature 40.80 °C;
- final controller temperature 43.75 °C.

The final SOC decrease and temperature rise came from normal modeled operation;
no artificial thermal acceleration was used. Brake temperature remains
`NOT_MODELLED`.

Native Gazebo, dashboard, recorder, and race-control operation for the full
30-minute duration remains `NOT_EXECUTED`; the completed result is a headless
core durability qualification. The evidence file is
`artifacts/phase16/soak/soak_results.json`.
