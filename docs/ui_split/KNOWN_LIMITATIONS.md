# Known Limitations (Dashboard / Lab split)

1. **No qualified metric geometry on the real camera.** Calibration is `MOUNT_CONFIGURED_UNVERIFIED` and depth scale unsurveyed, so the dashboard's bird's-eye correctly withholds Delaunay/Gabriel/gates/path for live data. The animated all-layer view is only populated for SIMULATED/REPLAY sources (labelled). Qualifying it needs the physical survey + calibration session — not software.
2. **No live odometry/VCU/encoder data** in this environment, so Localization, Mission and Vehicle pages show explicit "not reported" states; they were verified in jsdom only. ROS shows `ROS_OFFLINE`; safety envelope `UNKNOWN`.
3. **Smooth animation not watched in a browser.** The pane throttled `requestAnimationFrame`; interpolation is implemented and unit-structured but unobserved. Stale/offline transitions were verified in jsdom, not by unplugging hardware.
4. **Camera hot-unplug/replug NOT TESTED** (only simulated via a missing bridge library).
5. **Lab gaps:** `/perception`, `/depth-calibration`, `/benchmark`, `/faults` are explicit "not implemented"; `/scenario`, `/dataset`, `/logs` partial. LIVE READ-ONLY provenance is disabled by design.
6. **Legacy nav text.** `src/shared/LabNav.tsx` (shared with the legacy root app) still says "ROS gateway :8765" in the Lab's `/core` page nav. Text only; the Lab cannot reach :8765.
7. **`/mission` proxy rule** uses `^/mission/`; any production static server for `dist-*` needs equivalent reverse-proxy rules (dev/preview servers have them).
8. **No code-split guarantee between legacy apps:** the legacy root app (`/`, `/live`, `/ops`, `/shared`, `/demo`) is unchanged and still ships as one bundle; the new apps are separate bundles.
9. **Events page** logs only transitions seen by that browser session; lost on reload. No vehicle log source exists.
10. **Services/units:** systemd units are examples only — never installed, never run on a Raspberry Pi. Target is a Pi; everything here ran on x86_64.
11. **Dev servers (Vite)** are used for `--dev` runs; `--prod` uses `vite preview`, not a hardened web server.
12. **`/api/v1/diagnostics/pipeline`** exists only in perception service instances started after the change (a pre-existing instance returns 404; the dashboard handles that).
13. **tsconfig:** `tsc -p .` is a no-op (`files: []`); use `-p tsconfig.app.json`.
14. **No git** in `phase_sim/`, `kart/`, `scripts/`: rollback is file-copy (`_rollback/`).
15. Not covered by this work: independent E-stop, physical braking/steering, vehicle autonomy — all NO-GO.
