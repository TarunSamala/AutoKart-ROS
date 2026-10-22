# Dashboard safety

The VCU state, independent safety state, connection state, fault count, E-stop
condition, speed, and “NO ACTUATOR AUTHORITY” banner are global UI elements.
Emergency stop and fault lockout use persistent critical styling.

The safety page displays motor-enable and contactor requests, command age,
controlled-stop phase, dynamic speed-limit candidates, stopping-distance
components, and active faults. Actuator pages expose the brake feedback chain
and requested/actual traction mismatch.

Safety rules:

- No dashboard endpoint can command an actuator or VCU state.
- POST/PUT/PATCH/DELETE return 405.
- WebSocket direction is server-to-browser telemetry.
- Invalid/backward telemetry is marked invalid.
- Live disconnect activates a stale overlay and freezes the pose.
- Debug ground truth is labelled and disabled by default.
- UI state always follows reported state, never requested state.

