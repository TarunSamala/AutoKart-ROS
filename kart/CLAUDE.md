# kart/ — orientation

This directory holds **two separate layers**. Know which one you're touching
before you change anything.

## Layer A — generic SIL simulator (`kart_sim/`)

A pure-Python, no-ROS simulation of the kart's dynamics, sensors, and a
handful of mission/scenario implementations (skidpad, braking, autocross,
and `my_track`). Nothing here talks to real hardware. This is where
`my_track_algorithm.py`, `my_track_mission.py`, `my_track_runner.py`, and
`my_track_matrix.py` live — see [`docs/MY_TRACK.md`](../docs/MY_TRACK.md) for
the full history of that scenario: every bug found and fixed, what's still
open, and the ground-truth completion criterion used to grade a run.

Run its tests from `kart/`:

```bash
python3 -m pytest -p no:anyio tests/ --ignore=tests/gazebo_follower
```

(`gazebo_follower` needs a running ROS/Gazebo stack and isn't part of this
suite.)

## Layer B — the real ROS mission stack (`ros2_ws/`)

The actual flight code that will run on the physical kart: VCU firmware
bridge, safety monitor, planner, controller, dashboard gateway, etc., under
`ros2_ws/src/autokart_*`. **`feat/my-track` does not touch this layer** — by
design, per [`docs/my_track_spec/RECON.md`](../docs/my_track_spec/RECON.md).
my_track exists only in the Layer A simulator; feeding its cone layout into
the real planner/controller nodes is an explicitly deferred follow-up, not
done here.

## Safety rules — never touch, regardless of task

- **VCU firmware** (`ros2_ws/src/autokart_vcu_safety/`, `autokart_safety_bridge/`)
- **E-stop logic**, in firmware or in any safety-monitor node
- **Command-expiry / watchdog behaviour** on the motion command path
- **The 50 Hz control-loop contract** between planner/controller and VCU

These exist to keep a physical vehicle from doing something dangerous if
software hangs or sends garbage. A SIL-simulator change (Layer A) has no way
to need a change here — if a task seems to require one, that's a sign the
task is out of scope for this directory, not a reason to make the change.

## Shared config

- Kart dimension/dynamics profiles: `kart/configs/karts/*.yaml` (`shared.yaml`,
  `built.yaml`, plus provisional `prototype*.yaml`). Each file marks which
  fields are MEASURED vs. ASSUMED in its own header comment — read that
  before trusting a number.
- HP60C camera sensor model: `kart/configs/sensors/hp60c.yaml`.
- my_track's cone layout: `docs/my_track_spec/my_track_layout.json`.

## Before trusting any of this with a real vehicle

Nothing in `kart_sim/` has been validated against real hardware. See the
"Measurements still owed" section at the bottom of
[`docs/MY_TRACK.md`](../docs/MY_TRACK.md) for the specific list (mass, CG,
tyre model, actuator rates, real sunlight test, etc.) An independent
hardware E-stop is required for any powered test regardless of what this
simulation reports, and the first powered test of the real vehicle must be a
supervised creep at very low speed.
