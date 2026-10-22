# Gazebo No-Double-Physics Evidence

The internal `DynamicFourWheelKart` is the only vehicle integrator. The Gazebo
model is static, contains no drive/control plugin and accepts no ROS motion
command.

## Preventive controls

- Configuration rejects `internal + dynamics_authority`.
- SDF startup rejects non-static chassis and prohibited dynamics/controller
  tokens.
- The follower subscribes only to the immutable authoritative snapshot.
- Pose and joints are set from measured state; Gazebo velocity is supplied only
  for sensor consistency.
- Contact state is not subscribed or fed into the plant.
- The follower is the only `/clock`, dynamic base TF and joint-state source.
- Any mismatch/second-authority diagnostic pauses and invalidates the run.

## Executed trials

`scripts/test_no_double_physics.sh` ran four complete deterministic laps:

| DP1 mode | Follower | Authoritative SHA-256 | Equal to disabled |
|---|---:|---|---|
| disabled | off | `28164b960133…349a2` | yes |
| low | 20 Hz | same | yes |
| normal | 100 Hz | same | yes |
| high | 200 Hz | same | yes |

Maximum authoritative trajectory difference was exactly 0 m and all four runs
finished in VCU `SAFE`.

DP2 changed the role to `dynamics_authority` while internal physics was
selected; validation rejected it. DP3 injected a Gazebo-side visual contact;
the follower recorded one visual-only contact and the internal snapshot
remained untouched.

Gazebo collision response is therefore not a physical collision model in this
mode. A future contact co-simulation interface requires a separate authority
design and parity acceptance.
