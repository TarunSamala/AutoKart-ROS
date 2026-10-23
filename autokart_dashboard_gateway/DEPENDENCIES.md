# External dependencies — autokart_dashboard_gateway

## ROS package dependencies (ordinary colcon deps)

- `rclpy` — ROS 2 Humble core.
- `python3-aiohttp` (`aiohttp>=3.8`) — the HTTP/WebSocket server.
- `autokart_msgs` is **not** a colcon dependency of this package itself —
  it is imported at **runtime**, inside the real, reused `gateway.py`'s
  `make_ros_node()`, only when `mode: "live"`. Any workspace that launches
  this node in `mode: "live"` must have `autokart_msgs` built and sourced.

## Algorithm dependency: the real `autokart_dashboard_gateway` implementation

This node does **not** vendor or reimplement the gateway. It lazily imports
`kart/ros2_ws/src/autokart_dashboard_gateway/autokart_dashboard_gateway/{gateway,core}.py`
at runtime via `gateway_package_path` — the ROS package directory that
CONTAINS the nested `autokart_dashboard_gateway` Python package, matching
the identical convention `autokart_safety` uses for its own core dependency.

**Why this package's own importable Python module is named
`autokart_dashboard_gateway_node`, not `autokart_dashboard_gateway`:** the
dependency being reused is itself a Python package named
`autokart_dashboard_gateway`. Naming this wrapper's module identically would
make `import autokart_dashboard_gateway.gateway` ambiguous with (or recurse
into) this very package. The ROS package/directory name is still
`autokart_dashboard_gateway`, matching this library's node-11 naming — only
the internal Python module differs.

## Transitive dependency: `autokart_perception` is REQUIRED even without camera display

Found while building this adapter: `gateway.py` unconditionally does
`from .camera_feed import CameraFeed` at module level, and `camera_feed.py`
unconditionally does `from autokart_perception.live_render import
depth_preview, encode_jpeg`. **Importing the real gateway module therefore
always requires `autokart_perception` to be importable too — even with
`enable_camera_feed: false` and even in `mode: "replay"`.** This is a
property of the existing, reused code (changing it is out of scope — it
would mean editing `kart/ros2_ws/src/autokart_dashboard_gateway`, which this
session was told not to touch). `perception_package_path` must point at
`autonomy_stack_phase1/phase_01_perception/src` (the parent of
`autokart_perception/`, the same canonical source `autokart_hp60c` already
documents for its own `hp60c` backend).

**A second, related finding:** this development machine has
`autokart_perception` permanently importable via a site-wide editable pip
install (`__editable__.autokart_perception-0.2.0.pth` in
`~/.local/lib/python3.10/site-packages`), independent of any path given to
this adapter. Without an explicit check, a WRONG `perception_package_path`
would not fail as this adapter's contract promises — it would silently
succeed by falling through to whatever `autokart_perception` happens to be
importable globally, which may not be the version the deployer intended.
`load_gateway()` therefore explicitly imports `autokart_perception` first
and verifies the resolved module file actually lives under the given
`perception_package_path` before proceeding, raising
`PERCEPTION_DEPENDENCY_PATH_MISMATCH` otherwise. See
`test/test_adapter.py::test_load_gateway_wrong_perception_path_is_rejected_not_silently_substituted`.

## Package-name collision risk (documented, not hidden)

If a user colcon-builds this package in the same workspace as
`kart/ros2_ws/src/autokart_dashboard_gateway` (same ROS package name,
`autokart_dashboard_gateway`), colcon will refuse to build due to a
duplicate package name. This mirrors the exact, already-flagged duplication
risk this library's `AUDIT_PHASE0.md` records for `autokart/pi/ros2_ws` vs
`kart/ros2_ws` — not a new problem introduced here, but worth stating
explicitly: this standalone node is meant to be dropped into **a different**
workspace from `kart/ros2_ws`, not built alongside it under the same name.

## What was NOT verified in this session

- No browser, no real Mission Control frontend build, was exercised against
  this wrapper's HTTP/WebSocket surface — only the underlying `aiohttp` app
  factory (already extensively tested in the reused package's own
  `test/test_live_gateway.py`, re-run unmodified as part of validation; see
  `reports/VALIDATOR_REPORT.md`).
- No real camera hardware — `enable_camera_feed` defaults `false` in both
  shipped profiles.
