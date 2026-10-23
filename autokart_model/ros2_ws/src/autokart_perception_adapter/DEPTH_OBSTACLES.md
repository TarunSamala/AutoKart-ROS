# Conservative depth obstacle processing

`DepthObstacleConfig` and
`extract_depth_obstacles(depth, config, *, calibration_verified, depth_alignment_verified)`
provide a ROS-free extractor. All configuration fields are required; there are no
fallback intrinsics, transforms, units, vehicle dimensions or physical thresholds.
See the dataclass for exact field names. Depth is a rectangular indexable 2D array
and values times depth_scale_m must represent optical-axis Z metres. A radial-range
device needs its documented conversion before this API.

Supply measured fx/fy/cx/cy, image dimensions, depth scale/range, rigid 4x4
T_base_camera mapping camera optical coordinates into base_link, calibration identity,
explicit half-open ROI `(u0,v0,u1,v1)`, surveyed ground_z_m in base_link, ground
tolerance, raised-obstacle height band, corridor near/far/half-width, spatial cell
size, samples-per-cell and permitted invalid-depth fraction. Standard base_link
origin is not assumed to lie on the ground. Qualification flags require actual
boolean true and attest both calibration and depth/frame alignment; they cannot
replace measurement records or calibrated-hash verification by the caller.

Raw depth projects as `(u-cx)*Z/fx, (v-cy)*Z/fy, Z` then transforms to base_link.
Ground returns must agree with the supplied surveyed height/tolerance. Raised
returns within the configured obstacle height band form connected XY occupancy
clusters. Every cluster returns actual observed point count, min/max height,
base_link polygon and a circle enclosing that polygon. Cell half-width padding
includes discretization uncertainty. These footprints represent observed surfaces;
their unseen extents remain unknown. No semantic class is guessed.

Output dictionary fields: valid, frame_id, calibration_hash, obstacles,
coverage_complete, coverage_region, reason_codes, invalid_depth_fraction,
observed_cells, total_cells, measured_point_count, unknown_cells. `valid` means
calibrated processing executed, and can be true while coverage_complete=false.
A downstream clearance decision must require both. Calibration/schema failure
returns valid=false, coverage_complete=false, empty obstacles and explicit reason.

Coverage is bounded to the supplied base_link corridor, depth range and pixel ROI.
Each corridor cell requires measured floor or raised-surface returns. Unobserved
cells, depth holes, occluded far regions and unclassified height returns keep
coverage_complete=false. No interpolation, dilation or temporal persistence fills
unknown coverage cells. A clear measured region establishes only this discretized
surface coverage; it is not a physical free-space or release certificate. Cell
resolution and sample requirement must be validated for the smallest relevant
hazard. Overhead and below-ground returns outside configured acceptance bands
stay unclassified. A nonplanar ground surface needs a separately measured model;
this API intentionally accepts only an explicit ground-plane height.

The caller owns source timestamp pairing, camera identity/hash verification,
provenance, capture-aligned pose transformation and freshness/expiry. The module
has no ROS publication, camera opener, actuator access or safety authority. In
live deployment the missing measurements keep flags false and outputs invalid.
All tests use invented SYNTHETIC geometry and are not physical evidence.

Computational bounds: ROI <=1,000,000 pixels and coverage grid <=100,000 cells;
violations reject explicitly. No native inference or ROS dependencies are needed.
