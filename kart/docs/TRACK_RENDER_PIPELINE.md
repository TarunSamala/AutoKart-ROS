# Track render pipeline

Future CARLA/Unreal conversion must export Phase 5 centreline/boundaries,
elevation/banking, kerbs, barriers, corners, and optical/semantic/LiDAR material
metadata. Physics tire friction remains owned by the simulator backend and is
not inferred from a rendered material.
