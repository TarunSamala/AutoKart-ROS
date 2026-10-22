# Rendered sensors

Native camera/depth/LiDAR backends must retain Phase 6 sensor pose/calibration,
capture timing, scan/exposure intervals, latency, quality, and faults. Debug
depth/semantics must stay on debug-only channels. None are implemented in the
dependency-free follower session.
