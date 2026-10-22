# Sensor Architecture

**Verdict: PROTOTYPE_TEST_REQUIRED.** `AsyncSensor` owns an independent phase-accumulator schedule, seeded jitter stream, local clock offset/drift, immutable message header, and bounded-in-practice delivery queue. Capture time, measurement time, delivery time, and local-clock time are separate. Ground truth is passed into measurement functions and is never itself published as a sensor message.

Implemented headless sensors are a planar IMU and wheel encoder. Camera, RGB-D, LiDAR, steering/brake/electrical sensor interfaces and replay remain pending.
