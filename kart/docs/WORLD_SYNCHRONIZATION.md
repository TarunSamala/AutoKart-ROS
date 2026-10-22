# World synchronization

Visual ticks use a configured fixed delta and authoritative simulation timestamp.
Pause freezes the world follower; reset clears it and leaves it paused. Simulator
to Unreal mapping is metres `(x, y, z) -> (x, -y, z)` with yaw sign inversion;
round-trip mapping is unit tested. Engine sensor callbacks must be postprocessed
through Phase 6 capture/delivery timing rather than treated as measurement time.
