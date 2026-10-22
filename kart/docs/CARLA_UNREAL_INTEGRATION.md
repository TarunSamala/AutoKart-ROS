# CARLA/Unreal integration

The authority model is simulator/Chrono physics authoritative with CARLA/Unreal
as a visual/sensor follower. `VisualFollowerSession` accepts authoritative
states on explicit fixed ticks and has no actuator or physics-integration API,
preventing double physics. CARLA and Unreal bindings were not detected.
