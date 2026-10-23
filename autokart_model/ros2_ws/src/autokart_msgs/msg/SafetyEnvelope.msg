builtin_interfaces/Time timestamp
string frame_id
float64 maximum_speed_mps
float64 maximum_acceleration_mps2
float64 guaranteed_deceleration_mps2
float64 clear_distance_m
float64 stopping_distance_m
float64 lateral_limit_mps2
geometry_msgs/Polygon allowed_region
string limiting_reason
bool valid
# Provenance of the input data this envelope was computed from. One of:
# LIVE (real physical sensor), SIMULATED, REPLAY, or UNVERIFIED (unset/
# unrecognized). Self-reported by the publishing node's configuration, not
# cryptographically verified -- never trust this field alone as proof of a
# real physical sensor; treat it as a required, auditable declaration.
string data_source
