# High-level command only. Integrated nodes must send this to the VCU bridge.
builtin_interfaces/Time timestamp
builtin_interfaces/Time expiry
string source
uint32 sequence
string mode
float64 target_speed_mps
float64 steering_angle_rad
float64 curvature_1pm
bool use_curvature
float64 acceleration_limit_mps2
float64 deceleration_limit_mps2
bool valid
uint32 integrity
