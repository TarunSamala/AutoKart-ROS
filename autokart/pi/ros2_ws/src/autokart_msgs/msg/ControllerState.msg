std_msgs/Header header
string state                        # IDLE RUN STOPPING STOPPED
float32 target_speed_mps
float32 measured_speed_mps
float32 throttle_v
float32 brake_pct
float32 steer_deg_cmd               # Pure Pursuit result. Observed only: steering is not actuated in this release.
bool steer_actuated
float32 alpha_rad
float32 lookahead_m
float32 wheelbase_m
float32 lookahead_x
float32 lookahead_y
bool lookahead_valid
bool stopped
