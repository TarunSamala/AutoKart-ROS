std_msgs/Header header
uint32 frame_id
bool path_valid
geometry_msgs/Point[] path          # odom frame
bool stop_valid
geometry_msgs/Point stop_target     # odom frame
float32 nearest_cone_m              # forward distance from the front bumper to the cone ahead
float32 available_m                 # distance still available before the stop point
float32 required_stop_m             # v^2/(2a) + v*t_latency + margin
float32 target_speed_mps
string decision                     # HOLD NO_PATH CRUISE BRAKE STOP
