std_msgs/Header header
uint32 frame_id         # frame id of the newest detections folded into this map
bool valid
string status
float32 corridor_width_m
ConeDetection[] cones   # odom frame
geometry_msgs/Point[] centerline   # odom frame, ordered forward
