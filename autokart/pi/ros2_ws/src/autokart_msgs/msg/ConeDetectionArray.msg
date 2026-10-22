std_msgs/Header header
uint32 frame_id         # same id is carried by /autokart/camera/compressed (header.frame_id)
uint32 image_width
uint32 image_height
float32 inference_ms
bool depth_valid
bool calibration_verified
ConeDetection[] cones   # positions in the vehicle frame
