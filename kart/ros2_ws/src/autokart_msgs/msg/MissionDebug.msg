# Optional, read-only autonomy explanation channel. One message per pipeline stage.
# payload_json follows docs/mission_control_debug.md; it is never a control input.
builtin_interfaces/Time measurement_time
uint64 cycle
string frame_id
string stage
string payload_json
