# Host-received status from the NOOP-only ESP32-S3 bench firmware.
# This is NOT production vehicle state or actuator feedback.
builtin_interfaces/Time receipt_time
string source
uint16 protocol_version
uint64 boot_id
uint64 vcu_uptime_ms
uint64 last_accepted_sequence
bool agent_online
uint8 controller_state
uint32 fault_bits
uint64 last_command_age_ms
bool has_accepted_command
bool packet_valid
bool stale
string reason
bool vehicle_authorized
