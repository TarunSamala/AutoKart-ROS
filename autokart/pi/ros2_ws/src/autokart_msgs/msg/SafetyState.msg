std_msgs/Header header
string state                        # DISARMED ARMING ARMED FAULT
bool arm_allowed
bool vcu_heartbeat_ok
bool startup_ready
bool stop_latched
string[] reasons
StartupStage[] stages
