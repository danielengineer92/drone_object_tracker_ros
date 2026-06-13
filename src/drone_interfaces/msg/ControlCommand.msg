# Movement command output
builtin_interfaces/Time stamp

# Command type
string command_type  # VELOCITY, POSITION, IDLE

# Velocity commands (m/s, body frame)
float32 velocity_forward
float32 velocity_right
float32 velocity_down
float32 yaw_rate

# Whether this command was actually sent to the drone
bool executed
string execution_status  # SENT, BLOCKED_SAFETY, BLOCKED_AUTONOMOUS_DISABLED, SIMULATED

# Source tracking error that generated this command
float32 source_error_x
float32 source_error_y