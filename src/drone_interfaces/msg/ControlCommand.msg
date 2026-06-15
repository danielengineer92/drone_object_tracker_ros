# Movement command output
builtin_interfaces/Time stamp

# Command type
string command_type  # VELOCITY, POSITION, IDLE

# Velocity commands (m/s, body frame)
float32 velocity_forward
float32 velocity_right
float32 velocity_down
float32 yaw_rate

# Whether the control node approved this command for the MAVSDK bridge
bool executed
string execution_status  # SENT, BLOCKED_*, IDLE, SIMULATED

# Source tracking error that generated this command
float32 source_error_x
float32 source_error_y