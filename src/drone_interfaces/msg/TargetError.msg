# Tracking error relative to image center
builtin_interfaces/Time stamp

# Normalized error (-1.0 to 1.0, 0.0 = centered)
float32 error_x
float32 error_y

# Target visibility
bool target_visible

# Target information
string target_class
float32 target_confidence
float32 target_area

# Tracking state
string tracking_state  # SEARCHING, LOCKED, LOST
float32 time_since_last_seen