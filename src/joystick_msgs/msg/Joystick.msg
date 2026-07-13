std_msgs/Header header
control_msgs/Control control_msg
float32 accel_ratio
bool e_stop_en
bool is_recording
# true => request MANUAL (joystick) driving, false => AUTO (/control) driving.
# Latched by the joystick A button; the control_node arbitrates on this flag.
bool manual_mode
