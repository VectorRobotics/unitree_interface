uint8 CONTROL_MODE_MONOSTATE   = 0
uint8 CONTROL_MODE_IDLE        = 1
uint8 CONTROL_MODE_HIGH_LEVEL  = 2
uint8 CONTROL_MODE_EMERGENCY   = 3
uint8 CONTROL_MODE_LOW_LEVEL   = 4

std_msgs/Header header

uint8 current_mode_id
string current_mode_name
