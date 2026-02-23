uint8 PROFILE_DEFAULT            = 0
uint8 PROFILE_DAMP               = 1
uint8 PROFILE_VISUAL_SERVO       = 2
uint8 PROFILE_WHOLE_BODY_CONTROL = 3
uint8 PROFILE_EFFORT_ONLY        = 4

std_msgs/Header header

uint8 current_profile_id
string current_profile_name
