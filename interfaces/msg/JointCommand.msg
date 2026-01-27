uint8 joint_index

uint8 MOTOR_MODE_STOP  = 0 # Disable
uint8 MOTOR_MODE_SERVO = 1 # Enable
uint8 mode

float32 q      # Setpoint q
float32 dq     # Setpoint dq

float32 tau    # Feedforward

float32 kp     # Stiffness
float32 kd     # Damping