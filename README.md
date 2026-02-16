# unitree_interface

ROS 2 interface for the Unitree G1 humanoid robot (29 DoF). Bridges the Unitree SDK with standard ROS 2 topics and services.

## Node

**`unitree_interface`** (default name)

## Advertised Services

| Service | Type | Description |
|---------|------|-------------|
| `~/change_mode` | `unitree_interface_msgs/srv/ChangeControlMode` | Request a control mode transition. See `ControlMode.msg` for valid mode IDs. |
| `~/ready_locomotion` | `std_srvs/srv/Trigger` | Runs the locomotion ready sequence (damp, stand up, start). Blocking, retryable on failure. |

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~/current_mode` | `unitree_interface_msgs/msg/ControlMode` | Current control mode (latched, reliable). |
| `~/joint_states` | `sensor_msgs/msg/JointState` | Joint state feedback from the robot. Published at the SDK's `rt/lowstate` rate. Publishes for all 29 joints, using URDF joint names. |

## Subscribed Topics

| Topic | Type | Mode | Description |
|-------|------|------|-------------|
| `~/cmd_vel` | `geometry_msgs/msg/Twist` | HighLevelMode | Velocity commands (linear.x, linear.y, angular.z). |
| `~/cmd_arm` | `sensor_msgs/msg/JointState` | HighLevelMode | Arm joint commands via `rt/arm_sdk`. Uses URDF joint names for upper body joints (arms + waist). Position, velocity, and effort fields are used. Motor kp/kd are set automatically based on gearbox type. |
| `~/cmd_low` | `sensor_msgs/msg/JointState` | LowLevelMode | Direct low-level motor commands for all 29 joints. Requires `UNITREE_INTERFACE_ENABLE_LOW_LEVEL_MODE` at build time. |
| `/estop` | `std_msgs/msg/Empty` | Any | Emergency stop. Transitions to EmergencyMode (damps all joints). Global topic. |
| `~/tts` | `std_msgs/msg/String` | Any | Text-to-speech via the robot's speaker. |

## Control Modes

See [`control_modes.md`](unitree_interface/docs/control_modes.md).

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `network_interface` | string | `"eth0"` | Network interface for Unitree SDK communication. |
| `motion_switcher_client_timeout` | double | `5.0` | Timeout (s) for the motion switcher client. |
| `loco_client_timeout` | double | `10.0` | Timeout (s) for the locomotion client. |
| `audio_client_timeout` | double | `5.0` | Timeout (s) for the audio client. |
| `volume` | int | `100` | Speaker volume (0-100). |
| `mode_change_service_name` | string | `"~/change_mode"` | Service name for mode changes. |
| `ready_locomotion_service_name` | string | `"~/ready_locomotion"` | Service name for the locomotion ready sequence. |
| `current_mode_topic` | string | `"~/current_mode"` | Topic for current mode publication. |
| `cmd_vel_topic` | string | `"~/cmd_vel"` | Topic for velocity commands. |
| `cmd_arm_topic` | string | `"~/cmd_arm"` | Topic for arm commands. |
| `joint_states_topic` | string | `"~/joint_states"` | Topic for joint state feedback. |
| `tts_topic` | string | `"~/tts"` | Topic for text-to-speech. |
| `cmd_low_topic` | string | `"~/cmd_low"` | Topic for low-level commands (if enabled). |
| `estop_topic` | string | `"/estop"` | Topic for emergency stop. |

## Custom Messages (`unitree_interface_msgs`)

### `ControlMode.msg`

```
uint8 CONTROL_MODE_MONOSTATE   = 0
uint8 CONTROL_MODE_IDLE        = 1
uint8 CONTROL_MODE_HIGH_LEVEL  = 2
uint8 CONTROL_MODE_EMERGENCY   = 3
uint8 CONTROL_MODE_LOW_LEVEL   = 4

std_msgs/Header header
uint8 current_mode_id
string current_mode_name
```

### `ChangeControlMode.srv`

```
uint8 requested_mode
---
bool success
string message
```

## Joints

All joint topics use URDF-style names. See [`topology.hpp`](unitree_interface/include/unitree_interface/topology.hpp) for gearbox assignments.
