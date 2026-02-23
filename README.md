# unitree_interface

ROS 2 interface for the Unitree G1 humanoid robot (29 DoF). Bridges the Unitree SDK with standard ROS 2 topics and services.

## Node

**`unitree_interface`** (default name)

## Advertised Services

| Service | Type | Description |
|---------|------|-------------|
| `~/change_mode` | `unitree_interface_msgs/srv/ChangeControlMode` | Request a control mode transition. See `ControlMode.msg` for valid mode IDs. |
| `~/ready_locomotion` | `std_srvs/srv/Trigger` | Runs the locomotion ready sequence (damp, stand up, start). Blocking, retryable on failure. |
| `~/release_arms` | `std_srvs/srv/Trigger` | Ramps down the arm SDK weight from 1.0 to 0.0, handing upper-body control back to the locomotion controller. Blocking (~5s by default). |
| `~/set_profile` | `unitree_interface_msgs/srv/SetProfile` | Switch the active gain profile. See `Profile.msg` for valid profile IDs. |

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~/current_mode` | `unitree_interface_msgs/msg/ControlMode` | Current control mode (latched, reliable). |
| `~/current_profile` | `unitree_interface_msgs/msg/Profile` | Current gain profile (latched, reliable). |
| `~/joint_states` | `sensor_msgs/msg/JointState` | Joint state feedback from the robot. Published at the SDK's `rt/lowstate` rate. Publishes for all 29 joints, using URDF joint names. |

## Subscribed Topics

| Topic | Type | Mode | Description |
|-------|------|------|-------------|
| `~/cmd_vel` | `geometry_msgs/msg/Twist` | HighLevelMode | Velocity commands (linear.x, linear.y, angular.z). |
| `~/cmd_arm` | `sensor_msgs/msg/JointState` | HighLevelMode | Arm joint commands via `rt/arm_sdk`. Uses URDF joint names for upper body joints (arms + waist). Position, velocity, and effort fields are used. Motor kp/kd are set by the active gain profile. |
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
| `release_arms_service_name` | string | `"~/release_arms"` | Service name for releasing arm SDK control. |
| `set_profile_service_name` | string | `"~/set_profile"` | Service name for switching gain profiles. |
| `release_arms_steps` | int | `250` | Number of weight ramp-down steps for arm release. |
| `release_arms_interval_ms` | int | `20` | Delay (ms) between each ramp-down step. Total duration = steps × interval. |
| `current_mode_topic` | string | `"~/current_mode"` | Topic for current mode publication. |
| `current_profile_topic` | string | `"~/current_profile"` | Topic for current profile publication. |
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

### `Profile.msg`

```
uint8 PROFILE_DEFAULT            = 0
uint8 PROFILE_DAMP               = 1
uint8 PROFILE_VISUAL_SERVO       = 2
uint8 PROFILE_WHOLE_BODY_CONTROL = 3
uint8 PROFILE_EFFORT_ONLY        = 4

std_msgs/Header header
uint8 current_profile_id
string current_profile_name
```

### `ChangeControlMode.srv`

```
uint8 requested_mode
---
bool success
string message
```

### `SetProfile.srv`

```
uint8 requested_profile
---
bool success
string message
```

## Gain Profiles

Each profile defines per-joint kp/kd gains for all 29 joints. The active profile is selected via `~/set_profile` and determines the motor gains used by `~/cmd_arm` and `~/cmd_low`. See [`profiles.hpp`](unitree_interface/include/unitree_interface/profiles.hpp) for definitions.

## Usage Examples

Switch to HighLevel mode:

```bash
ros2 service call /unitree_interface/change_mode unitree_interface_msgs/srv/ChangeControlMode "{requested_mode: 2}"
```

Switch to LowLevel mode (if enabled at build time):

```bash
ros2 service call /unitree_interface/change_mode unitree_interface_msgs/srv/ChangeControlMode "{requested_mode: 4}"
```

Run the locomotion ready sequence (damp, stand up, start):

```bash
ros2 service call /unitree_interface/ready_locomotion std_srvs/srv/Trigger "{}"
```

Release arm SDK control back to the locomotion controller:

```bash
ros2 service call /unitree_interface/release_arms std_srvs/srv/Trigger "{}"
```

Switch to the VisualServo gain profile:

```bash
ros2 service call /unitree_interface/set_profile unitree_interface_msgs/srv/SetProfile "{requested_profile: 2}"
```

Switch to the EffortOnly gain profile (kp=0, kd=0):

```bash
ros2 service call /unitree_interface/set_profile unitree_interface_msgs/srv/SetProfile "{requested_profile: 4}"
```

Send a velocity command:

```bash
ros2 topic pub --once /unitree_interface/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.15, y: 0.0}, angular: {z: 0.0}}"
```

Send a text-to-speech command:

```bash
ros2 topic pub --once /unitree_interface/tts std_msgs/msg/String "{data: 'hello world'}"
```

Emergency stop:

```bash
ros2 topic pub --once /estop std_msgs/msg/Empty "{}"
```

## Joints

All joint topics use URDF-style names. See [`topology.hpp`](unitree_interface/include/unitree_interface/topology.hpp) for joint indices and groups.
