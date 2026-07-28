# unitree_interface

ROS 2 interface for the Unitree G1 humanoid robot (29 DoF). Bridges the Unitree SDK with standard ROS 2 topics and services.

## Building

### Clone

The Unitree SDK is a git submodule. A plain `git clone` leaves
`unitree_interface/third_party/unitree_sdk2` empty and CMake fails during configure with:

```
CMake Error at CMakeLists.txt:22 (add_subdirectory):
  The source directory .../third_party/unitree_sdk2 does not contain a CMakeLists.txt file.
```

Clone recursively:

```bash
git clone --recurse-submodules https://github.com/VectorRobotics/unitree_interface.git
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

### Dependencies

Built and tested against ROS 2 Jazzy on Ubuntu 24.04.

Every dependency resolves through `rosdep` except
[`dynamic_params`](https://github.com/VectorRobotics/vector_robotics/tree/main/src/dynamic_params),
a thin wrapper around the ROS 2 parameter API. It is used for every runtime parameter this
node declares, so the build cannot proceed without it:

```
$ rosdep install --from-paths . --ignore-src
ERROR[unitree_interface]: Cannot locate rosdep definition for [dynamic_params]

$ colcon build
CMake Error at CMakeLists.txt:18 (find_package):
  Could not find a package configuration file provided by "dynamic_params"
```

`dynamic_params` currently lives in a private repository, which means this package cannot be
built outside the VectorRobotics organization. See #TODO_ISSUE for tracking.

Build `dynamic_params` into a colcon workspace and source that workspace's overlay before
building here.

### Build

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/dynamic_params_ws/install/setup.bash
colcon build
```

## Troubleshooting

### `ModuleNotFoundError: No module named 'em'`

Symptom — message generation for `unitree_interface_msgs` fails during configure:

```
CMake Error at /opt/ros/jazzy/share/rosidl_adapter/cmake/rosidl_adapt_interfaces.cmake:57 (message):
  execute_process(/home/you/.local/bin/python3.10 -m rosidl_adapter ...) returned error code 1:
  ...
  ModuleNotFoundError: No module named 'em'
```

Cause — CMake selected a Python interpreter that ROS 2 is not installed under. `FindPython3`
defaults to `Python3_FIND_STRATEGY=LOCATION`: it walks `PATH` in order and takes the first
interpreter it finds, *without* comparing versions. If conda, pyenv, uv, or a virtualenv puts
a `python3.x` on `PATH` ahead of `/usr/bin`, CMake picks that one — and it has neither
`empy` nor ROS 2's `rosidl_adapter`. Note the interpreter path in the error message: if it is
not `/usr/bin/python3`, this is your problem.

Fix — point CMake at the interpreter ROS 2 was built against. The stale choice is cached in
`CMakeCache.txt`, so the build directory must be removed as well:

```bash
rm -rf build install
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

Alternatively, keep the interfering interpreter off `PATH` while building.

## Node

**`unitree_interface`** (default name)

## Advertised Services

| Service | Type | Description |
|---------|------|-------------|
| `~/change_mode` | `unitree_interface_msgs/srv/ChangeControlMode` | Request a control mode transition. See `ControlMode.msg` for valid mode IDs. |
| `~/ready_locomotion` | `std_srvs/srv/Trigger` | Runs the locomotion ready sequence (damp, stand up, start). Blocking, retryable on failure. |
| `~/release_arms` | `std_srvs/srv/Trigger` | Ramps down the arm SDK weight from 1.0 to 0.0, handing upper-body control back to the locomotion controller. Blocking (~5s by default). |
| `~/reset_integral_error` | `std_srvs/srv/Trigger` | Zeros out accumulated integral error for all joints and resets the integral timer. |
| `~/set_profile` | `unitree_interface_msgs/srv/SetProfile` | Switch the active gain profile. See `Profile.msg` for valid profile IDs. |
| `~/set_hand_pose` | `unitree_interface_msgs/srv/SetHandPose` | Smoothly move a Dex3 hand to a predefined pose. Requires HighLevelMode. Blocking (~1s by default). See `SetHandPose.srv` for hand/pose IDs. |

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~/current_mode` | `unitree_interface_msgs/msg/ControlMode` | Current control mode (latched, reliable). |
| `~/current_profile` | `unitree_interface_msgs/msg/Profile` | Current gain profile (latched, reliable). |
| `~/joint_states` | `sensor_msgs/msg/JointState` | Joint state feedback from the robot. Published at the SDK's `rt/lowstate` rate. Publishes for all 29 joints, using URDF joint names. |
| `~/left_hand_states` | `sensor_msgs/msg/JointState` | Left Dex3 hand joint state feedback (7 joints). |
| `~/right_hand_states` | `sensor_msgs/msg/JointState` | Right Dex3 hand joint state feedback (7 joints). |

## Subscribed Topics

| Topic | Type | Mode | Description |
|-------|------|------|-------------|
| `~/cmd_vel` | `geometry_msgs/msg/TwistStamped` | HighLevelMode | Velocity commands (twist.linear.x, twist.linear.y, twist.angular.z). |
| `~/cmd_arm` | `sensor_msgs/msg/JointState` | HighLevelMode | Arm joint commands via `rt/arm_sdk`. Uses URDF joint names for upper body joints (arms + waist). Position, velocity, and effort fields are used. Motor kp/kd are set by the active gain profile. |
| `~/cmd_low` | `sensor_msgs/msg/JointState` | LowLevelMode | Direct low-level motor commands for all 29 joints. |
| `/estop` | `std_msgs/msg/Empty` | Any | Emergency stop. Transitions to EmergencyMode (damps all joints). Global topic. |
| `~/tts` | `std_msgs/msg/String` | Any | Text-to-speech via the robot's speaker. |

## Control Modes

See [`control_modes.md`](unitree_interface/docs/control_modes.md).

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motion_switcher_client_timeout` | double | `5.0` | Timeout (s) for the motion switcher client. |
| `loco_client_timeout` | double | `10.0` | Timeout (s) for the locomotion client. |
| `audio_client_timeout` | double | `5.0` | Timeout (s) for the audio client. |
| `volume` | int | `100` | Speaker volume (0-100). |
| `mode_change_service_name` | string | `"~/change_mode"` | Service name for mode changes. |
| `ready_locomotion_service_name` | string | `"~/ready_locomotion"` | Service name for the locomotion ready sequence. |
| `release_arms_service_name` | string | `"~/release_arms"` | Service name for releasing arm SDK control. |
| `reset_integral_error_service_name` | string | `"~/reset_integral_error"` | Service name for resetting integral error. |
| `set_profile_service_name` | string | `"~/set_profile"` | Service name for switching gain profiles. |
| `set_hand_pose_service_name` | string | `"~/set_hand_pose"` | Service name for setting hand poses. |
| `release_arms_steps` | int | `250` | Number of weight ramp-down steps for arm release. |
| `release_arms_interval_ms` | int | `20` | Delay (ms) between each ramp-down step. Total duration = steps × interval. |
| `hand_pose_steps` | int | `100` | Number of interpolation steps for hand pose transitions. |
| `hand_pose_interval_ms` | int | `10` | Delay (ms) between each interpolation step. Total duration = steps × interval. |
| `current_mode_topic` | string | `"~/current_mode"` | Topic for current mode publication. |
| `current_profile_topic` | string | `"~/current_profile"` | Topic for current profile publication. |
| `cmd_vel_topic` | string | `"~/cmd_vel"` | Topic for velocity commands. |
| `cmd_arm_topic` | string | `"~/cmd_arm"` | Topic for arm commands. |
| `joint_states_topic` | string | `"~/joint_states"` | Topic for joint state feedback. |
| `tts_topic` | string | `"~/tts"` | Topic for text-to-speech. |
| `cmd_low_topic` | string | `"~/cmd_low"` | Topic for low-level commands (if enabled). |
| `estop_topic` | string | `"/estop"` | Topic for emergency stop. |
| `left_hand_states_topic` | string | `"~/left_hand_states"` | Topic for left hand state feedback. |
| `right_hand_states_topic` | string | `"~/right_hand_states"` | Topic for right hand state feedback. |

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

### `SetHandPose.srv`

```
uint8 HAND_LEFT  = 0
uint8 HAND_RIGHT = 1

uint8 POSE_BALL  = 0
uint8 POSE_POINT = 1

uint8 target_hand
uint8 target_pose
---
bool success
string message
```

## Gain Profiles

Each profile defines per-joint kp, kd, and ki gains for all 29 joints. The active profile is selected via `~/set_profile` and determines the motor gains used by `~/cmd_arm` and `~/cmd_low`. See [`profiles.hpp`](unitree_interface/include/unitree_interface/profiles.hpp) for definitions.

### Integral control

The Unitree SDK's on-board motor controller implements PD control: `torque = kp*(q_desired - q_actual) + kd*(dq_desired - dq_actual) + tau`. There is no native ki field. Integral control is computed application-side: the position error is accumulated over time and `ki * integral_error` is added to the effort (tau) field before sending commands to the robot.

Integral error is automatically reset on mode changes and profile switches. It can also be manually reset via the `~/reset_integral_error` service.

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

Point with the left hand:

```bash
ros2 service call /unitree_interface/set_hand_pose unitree_interface_msgs/srv/SetHandPose "{target_hand: 0, target_pose: 1}"
```

Ball with the right hand:

```bash
ros2 service call /unitree_interface/set_hand_pose unitree_interface_msgs/srv/SetHandPose "{target_hand: 1, target_pose: 0}"
```

Reset integral error:

```bash
ros2 service call /unitree_interface/reset_integral_error std_srvs/srv/Trigger "{}"
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
ros2 topic pub --once /unitree_interface/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 1.15, y: 0.0}, angular: {z: 0.0}}}"
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

To command the joints one by one

```
ros2 topic pub --once /position_control sensor_msgs/msg/JointState " \
  { header: {}, \
  name: [
    'waist_pitch_joint',
    'waist_roll_joint',
    'waist_yaw_joint',
    'right_shoulder_pitch_joint', 
    'left_shoulder_pitch_joint', 
    'left_shoulder_roll_joint', 
    'right_shoulder_roll_joint', 
    'left_shoulder_yaw_joint', 
    'right_shoulder_yaw_joint',
    'right_elbow_joint',
    'left_elbow_joint', 
    'right_wrist_pitch_joint',
    'left_wrist_pitch_joint',
    'right_wrist_yaw_joint',
    'left_wrist_yaw_joint',
    'lright_wrist_roll_joint',
    'left_wrist_roll_joint'], 
position: [0.0,0.0,0.0,-1.5,-1.5,0.0,0.0,0.3,-0.3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]}"
```
