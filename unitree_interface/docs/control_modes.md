# Control Modes

![Control mode transitions](control_modes.png)

## Modes

| ID | Mode | Name | Description |
|----|------|------|-------------|
| 0 | `std::monostate` | std::monostate | The system holds this state only immediately after initialization, before any transition has been requested and the sdk wrapper has not been initialized yet. |
| 1 | `IdleMode` | Idle | Initialized, no commands are forwarded to the G1. |
| 2 | `HighLevelMode` | High | Locomotion via `cmd_vel`, arm control via `cmd_arm`. |
| 3 | `EmergencyMode` | Emergency | Emergency stop state. Cannot transition out. All joints will be damped, with no gravity compensation. |
| 4 | `LowLevelMode` | LowLevel | Direct motor control via `cmd_low`. Build-time opt-in. |

## Transitions

### EmergencyMode transitions

Our modes don't truly reflect Unitree's internal controller state. This creates a hazard: we can transition from LowLevelMode to IdleMode and then attempt EmergencyMode without knowing whether the internal controller is in high-level or low-level mode. Damping mode is **not available** in low-level/debug/development mode - attempting it without high-level services active risks damage to the G1.

To handle this, transitions to EmergencyMode from IdleMode and LowLevelMode first ensure high-level services are active before calling `UnitreeSDKWrapper::damp()`. If `UnitreeSDKWrapper::damp()` fails, the system is still left in EmergencyMode with an error log indicating repeated estop calls may be needed.

### HighLevelMode entry

`MotionSwitcherClient::SelectMode("ai")` activates the `ai_sport` service. If not already active, upon re-enabling, the G1 enters zero torque mode (goes limp), similar to boot behavior.

### LowLevelMode entry

`MotionSwitcherClient::ReleaseMode()` turns off the `ai_sport` service and puts the G1 into debug mode. The G1 will lose control (go limp) on entry.
