# Unitree Interface

## Modes

- **std::monostate**: A stand-in for a logically uninitialized control mode. This might not be necessary anymore.
- **IdleMode**: A mode wherein no commands will be forwarded to the G1.
- **HighLevelMode**: A mode wherein high-level commands (as provided by the LocoClient) can be forwarded to the G1.
- **LowLevelMode**: A mode wherein we can control motor gains and provide joint-level commands.
- **EmergencyMode**: On transitioning to this mode, the G1 will immediately attempt to regain high-level control and then attempt to enter damping mode. Transitioning away from this mode is not allowed.

Our modes don't truly reflect unitree's internal controller state. We can land up in a situation wherein we transition from LowLevelMode to IdleMode and attempt to transition to EmergencyMode. We must be cognizant of the internal controller state before attempting to enter damping mode while transitioning to EmergencyMode.

Currently, we attempt to enter HighLevelMode each time we transition from either IdleMode to EmergencyMode (and similarly for LowLevelMode). It might be a better idea to enforce the invariant that IdleMode always exists with the high-level control services active to make transitioning to EmergencyMode both faster and potentially more reliable.

## Some Notes

- Useful (AI generated) [reference](https://deepwiki.com/unitreerobotics/unitree_sdk2) for the Unitree SDK.
- From FieldAI notes: "Avoid Command Conflicts: When programming via the SDK, always ensure the robot is in Development Mode (Step 9) to avoid the internal motion control program interfering with your custom commands." The interface internally contains a state machine and command gate/multiplexer to safeguard against exactly this problem.
- **Damping mode is not available in development/debug/low-level mode**. We must transition to high-level mode (if we're not in it already) before activating damping or risk damage to the G1.
- High-level services are made active upon boot. For us, this means we're implicitly in HighLevelMode.
- MotionSwitcherClient::ReleaseMode() will turn off the ai_sport service and the G1 will enter debug mode. When this happens, the G1 will also lose control (i.e. go limp).
- MotionSwitcherclient::SelectMode("ai") will turn on the ai_sport service and bring the high-level services back online.
- Upon re-enabling the ai_sport service, the G1 will enter zero torque mode (similar to what happens when the G1 boots up) and go limp.
- Call to LocoClient::Stand() isn't very reliable. Can fail (the G1 will not stand) even if the return code is 0. May require a call to Damp() before.
- Calls to SetBalanceMode() only take effect once the G1 "starts" (call to Start() returns 0). SetBalanceMode(0) will make it stand still until a velocity command is given. SetBalanceMode(1) will cause it to march in place.
- ZeroTorque() also seems to only work after a call to Damp().
- The G1 oscillates while marching/walking in "regular" (low speed) mode. This behavior doesn't occur in the "running" (higher speed) mode.
- There is a lock around the waste of the G1 that constricts some of the DoF. Removing it may help with oscillation issue.

## TODO

- Test if adding a delay after initialize() makes the call to Stand() more reliable.
- Add hybrid / arm-action mode
- Get to a safe "mode" before transitioning High -> Low and vice-versa [Maybe not needed]
- Add more descriptive logging messages
- Const-correctness?
- Thread-safety?
- Republish low-state if it can't be read
- Build optimizations
- Send a TTS message when the battery is low (stretch)
