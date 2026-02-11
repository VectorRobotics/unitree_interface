# SDK & Hardware Notes

## References

- Useful (AI generated) [reference](https://deepwiki.com/unitreerobotics/unitree_sdk2) for the Unitree SDK.
- From FieldAI notes: "Avoid Command Conflicts: When programming via the SDK, always ensure the robot is in Development Mode to avoid the internal motion control program interfering with your custom commands."

## LocoClient

- `LocoClient::Stand()` requires a call to `LocoClient::Damp()` before.
- `LocoClient::ZeroTorque()` also seems to only work after a call to `LocoClient::Damp()`.
- Calls to `LocoClient::SetBalanceMode()` only take effect once the G1 "starts" (call to `LocoClient::Start()` returns 0). `LocoClient::SetBalanceMode(0)` will make it stand still until a velocity command is given. `LocoClient::SetBalanceMode(1)` will cause it to march in place.

## G1 Hardware

- The G1 oscillates while marching/walking in "regular" (low speed) mode. This behavior doesn't occur in the "running" (higher speed) mode.
- There is a lock around the waist of the G1 that constricts some of the DoF. Removing it may help with the oscillation issue.
