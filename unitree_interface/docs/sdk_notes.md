# SDK & Hardware Notes

## References

- Useful (AI generated) [reference](https://deepwiki.com/unitreerobotics/unitree_sdk2) for the Unitree SDK.
- From FieldAI notes: "Avoid Command Conflicts: When programming via the SDK, always ensure the robot is in Development Mode to avoid the internal motion control program interfering with your custom commands."

## ChannelFactory

- Calling `ChannelFactory::Instance()->Init()` with a domain ID and network interface invokes the creation of a DDS domain. If the domain already exists (as is the case when using a ROS node), the call will fail and the SDK will cause the node to crash.
- Calling `ChannelFactory::Instance()->Init()` with only a domain ID does not behave the same - perhaps attaching to the same domain instead of trying to create it.

## LocoClient

- `LocoClient::Stand()` requires a call to `LocoClient::Damp()` before.
- `LocoClient::ZeroTorque()` also seems to only work after a call to `LocoClient::Damp()`.
- Calls to `LocoClient::SetBalanceMode()` only take effect once the G1 "starts" (call to `LocoClient::Start()` returns 0). `LocoClient::SetBalanceMode(0)` will make it stand still until a velocity command is given. `LocoClient::SetBalanceMode(1)` will cause it to march in place.
- `LocoClient::Move()` won't work if the commanded velocity is too small (e.g, < 0.1).
- `LocoClient::Start()` puts the internal motion control FSM in state `500`. This state exhibit jittering during motion. The hand-held controller's `Regular Mode` directly sets the internal motion control FSM to `501`, which does not exhibit jittering.
- The hand-held controller's `Running Mode` directly sets the internal motion contorl FSM to `801` and seems to exhibit even better balance than `Regular Mode`.

## rt/arm_sdk

- Doesn't work if you don't call `LocoClient::Stand()`.
- Need to control the waist to compensate for arm motion.

## G1 Hardware

- Occasionally, the G1 boots up into development / debug / low-level mode directly.
- If the `mode_machine` provided in `LowCmd` doesn't match the `mode_machine` in `LowState`, the command sent to the G1 produces no action.
