# Unitree Interface

The interface contains a state machine and command gate/multiplexer to prevent SDK command conflicts between high-level and low-level control modes. See [docs/](docs/) for more detailed notes:

- [Control Modes](docs/control_modes.md) - mode definitions, transitions
- [SDK & Hardware Notes](docs/sdk_notes.md) - SDK quirks, hardware observations

## TODO

- Investigate why cmd_vel is not working
- Let go of arms upon shutdown / release call
- Add low-level damping mode
- Add position hold (safe stop)
- Add a way to set kp and kd values
- Get to a safe "mode" before transitioning High -> Low and vice-versa
- Add more descriptive logging messages
- Const-correctness?
- Thread-safety?
- Build optimizations
- Send a TTS message when the battery is low (stretch)
