# Unitree Interface

The interface contains a state machine and command gate/multiplexer to prevent SDK command conflicts between high-level and low-level control modes. See [docs/](docs/) for more detailed notes:

- [Control Modes](docs/control_modes.md) - mode definitions, transitions
- [SDK & Hardware Notes](docs/sdk_notes.md) - SDK quirks, hardware observations

## TODO

- Add low-level damping mode
- Add position hold (safe stop)
- Add a way to set kp and kd values
- Investigate why the ros domain needs to be different to use the interface
- Get to a safe "mode" before transitioning High -> Low and vice-versa
- Add more descriptive logging messages
- Const-correctness?
- Thread-safety?
- Build optimizations
- Send a TTS message when the battery is low (stretch)
