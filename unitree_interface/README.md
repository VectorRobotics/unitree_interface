# Unitree Interface

The interface contains a state machine and command gate/multiplexer to prevent SDK command conflicts between high-level and low-level control modes. See [docs/](docs/) for more detailed notes:

- [Control Modes](docs/control_modes.md) - mode definitions, transitions
- [SDK & Hardware Notes](docs/sdk_notes.md) - SDK quirks, hardware observations

## TODO

- Add low-level damping mode
- Add more descriptive logging messages
- Const-correctness?
- Build optimizations
