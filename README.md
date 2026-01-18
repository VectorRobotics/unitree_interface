# Unitree Interface

## Modes
- **std::monostate**: A stand-in for a logically uninitialized mode. Since mode creation methods are private (the node cannot access them, only the sdk wrapper and transition structs can), we initialize to a monostate and immediately perform a transition to IdleMode.
- **IdleMode**: A mode wherein no commands will be forwarded to the G1.
- **HighLevelMode**: A mode wherein high-level actions (as defined by unitree) can be forwarded to the G1.
- **LowLevelMode**: A mode wherein we can control motor gains and provide joint-level commands.
- **EmergencyStop**: On transitioning to this mode, the G1 will immediately attempt to regain high-level control and execute joint-level damping. Transitioning away from this mode is not allowed.

## Notes for the G1


## TODO:
- Figure out all low-level sdk commands required
- Wrap all required low-level sdk commands
- Expose wrapped sdk commands via the appropriate modes
- Populate the execute methods of all transitions
- Add pubs/subs for the sdk wrapper
- Add pubs/subs for the vector stack
- Build optimizations