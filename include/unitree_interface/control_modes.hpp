#ifndef VECTOR_CONTROL_MODES_HPP
#define VECTOR_CONTROL_MODES_HPP

#include <string>
#include <variant>

namespace unitree_interface {

    class UnitreeSDKWrapper;

    class IdleMode {
    public:
        std::string name() const { return "Idle"; }

    private:
        friend class UnitreeSDKWrapper;

        IdleMode() = default;
    };

    class HighLevelMode {
    public:
        std::string name() const { return "HighLevel"; }

        // TODO: Add high-level control capabilities

    private:
        friend class UnitreeSDKWrapper;

        HighLevelMode() = default;
    };

    class LowLevelMode {
    public:
        std::string name() const { return "LowLevel"; }

        // TODO: Add low-level control capabilities

    private:
        friend class UnitreeSDKWrapper;

        LowLevelMode() = default;
    };

    class EmergencyStop {
    public:
        std::string name() const { return "EmergencyStop"; }

        bool execute(UnitreeSDKWrapper& sdk_wrapper);

    private:
        friend class UnitreeSDKWrapper;

        EmergencyStop() = default;
    };

    // clang-format off
    using ControlMode = std::variant<
        std::monostate,
        IdleMode,
        HighLevelMode,
        LowLevelMode,
        EmergencyStop
    >;
    // clang-format on

    bool can_transition(const ControlMode& from, const ControlMode& to);

    ControlMode execute_transition(
        const ControlMode& from,
        const ControlMode& to,
        UnitreeSDKWrapper& unitree_interface
    );

} // namespace joint_cmd_mux

#endif