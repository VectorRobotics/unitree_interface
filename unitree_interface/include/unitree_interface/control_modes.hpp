#ifndef VECTOR_CONTROL_MODES_HPP
#define VECTOR_CONTROL_MODES_HPP

#include "unitree_interface_msgs/msg/control_mode.hpp"
#include "unitree_interface_msgs/msg/joint_commands.hpp"

#include <cstdint>
#include <string>
#include <type_traits>
#include <variant>

namespace unitree_interface {

    // ========== Control modes ==========
    struct IdleMode {};

    struct HighLevelMode {};

    // TODO: Implement this
    // struct HybridMode {}; // Rename to ArmActionMode?

    // TODO: Disable this?
    struct LowLevelMode {};

    struct EmergencyMode {};

    // ====================
    // clang-format off
    using ControlMode = std::variant<
        std::monostate,
        IdleMode,
        HighLevelMode,
        // HybridMode,
        LowLevelMode,
        EmergencyMode
    >;
    // clang-format on

    // ========== Control mode traits ==========
    template <typename T>
    struct ControlModeTraits;

    template <>
    struct ControlModeTraits<std::monostate> {
        static constexpr std::uint8_t id = unitree_interface_msgs::msg::ControlMode::CONTROL_MODE_MONOSTATE;

        static constexpr const char* name() { return "std::monostate"; }
    };

    template <>
    struct ControlModeTraits<IdleMode> {
        static constexpr std::uint8_t id = unitree_interface_msgs::msg::ControlMode::CONTROL_MODE_IDLE;

        static constexpr const char* name() { return "Idle"; }
    };

    template <>
    struct ControlModeTraits<HighLevelMode> {
        static constexpr std::uint8_t id = unitree_interface_msgs::msg::ControlMode::CONTROL_MODE_HIGH_LEVEL;

        static constexpr const char* name() { return "HighLevel"; }
    };

    // template <>
    // struct ControlModeTraits<HybridMode> {
    //     static constexpr std::uint8_t id = unitree_interface_msgs::msg::ControlMode::CONTROL_MODE_HYBRID;

    //     static constexpr const char* name() { return "Hybrid"; }
    // };

    template <>
    struct ControlModeTraits<LowLevelMode> {
        static constexpr std::uint8_t id = unitree_interface_msgs::msg::ControlMode::CONTROL_MODE_LOW_LEVEL;

        static constexpr const char* name() { return "LowLevel"; }
    };

    template <>
    struct ControlModeTraits<EmergencyMode> {
        static constexpr std::uint8_t id = unitree_interface_msgs::msg::ControlMode::CONTROL_MODE_EMERGENCY;

        static constexpr const char* name() { return "Emergency"; }
    };

    template <typename... Ts>
    struct always_false : std::false_type {};

} // namespace unitree_interface

#endif // VECTOR_CONTROL_MODES_HPP