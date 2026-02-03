#ifndef VECTOR_CONTROL_MODES_HPP
#define VECTOR_CONTROL_MODES_HPP

#include "unitree_interface_msgs/msg/control_mode.hpp"

#include <cstdint>
#include <type_traits>
#include <variant>

namespace unitree_interface {

    template <typename T>
    struct ControlModeTraits;

    template <typename... Ts>
    struct always_false : std::false_type {};

    // ========== std::monostate ==========
    template <>
    struct ControlModeTraits<std::monostate> {
        static constexpr std::uint8_t id = unitree_interface_msgs::msg::ControlMode::CONTROL_MODE_MONOSTATE;

        static constexpr const char* name() { return "std::monostate"; }
    };

    // ========== IdleMode ==========
    struct IdleMode {};

    template <>
    struct ControlModeTraits<IdleMode> {
        static constexpr std::uint8_t id = unitree_interface_msgs::msg::ControlMode::CONTROL_MODE_IDLE;

        static constexpr const char* name() { return "Idle"; }
    };

    // ========== HighLevelMode ==========
    struct HighLevelMode {};

    template <>
    struct ControlModeTraits<HighLevelMode> {
        static constexpr std::uint8_t id = unitree_interface_msgs::msg::ControlMode::CONTROL_MODE_HIGH_LEVEL;

        static constexpr const char* name() { return "HighLevel"; }
    };

    // ========== ArmActionMode ==========
    // struct ArmActionMode {};

    // template <>
    // struct ControlModeTraits<ArmActionMode> {
    //     static constexpr std::uint8_t id = unitree_interface_msgs::msg::ControlMode::CONTROL_MODE_ARM_ACTION;

    //     static constexpr const char* name() { return "ArmAction"; }
    // };

#ifdef UNITREE_INTERFACE_ENABLE_LOW_LEVEL_MODE
    // ========== LowLevelMode ==========
    struct LowLevelMode {};

    template <>
    struct ControlModeTraits<LowLevelMode> {
        static constexpr std::uint8_t id = unitree_interface_msgs::msg::ControlMode::CONTROL_MODE_LOW_LEVEL;

        static constexpr const char* name() { return "LowLevel"; }
    };
#endif

    // ========== EmergencyMode ==========
    struct EmergencyMode {};

    template <>
    struct ControlModeTraits<EmergencyMode> {
        static constexpr std::uint8_t id = unitree_interface_msgs::msg::ControlMode::CONTROL_MODE_EMERGENCY;

        static constexpr const char* name() { return "Emergency"; }
    };

    // clang-format off
    using ControlMode = std::variant<
        std::monostate,
        IdleMode,
        HighLevelMode,
        // ArmActionMode,
#ifdef UNITREE_INTERFACE_ENABLE_LOW_LEVEL_MODE
        LowLevelMode,
#endif
        EmergencyMode
    >;
    // clang-format on

} // namespace unitree_interface

#endif // VECTOR_CONTROL_MODES_HPP