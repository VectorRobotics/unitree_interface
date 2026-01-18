#ifndef VECTOR_CONTROL_TRANSITION_HPP
#define VECTOR_CONTROL_TRANSITION_HPP

#include "unitree_interface/control_modes.hpp"

#include <rclcpp/logging.hpp>

namespace unitree_interface {

    class UnitreeSDKWrapper;

    // Base template - no transitions allowed
    template <typename From, typename To>
    struct Transition {
        static constexpr bool allowed = false;
    };

    // ========== std::monostate ==========
    template <>
    struct Transition<std::monostate, IdleMode> {
        static constexpr bool allowed = true;

        static ControlMode execute(UnitreeSDKWrapper& sdk_wrapper);
    };

    template <>
    struct Transition<std::monostate, EmergencyStop> {
        static constexpr bool allowed = true;

        static ControlMode execute(UnitreeSDKWrapper& sdk_wrapper);
    };

    // ========== IdleMode ==========
    template <>
    struct Transition<IdleMode, HighLevelMode> {
        static constexpr bool allowed = true;

        static ControlMode execute(UnitreeSDKWrapper& sdk_wrapper);
    };

    template <>
    struct Transition<IdleMode, LowLevelMode> {
        static constexpr bool allowed = true;

        static ControlMode execute(UnitreeSDKWrapper& sdk_wrapper);
    };

    template <>
    struct Transition<IdleMode, EmergencyStop> {
        static constexpr bool allowed = true;

        static ControlMode execute(UnitreeSDKWrapper& sdk_wrapper);
    };

    // ========== HighLevelMode ==========
    template <>
    struct Transition<HighLevelMode, IdleMode> {
        static constexpr bool allowed = true;

        static ControlMode execute(UnitreeSDKWrapper& sdk_wrapper);
    };

    template <>
    struct Transition<HighLevelMode, LowLevelMode> {
        static constexpr bool allowed = true;

        static ControlMode execute(UnitreeSDKWrapper& sdk_wrapper);
    };

    template <>
    struct Transition<HighLevelMode, EmergencyStop> {
        static constexpr bool allowed = true;

        static ControlMode execute(UnitreeSDKWrapper& sdk_wrapper);
    };

    // ========== LowLevelMode ==========
    template <>
    struct Transition<LowLevelMode, IdleMode> {
        static constexpr bool allowed = true;

        static ControlMode execute(UnitreeSDKWrapper& sdk_wrapper);
    };

    template <>
    struct Transition<LowLevelMode, HighLevelMode> {
        static constexpr bool allowed = true;

        static ControlMode execute(UnitreeSDKWrapper& sdk_wrapper);
    };

    template <>
    struct Transition<LowLevelMode, EmergencyStop> {
        static constexpr bool allowed = true;

        static ControlMode execute(UnitreeSDKWrapper& sdk_wrapper);
    };

}

#endif