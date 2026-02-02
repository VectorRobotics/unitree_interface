#ifndef VECTOR_MODE_TRANSITIONS_HPP
#define VECTOR_MODE_TRANSITIONS_HPP

#include "unitree_interface/control_modes.hpp"
#include "unitree_interface/unitree_sdk_wrapper.hpp"

#include <rclcpp/logging.hpp>
#include <variant>

namespace unitree_interface {

    // Base template - no transitions allowed
    template <typename From, typename To>
    struct Transition {
        static constexpr bool allowed = false;
    };

    // ========== std::monostate ==========
    template <>
    struct Transition<std::monostate, IdleMode> {
        static constexpr bool allowed = true;

        static ControlMode execute(UnitreeSDKWrapper&);
    };

    template <>
    struct Transition<std::monostate, EmergencyMode> {
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
    struct Transition<IdleMode, EmergencyMode> {
        static constexpr bool allowed = true;

        static ControlMode execute(UnitreeSDKWrapper& sdk_wrapper);
    };

    // ========== HighLevelMode ==========
    template <>
    struct Transition<HighLevelMode, IdleMode> {
        static constexpr bool allowed = true;

        static ControlMode execute(UnitreeSDKWrapper&);
    };

    template <>
    struct Transition<HighLevelMode, LowLevelMode> {
        static constexpr bool allowed = true;

        static ControlMode execute(UnitreeSDKWrapper& sdk_wrapper);
    };

    template <>
    struct Transition<HighLevelMode, EmergencyMode> {
        static constexpr bool allowed = true;

        static ControlMode execute(UnitreeSDKWrapper& sdk_wrapper);
    };

    // TODO: Add transitions for hybrid mode

    // ========== LowLevelMode ==========
    template <>
    struct Transition<LowLevelMode, IdleMode> {
        static constexpr bool allowed = true;

        static ControlMode execute(UnitreeSDKWrapper&);
    };

    template <>
    struct Transition<LowLevelMode, HighLevelMode> {
        static constexpr bool allowed = true;

        static ControlMode execute(UnitreeSDKWrapper& sdk_wrapper);
    };

    template <>
    struct Transition<LowLevelMode, EmergencyMode> {
        static constexpr bool allowed = true;

        static ControlMode execute(UnitreeSDKWrapper& sdk_wrapper);
    };

    // ========== Helper functions ==========
    using TransitionResult = std::pair<ControlMode, bool>;

    TransitionResult try_transition_to(
        const ControlMode& from,
        std::uint8_t to_id,
        UnitreeSDKWrapper& sdk_wrapper
    );

    template <typename ToType>
    bool can_transition(const ControlMode& from) {
        return std::visit(
            [](auto&& f) {
                using FromType = std::decay_t<decltype(f)>;

                return Transition<FromType, ToType>::allowed;
            },
            from
        );
    }

    template <typename ToType>
    TransitionResult try_transition(
        const ControlMode& from,
        UnitreeSDKWrapper& sdk_wrapper
    ) {
        if (!sdk_wrapper.is_initialized()) {
            RCLCPP_WARN(
                sdk_wrapper.get_logger(),
                "Attempted to execute a transition with an uninitialized sdk_wrapper"
            );
            return {from, false};
        }

        return std::visit(
            [&](auto&& f) -> TransitionResult {
                using FromType = std::decay_t<decltype(f)>;

                if constexpr (Transition<FromType, ToType>::allowed) {
                    RCLCPP_INFO(
                        sdk_wrapper.get_logger(),
                        "Executing transition from %s to %s",
                        ControlModeTraits<FromType>::name(),
                        ControlModeTraits<ToType>::name()
                    );
                    auto resultant_mode = Transition<FromType, ToType>::execute(sdk_wrapper);
                    bool success = std::holds_alternative<ToType>(resultant_mode);

                    return {resultant_mode, success};
                }

                RCLCPP_WARN(
                    sdk_wrapper.get_logger(),
                    "Attempted to execute an illegal transition from %s to %s",
                    ControlModeTraits<FromType>::name(),
                    ControlModeTraits<ToType>::name()
                );
                return {from, false};
            },
            from
        );
    }

} // namespace unitree_interface

#endif // VECTOR_MODE_TRANSITIONS_HPP