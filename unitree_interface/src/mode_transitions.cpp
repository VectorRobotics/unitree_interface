#include "unitree_interface/mode_transitions.hpp"

#include "unitree_interface/control_modes.hpp"
#include "unitree_interface/unitree_sdk_wrapper.hpp"

namespace unitree_interface {

    /*
    Transitions to EmergencyMode are not idempotent. Re-invoking the transition will trigger
    an attempt to enter damping mode each time. Although, logically speaking, it is not possible to
    re-enter EmergencyMode since we cannot transition away from it.

    Other transitions are idempotent only if the sdk calls themselves are idempotent.

    Currently, all transitions must return either the mode we're starting from, or the mode
    we expect to end up in. In case of errors, if required, we must peform a rollback.
    */

    // ========== std::monostate ==========
    ControlMode Transition<std::monostate, IdleMode>::execute(UnitreeSDKWrapper& _) {
        return IdleMode{};
    }

    ControlMode Transition<std::monostate, EmergencyMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        /*
        We only hold std::monostate when the system has been freshly initialized. The hope is that
        we initially do have high-level control services enabled to allow for the direct execution of
        the emergency stop procedure.
        */

        const bool damp_success = sdk_wrapper.damp();

        if (!damp_success) {
            RCLCPP_ERROR(
                sdk_wrapper.get_logger(),
                "Call to damp failed during %s to Emergency transition. "
                "The system will be left in emergency mode. Repeated calls to damp(estop) may be required",
                ControlModeTraits<std::monostate>::name()
            );
        }

        return EmergencyMode{};
    }

    // ========== IdleMode ==========
    ControlMode Transition<IdleMode, HighLevelMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        if (sdk_wrapper.has_active_mode() || sdk_wrapper.select_mode("ai")) {
            return HighLevelMode{};
        }

        return IdleMode{};
    }

#ifdef UNITREE_INTERFACE_ENABLE_LOW_LEVEL_MODE
    ControlMode Transition<IdleMode, LowLevelMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        if (!sdk_wrapper.has_active_mode() || sdk_wrapper.release_mode()) {
            return LowLevelMode{};
        }

        return IdleMode{};
    }
#endif

    ControlMode Transition<IdleMode, EmergencyMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        /*
        Our modes don't truly reflect unitree's internal controller state. We can land up in
        a situation wherein we transition from LowLevelMode to IdleMode and attempt to transition
        to EmergencyMode. No longer knowing whether the internal controller itself is in "high-level"
        or "low-level" mode, if we erroneously attempt to enter damping mode without high-level
        control services active, we could possible land up in a dangerous situation.
        */

        if (!sdk_wrapper.has_active_mode()) {
            // Not in high-level mode - attempt to transition
            auto possible_high_level_mode = Transition<IdleMode, HighLevelMode>::execute(sdk_wrapper);

            if (!std::holds_alternative<HighLevelMode>(possible_high_level_mode)) {
                return IdleMode{};
            }
        }

        // At this point, we should have high-level services active
        if (!sdk_wrapper.damp()) {
            RCLCPP_ERROR(
                sdk_wrapper.get_logger(),
                "Call to damp failed during %s to Emergency transition. "
                "The system will be left in emergency mode. Repeated calls to damp(estop) may be required",
                ControlModeTraits<IdleMode>::name()
            );
        }

        return EmergencyMode{};
    }

    // ========== HighLevelMode ==========
    ControlMode Transition<HighLevelMode, IdleMode>::execute(UnitreeSDKWrapper& _) {
        return IdleMode{};
    }

#ifdef UNITREE_INTERFACE_ENABLE_LOW_LEVEL_MODE
    ControlMode Transition<HighLevelMode, LowLevelMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        if (!sdk_wrapper.has_active_mode() || sdk_wrapper.release_mode()) {
            return LowLevelMode{};
        }

        return HighLevelMode{};
    }
#endif

    ControlMode Transition<HighLevelMode, EmergencyMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        if (sdk_wrapper.damp()) {
            return EmergencyMode{};
        }

        return HighLevelMode{};
    }

#ifdef UNITREE_INTERFACE_ENABLE_LOW_LEVEL_MODE
    // ========== LowLevelMode ==========
    ControlMode Transition<LowLevelMode, IdleMode>::execute(UnitreeSDKWrapper& _) {
        return IdleMode{};
    }

    ControlMode Transition<LowLevelMode, HighLevelMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        // TODO: Add "boot" sequence (damp() -> stand_up() -> start())
        if (sdk_wrapper.has_active_mode() || sdk_wrapper.select_mode("ai")) {
            return HighLevelMode{};
        }

        return LowLevelMode{};
    }

    ControlMode Transition<LowLevelMode, EmergencyMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        auto possible_high_level_mode = Transition<LowLevelMode, HighLevelMode>::execute(sdk_wrapper);

        if (!std::holds_alternative<HighLevelMode>(possible_high_level_mode)) {
            return LowLevelMode{};
        }

        if (sdk_wrapper.damp()) {
            return EmergencyMode{};
        }

        // The internal controller should be high-level at this point, so we need to rollback to low-level
        auto possible_low_level_mode = Transition<HighLevelMode, LowLevelMode>::execute(sdk_wrapper);

        if (std::holds_alternative<LowLevelMode>(possible_low_level_mode)) {
            return EmergencyMode{};
        }

        // damp() failed, and we also failed to transition back to low-level
        RCLCPP_ERROR(
            sdk_wrapper.get_logger(),
            "Call to damp failed during %s to Emergency transition. "
            "The system will be left in emergency mode. Repeated calls to damp(estop) may be required",
            ControlModeTraits<LowLevelMode>::name()
        );

        return EmergencyMode{};
    }
#endif

    // ========== Helper functions ==========
    TransitionResult try_transition_to(
        const ControlMode& from,
        const std::uint8_t to_id,
        UnitreeSDKWrapper& sdk_wrapper
    ) {
        switch (to_id) {
            case ControlModeTraits<IdleMode>::id:
                return try_transition<IdleMode>(from, sdk_wrapper);
                break;
            case ControlModeTraits<HighLevelMode>::id:
                return try_transition<HighLevelMode>(from, sdk_wrapper);
                break;
#ifdef UNITREE_INTERFACE_ENABLE_LOW_LEVEL_MODE
            case ControlModeTraits<LowLevelMode>::id:
                return try_transition<LowLevelMode>(from, sdk_wrapper);
                break;
#endif
            default:
                return {from, false};
        }
    }

} // namespace unitree_interface