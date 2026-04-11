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

        const bool damp_success = sdk_wrapper.damp_high();

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

    ControlMode Transition<IdleMode, LowLevelMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        if (!sdk_wrapper.has_active_mode() || sdk_wrapper.release_mode()) {
            return LowLevelMode{};
        }

        return IdleMode{};
    }

    ControlMode Transition<IdleMode, EmergencyMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        if (sdk_wrapper.has_active_mode()) {
            if (!sdk_wrapper.damp_high()) {
                RCLCPP_ERROR(
                    sdk_wrapper.get_logger(),
                    "Call to damp_high failed during %s to Emergency transition. "
                    "The system will be left in emergency mode. Repeated calls to damp(estop) may be required",
                    ControlModeTraits<IdleMode>::name()
                );
            }
        } else {
            sdk_wrapper.damp_low();
        }

        return EmergencyMode{};
    }

    // ========== HighLevelMode ==========
    ControlMode Transition<HighLevelMode, IdleMode>::execute(UnitreeSDKWrapper& _) {
        return IdleMode{};
    }

    ControlMode Transition<HighLevelMode, LowLevelMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        if (!sdk_wrapper.has_active_mode() || sdk_wrapper.release_mode()) {
            return LowLevelMode{};
        }

        return HighLevelMode{};
    }

    ControlMode Transition<HighLevelMode, EmergencyMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        if (sdk_wrapper.damp_high()) {
            return EmergencyMode{};
        }

        return HighLevelMode{};
    }

    // ========== LowLevelMode ==========
    ControlMode Transition<LowLevelMode, IdleMode>::execute(UnitreeSDKWrapper& _) {
        return IdleMode{};
    }

    ControlMode Transition<LowLevelMode, HighLevelMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        if (sdk_wrapper.has_active_mode() || sdk_wrapper.select_mode("ai")) {
            return HighLevelMode{};
        }

        return LowLevelMode{};
    }

    ControlMode Transition<LowLevelMode, EmergencyMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        sdk_wrapper.damp_low();

        return EmergencyMode{};
    }

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
            case ControlModeTraits<LowLevelMode>::id:
                return try_transition<LowLevelMode>(from, sdk_wrapper);
                break;
            default:
                return {from, false};
        }
    }

} // namespace unitree_interface