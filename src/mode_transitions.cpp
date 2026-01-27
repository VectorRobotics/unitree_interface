#include "unitree_interface/mode_transitions.hpp"

#include "unitree_interface/control_modes.hpp"
#include "unitree_interface/unitree_sdk_wrapper.hpp"

namespace unitree_interface {

    /*
    Transitions to EmergencyMode are not idempotent. Re-invoking the transition will trigger
    an attempt to enter damping mode each time. Although, logically speaking, it is not possible to
    re-enter EmeregencyMode since we cannot transition away from it.

    Other transitions are idempotent only if the sdk calls themselves are idempotent.

    Currently, all transitions must return either the mode we're starting from, or the mode
    we expect to end up in. In case of errors, if required, we must peform a rollback.
    */

    // ========== std::monostate ==========
    ControlMode Transition<std::monostate, IdleMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        return sdk_wrapper.create_idle_mode();
    }

    ControlMode Transition<std::monostate, EmergencyMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        /*
        We only hold std::monostate when the system has been freshly initialized. The hope is that
        we initially do have high-level control services enabled to allow for the direct execution of
        the emergency stop procedure.
        */

        auto emergency_mode = sdk_wrapper.create_emergency_mode();

        if (emergency_mode.damp(sdk_wrapper)) {
            return emergency_mode;
        }

        return std::monostate{};
    }

    // ========== IdleMode ==========
    ControlMode Transition<IdleMode, HighLevelMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        // TODO: Check if the mode string here is appropriate for the version of the Motion
        // Control Service we're running with
        if (sdk_wrapper.has_active_mode() || sdk_wrapper.select_mode("mcf")) {
            return sdk_wrapper.create_high_level_mode();
        }

        return sdk_wrapper.create_idle_mode();
    }

    ControlMode Transition<IdleMode, LowLevelMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        if (!sdk_wrapper.has_active_mode() || sdk_wrapper.release_mode()) {
            return sdk_wrapper.create_low_level_mode();
        }

        return sdk_wrapper.create_idle_mode();
    }

    ControlMode Transition<IdleMode, EmergencyMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        /*
        Our modes don't truly reflect unitree's internal controller state. We can land up in
        a situation wherein we transition from LowLevelMode to IdleMode and attempt to transition
        to EmergencyMode. No longer knowing whether the internal controller itself is in "high-level"
        or "low-level" mode, if we erroneously attempt to enter damping mode without high-level
        control services active, we'll land up in some real shit.
        */

        if (!sdk_wrapper.has_active_mode()) {
            // Not in high-level mode - attempt to transition
            auto possible_high_level_mode = Transition<IdleMode, HighLevelMode>::execute(sdk_wrapper);

            if (!std::holds_alternative<HighLevelMode>(possible_high_level_mode)) {
                return sdk_wrapper.create_idle_mode();
            }
        }

        auto emergency_mode = sdk_wrapper.create_emergency_mode();

        if (!emergency_mode.damp(sdk_wrapper)) {
            return sdk_wrapper.create_idle_mode();
        }

        return emergency_mode;
    }

    // ========== HighLevelMode ==========
    ControlMode Transition<HighLevelMode, IdleMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        return sdk_wrapper.create_idle_mode();
    }

    ControlMode Transition<HighLevelMode, LowLevelMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        if (!sdk_wrapper.has_active_mode() || sdk_wrapper.release_mode()) {
            return sdk_wrapper.create_low_level_mode();
        }

        return sdk_wrapper.create_high_level_mode();
    }

    ControlMode Transition<HighLevelMode, EmergencyMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        auto emergency_mode = sdk_wrapper.create_emergency_mode();

        if (emergency_mode.damp(sdk_wrapper)) {
            return emergency_mode;
        }

        return sdk_wrapper.create_high_level_mode();
    }

    // ========== LowLevelMode ==========
    ControlMode Transition<LowLevelMode, IdleMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        return sdk_wrapper.create_idle_mode();
    }

    ControlMode Transition<LowLevelMode, HighLevelMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        // TODO: Check if the mode string here is appropriate for the version of the Motion
        // Control Service we're running with
        if (sdk_wrapper.has_active_mode() || sdk_wrapper.select_mode("mcf")) {
            return sdk_wrapper.create_high_level_mode();
        }

        return sdk_wrapper.create_low_level_mode();
    }

    ControlMode Transition<LowLevelMode, EmergencyMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        auto possible_high_level_mode = Transition<LowLevelMode, HighLevelMode>::execute(sdk_wrapper);

        if (!std::holds_alternative<HighLevelMode>(possible_high_level_mode)) {
            return sdk_wrapper.create_low_level_mode();
        }

        auto emergency_mode = sdk_wrapper.create_emergency_mode();

        if (emergency_mode.damp(sdk_wrapper)) {
            return emergency_mode;
        }

        // The internal controller should be high-level at this point, so we need to rollback to low-level
        auto possible_low_level_mode = Transition<HighLevelMode, LowLevelMode>::execute(sdk_wrapper);

        if (std::holds_alternative<LowLevelMode>(possible_low_level_mode)) {
            return possible_low_level_mode;
        }

        // damp() failed, and we also failed to transition back to low-level
        RCLCPP_ERROR(
            sdk_wrapper.get_logger(),
            "Call to damp failed during %s to %s transition. "
            "The system will be left in emergency mode. Repeated calls to damp(estop) may be required",
            ControlModeTraits<LowLevelMode>::name(),
            ControlModeTraits<EmergencyMode>::name()
        );

        return emergency_mode;
    }

} // namespace unitree_interface