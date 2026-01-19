#include "unitree_interface/mode_transitions.hpp"

#include "unitree_interface/unitree_sdk_wrapper.hpp"

namespace unitree_interface {

    // TODO: Figure out if transitions need to be idempotent
    // TODO: Figure out error handling on state transitions

    // ========== std::monostate ==========
    ControlMode Transition<std::monostate, IdleMode>::execute(UnitreeSDKWrapper& sdk_wrapper) {
        RCLCPP_INFO(sdk_wrapper.get_logger(), "Transitioning from std::monostate to IdleMode");

        return sdk_wrapper.create_idle_mode();
    }

    ControlMode Transition<std::monostate, EmergencyStop>::execute(UnitreeSDKWrapper &sdk_wrapper) {
        RCLCPP_WARN(sdk_wrapper.get_logger(), "Transitioning from std::monostate to EmergencyStop");

        auto emergency_stop_mode = sdk_wrapper.create_emergency_stop_mode();

        emergency_stop_mode.execute(sdk_wrapper);

        return emergency_stop_mode;
    }

    // ========== IdleMode ==========
    ControlMode Transition<IdleMode, HighLevelMode>::execute(UnitreeSDKWrapper &sdk_wrapper) {
        // TODO: Transition from IdleMode to HighLevelMode
    }

    ControlMode Transition<IdleMode, LowLevelMode>::execute(UnitreeSDKWrapper &sdk_wrapper) {
        // TODO: Transition from IdleMode to LowLevelMode
    }

    ControlMode Transition<IdleMode, EmergencyStop>::execute(UnitreeSDKWrapper &sdk_wrapper) {
        RCLCPP_WARN(sdk_wrapper.get_logger(), "Transitioning from IdleMode to EmergencyStop");

        auto emergency_stop_mode = sdk_wrapper.create_emergency_stop_mode();

        emergency_stop_mode.execute(sdk_wrapper);

        return emergency_stop_mode;
    }

    // ========== HighLevelMode ==========
    ControlMode Transition<HighLevelMode, IdleMode>::execute(UnitreeSDKWrapper &sdk_wrapper) {
        // TODO: Transition from HighLevelMode to IdleMode
    }

    ControlMode Transition<HighLevelMode, LowLevelMode>::execute(UnitreeSDKWrapper &sdk_wrapper) {
        // TODO: Transition from HighLevelMode to LowLevelMode
    }

    ControlMode Transition<HighLevelMode, EmergencyStop>::execute(UnitreeSDKWrapper &sdk_wrapper) {
        RCLCPP_WARN(sdk_wrapper.get_logger(), "Transitioning from HighLevelMode to EmergencyStop");

        auto emergency_stop_mode = sdk_wrapper.create_emergency_stop_mode();

        emergency_stop_mode.execute(sdk_wrapper);

        return emergency_stop_mode;
    }

    // ========== LowLevelMode ==========
    ControlMode Transition<LowLevelMode, IdleMode>::execute(UnitreeSDKWrapper &sdk_wrapper) {
        // TODO: Transition from LowLevelMode to IdleMode
    }

    ControlMode Transition<LowLevelMode, HighLevelMode>::execute(UnitreeSDKWrapper &sdk_wrapper) {
        // TODO: Transition from LowLevelMode to HighLevelMode
    }

    ControlMode Transition<LowLevelMode, EmergencyStop>::execute(UnitreeSDKWrapper &sdk_wrapper) {
        RCLCPP_WARN(sdk_wrapper.get_logger(), "Transitioning from LowLevelMode to EmergencyStop");

        // TODO: Transition to HighLevelMode first to enable damping

        auto emergency_stop_mode = sdk_wrapper.create_emergency_stop_mode();

        emergency_stop_mode.execute(sdk_wrapper);

        return emergency_stop_mode;
    }

} // namespace unitree_interface