#include "unitree_interface/unitree_interface.hpp"

#include "unitree_interface/mode_transitions.hpp"

namespace unitree_interface {

    UnitreeInterface::UnitreeInterface(const rclcpp::NodeOptions& options)
        : Node("unitree_interface", options),
          sdk_wrapper_(shared_from_this(), "eth0", get_logger()),
          current_mode_(std::monostate{}) {
        sdk_wrapper_.initialize();

        current_mode_ = Transition<std::monostate, IdleMode>::execute(sdk_wrapper_);
    }   

} // namespace unitree_interface