#ifndef VECTOR_UNITREE_INTERFACE_HPP
#define VECTOR_UNITREE_INTERFACE_HPP

#include "unitree_interface/control_modes.hpp"
#include "unitree_interface/unitree_sdk_wrapper.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

namespace unitree_interface {

    class UnitreeSDKWrapper;

    class UnitreeInterface : public rclcpp::Node {
    public:
        UnitreeInterface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        ~UnitreeInterface() = default;

    private:
        UnitreeSDKWrapper sdk_wrapper_;
        ControlMode current_mode_;
    };

} // namespace unitree_interface

#endif // VECTOR_UNITREE_INTERFACE_HPP