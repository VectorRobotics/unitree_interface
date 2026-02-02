#ifndef VECTOR_UNITREE_INTERFACE_HPP
#define VECTOR_UNITREE_INTERFACE_HPP

#include "unitree_interface/control_modes.hpp"
#include "unitree_interface/unitree_sdk_wrapper.hpp"
#include "unitree_interface_msgs/srv/change_control_mode.hpp"
#include "unitree_interface_msgs/msg/control_mode.hpp"
#include "unitree_interface_msgs/msg/joint_commands.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>

#include <memory>

namespace unitree_interface {

    class UnitreeSDKWrapper;

    class UnitreeInterface : public rclcpp::Node {
    public:
        UnitreeInterface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        ~UnitreeInterface() = default;

        // ========== Initialization ==========
        void initialize();

    private:
        void initialize_services();

        void initialize_publishers();

        void create_subscriptions();

        void setup_mode_dependent_subscriptions();

        // ========== Callbacks ==========
        void handle_mode_change_request(
            unitree_interface_msgs::srv::ChangeControlMode::Request::SharedPtr request,
            unitree_interface_msgs::srv::ChangeControlMode::Response::SharedPtr response
        );

        void cmd_vel_callback(geometry_msgs::msg::Twist::SharedPtr message);

        // TODO: Add hybrid mode callbacks

        void joint_commands_callback(unitree_interface_msgs::msg::JointCommands::SharedPtr message);

        void estop_callback();

        void tts_callback(std_msgs::msg::String::SharedPtr message);

        // ========== Publish methods ==========
        void publish_current_mode();

        rclcpp::Logger logger_;
        std::unique_ptr<UnitreeSDKWrapper> sdk_wrapper_;
        ControlMode current_mode_;

        std::string mode_change_service_name_;
        std::string current_mode_topic_;
        std::string cmd_vel_topic_;
        std::string tts_topic_;
        std::string joint_commands_topic_;
        std::string estop_topic_;

        std::uint8_t volume_;

        rclcpp::Service<unitree_interface_msgs::srv::ChangeControlMode>::SharedPtr mode_change_service_;

        rclcpp::Publisher<unitree_interface_msgs::msg::ControlMode>::SharedPtr current_mode_pub_;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tts_sub_;
        // TODO: Add hybrid mode subscriptions
        rclcpp::Subscription<unitree_interface_msgs::msg::JointCommands>::SharedPtr joint_commands_sub_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr estop_sub_;
    };

} // namespace unitree_interface

#endif // VECTOR_UNITREE_INTERFACE_HPP