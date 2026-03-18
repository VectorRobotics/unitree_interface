#ifndef VECTOR_UNITREE_INTERFACE_HPP
#define VECTOR_UNITREE_INTERFACE_HPP

#include "unitree_interface/control_modes.hpp"
#include "unitree_interface/profiles.hpp"
#include "unitree_interface/topology.hpp"
#include "unitree_interface/unitree_sdk_wrapper.hpp"
#include "unitree_interface_msgs/msg/control_mode.hpp"
#include "unitree_interface_msgs/msg/profile.hpp"
#include "unitree_interface_msgs/srv/change_control_mode.hpp"
#include "unitree_interface_msgs/srv/set_profile.hpp"

#include <dynamic_params/dynamic_params.hpp>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <array>
#include <atomic>
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

        template <typename ProfileType>
        void declare_profile_gains(bool dynamic = true);

        void apply_profile_gains(const Profile& profile);

        std::uint8_t get_active_profile_id() const;

        // ========== Callbacks ==========
        void handle_mode_change_request(
            unitree_interface_msgs::srv::ChangeControlMode::Request::SharedPtr request,
            unitree_interface_msgs::srv::ChangeControlMode::Response::SharedPtr response
        );

        void handle_ready_locomotion_request(
            std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response
        );

        void handle_release_arms_request(
            std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response
        );

        void handle_set_profile_request(
            unitree_interface_msgs::srv::SetProfile::Request::SharedPtr request,
            unitree_interface_msgs::srv::SetProfile::Response::SharedPtr response
        );

        void cmd_vel_callback(geometry_msgs::msg::Twist::SharedPtr message);

        void cmd_arm_callback(sensor_msgs::msg::JointState::SharedPtr message);

#ifdef UNITREE_INTERFACE_ENABLE_LOW_LEVEL_MODE
        void cmd_low_callback(sensor_msgs::msg::JointState::SharedPtr message);
#endif

        void estop_callback();

        void tts_callback(std_msgs::msg::String::SharedPtr message);

        // ========== Publish methods ==========
        void publish_current_mode() const;

        void publish_current_profile() const;

        rclcpp::Logger logger_;
        dynamic_params::DynamicParams params_;
        std::unique_ptr<UnitreeSDKWrapper> sdk_wrapper_;
        ControlMode current_mode_;
        Profile current_profile_;
        std::atomic<bool> releasing_arms_{false};

        std::array<float, joints::num_joints> current_kp_ = Default::kp;
        std::array<float, joints::num_joints> current_kd_ = Default::kd;

        rclcpp::Service<unitree_interface_msgs::srv::ChangeControlMode>::SharedPtr mode_change_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready_locomotion_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_arms_service_;
        rclcpp::Service<unitree_interface_msgs::srv::SetProfile>::SharedPtr set_profile_service_;

        rclcpp::Publisher<unitree_interface_msgs::msg::ControlMode>::SharedPtr current_mode_pub_;
        rclcpp::Publisher<unitree_interface_msgs::msg::Profile>::SharedPtr current_profile_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tts_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_arm_sub_;
#ifdef UNITREE_INTERFACE_ENABLE_LOW_LEVEL_MODE
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_low_sub_;
#endif
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr estop_sub_;
    };

} // namespace unitree_interface

#endif // VECTOR_UNITREE_INTERFACE_HPP
