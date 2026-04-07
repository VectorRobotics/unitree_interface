#ifndef VECTOR_UNITREE_INTERFACE_HPP
#define VECTOR_UNITREE_INTERFACE_HPP

#include "unitree_interface/control_modes.hpp"
#include "unitree_interface/profiles.hpp"
#include "unitree_interface/topology.hpp"
#include "unitree_interface/unitree_sdk_wrapper.hpp"
#include "unitree_interface_msgs/msg/control_mode.hpp"
#include "unitree_interface_msgs/msg/profile.hpp"
#include "unitree_interface_msgs/srv/change_control_mode.hpp"
#include "unitree_interface_msgs/srv/set_hand_pose.hpp"
#include "unitree_interface_msgs/srv/set_profile.hpp"

#include <dynamic_params/dynamic_params.hpp>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <array>
#include <atomic>
#include <memory>
#include <shared_mutex>

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

        void handle_reset_integral_error_request(
            std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response
        );

        void handle_set_hand_pose_request(
            unitree_interface_msgs::srv::SetHandPose::Request::SharedPtr request,
            unitree_interface_msgs::srv::SetHandPose::Response::SharedPtr response
        );

        void cmd_vel_callback(geometry_msgs::msg::TwistStamped::SharedPtr message);

        void cmd_arm_callback(sensor_msgs::msg::JointState::SharedPtr message);

        void cmd_low_callback(sensor_msgs::msg::JointState::SharedPtr message);

        void estop_callback();

        void tts_callback(std_msgs::msg::String::SharedPtr message);

        // ========== Publish methods ==========
        void publish_current_mode() const;

        void publish_current_profile() const;

        // ========== Core state ==========
        rclcpp::Logger logger_;
        dynamic_params::DynamicParams params_;
        std::unique_ptr<UnitreeSDKWrapper> sdk_wrapper_;

        mutable std::shared_mutex state_mutex_;
        ControlMode current_mode_;
        Profile current_profile_;
        std::array<float, embodiment::num_joints> current_kp_ = Default::kp;
        std::array<float, embodiment::num_joints> current_kd_ = Default::kd;
        std::array<float, embodiment::num_joints> current_ki_ = Default::ki;

        std::atomic<bool> releasing_arms_{false};

        // ========== Callback groups ==========
        rclcpp::CallbackGroup::SharedPtr estop_cbg_;
        rclcpp::CallbackGroup::SharedPtr command_cbg_;
        rclcpp::CallbackGroup::SharedPtr service_cbg_;

        // ========== Services ==========
        rclcpp::Service<unitree_interface_msgs::srv::ChangeControlMode>::SharedPtr mode_change_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready_locomotion_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_arms_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_integral_error_service_;
        rclcpp::Service<unitree_interface_msgs::srv::SetProfile>::SharedPtr set_profile_service_;
        rclcpp::Service<unitree_interface_msgs::srv::SetHandPose>::SharedPtr set_hand_pose_service_;

        // ========== Publishers ==========
        rclcpp::Publisher<unitree_interface_msgs::msg::ControlMode>::SharedPtr current_mode_pub_;
        rclcpp::Publisher<unitree_interface_msgs::msg::Profile>::SharedPtr current_profile_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_hand_states_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_hand_states_pub_;

        // ========== Subscriptions ==========
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tts_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_arm_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_low_sub_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr estop_sub_;
    };

} // namespace unitree_interface

#endif // VECTOR_UNITREE_INTERFACE_HPP
