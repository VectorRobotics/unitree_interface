#ifndef VECTOR_UNITREE_SDK_WRAPPER_HPP
#define VECTOR_UNITREE_SDK_WRAPPER_HPP

#include <rclcpp/node.hpp>
#include <rclcpp/logging.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <memory>
#include <string>

// Forward declarations
namespace unitree::robot::b2 {
    class MotionSwitcherClient;
}

namespace unitree::robot::g1 {
    class LocoClient;
}

namespace unitree_interface {

    // Some more forward declarations
    class IdleMode;
    class HighLevelMode;
    class LowLevelMode;
    class EmergencyStop;
    template <typename From, typename To> struct Transition;

    class UnitreeSDKWrapper {
    public:
        using MotionSwitcherClientPtr = std::unique_ptr<unitree::robot::b2::MotionSwitcherClient>;
        using LocoClientPtr = std::unique_ptr<unitree::robot::g1::LocoClient>;

        explicit UnitreeSDKWrapper(
            const rclcpp::Node::SharedPtr& node,
            std::string network_interface,
            rclcpp::Logger logger = rclcpp::get_logger("unitree_interface")
        );

        // Can't copy
        UnitreeSDKWrapper(const UnitreeSDKWrapper&) = delete;
        UnitreeSDKWrapper& operator=(const UnitreeSDKWrapper&) = delete;

        // Can move
        UnitreeSDKWrapper(UnitreeSDKWrapper&&);
        UnitreeSDKWrapper& operator=(UnitreeSDKWrapper&&);

        ~UnitreeSDKWrapper();

        rclcpp::Logger get_logger() const { return logger_; }

        bool initialize();

        bool is_initialized() const {
            return initialized_;
        }

        std::pair<std::string, std::string> get_current_mode() const;

        bool has_active_mode() const;

    private:
        // Mode creation
        IdleMode create_idle_mode() const;

        HighLevelMode create_high_level_mode() const;

        LowLevelMode create_low_level_mode() const;

        EmergencyStop create_emergency_stop_mode() const;

        // Internal capabilities
        bool release_mode();

        bool select_mode(const std::string& mode_name);

        bool damp();

        bool emergency_stop();

        // High-level capabilities
        bool send_velocity_command_impl(const geometry_msgs::msg::Twist& cmd);

        // TODO: Add other high-level capabilities

        // Low-level capabilities
        // TODO: Add low-level capabilities

        // Friends
        friend class IdleMode;
        friend class HighLevelMode;
        friend class LowLevelMode;
        friend class EmergencyStop;

        template <typename From, typename To> friend struct Transition;

        std::string network_interface_;
        rclcpp::Logger logger_;
        bool initialized_;

        MotionSwitcherClientPtr msc_;
        LocoClientPtr loco_client_;
    };

} // namespace unitree_interface

#endif // VECTOR_UNITREE_SDK_WRAPPER_HPP
