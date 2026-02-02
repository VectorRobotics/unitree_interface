#ifndef VECTOR_UNITREE_SDK_WRAPPER_HPP
#define VECTOR_UNITREE_SDK_WRAPPER_HPP

#include "unitree_interface_msgs/msg/joint_commands.hpp"

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include <rclcpp/logger.hpp>

#include <memory>
#include <string>
#include <mutex>

// ========== Forward declarations ==========
namespace rclcpp {
    class Logger;
}

namespace unitree::robot::b2 {
    class MotionSwitcherClient;
}

namespace unitree::robot::g1 {
    class LocoClient;
    class AudioClient;
}

namespace unitree_interface {

    // ========== Aliases ==========
    using LowCmd = unitree_hg::msg::dds_::LowCmd_;
    using LowState = unitree_hg::msg::dds_::LowState_;

    class UnitreeSDKWrapper {
    public:
        explicit UnitreeSDKWrapper(
            std::string network_interface,
            rclcpp::Logger logger
        );

        // Can't copy
        UnitreeSDKWrapper(const UnitreeSDKWrapper&) = delete;
        UnitreeSDKWrapper& operator=(const UnitreeSDKWrapper&) = delete;

        // Can't move
        UnitreeSDKWrapper(UnitreeSDKWrapper&&) = delete;
        UnitreeSDKWrapper& operator=(UnitreeSDKWrapper&&) = delete;

        ~UnitreeSDKWrapper();

        [[nodiscard]]
        rclcpp::Logger get_logger() const { return logger_; }

        bool initialize(
            float msc_timeout,
            float loco_client_timeout,
            float audio_client_timeout
        );

        [[nodiscard]]
        bool is_initialized() const { return initialized_; }

        [[nodiscard]]
        std::pair<std::string, std::string> get_current_mode() const;

        [[nodiscard]]
        bool has_active_mode() const;

        [[nodiscard]]
        LowState get_low_state();

        // ========== General capabilities ==========
        bool release_mode();

        bool select_mode(const std::string& mode_name);

        // ========== High-level capabilities ==========
        bool send_velocity_command(
            float vx,
            float vy,
            float vyaw
        );

        bool damp();

        bool stand_up();

        bool set_balance_mode(const std::uint8_t balance_mode);

        bool start();

        // ========== Low-level capabilities ==========
        void send_joint_commands(
            const unitree_interface_msgs::msg::JointCommands& message
        );

        // ========== Audio capabilities ==========
        bool set_volume(std::uint8_t volume);

        bool send_speech_command(const std::string& message);

    private:
        void initialize_clients(
            float msc_timeout,
            float loco_client_timeout,
            float audio_client_timeout
        );

        void initialize_low_level_machinery();

        // ========== Callbacks ==========
        void low_state_callback(const void* message);

        // ========== Member variables ==========
        std::string network_interface_;
        rclcpp::Logger logger_;
        bool initialized_{false};

        // ========== Clients ==========
        std::unique_ptr<unitree::robot::b2::MotionSwitcherClient> msc_;
        std::unique_ptr<unitree::robot::g1::LocoClient> loco_client_;
        std::unique_ptr<unitree::robot::g1::AudioClient> audio_client_;

        // ========== Low-level stuff ==========
        // const std::string arm_cmd_topic_{"rt/arm_sdk"};
        const std::string low_cmd_topic_{"rt/lowcmd"};
        const std::string low_state_topic_{"rt/lowstate"};
        // const std::string torso_imu_topic_{"rt/secondary_imu"};

        const std::uint8_t mode_pr_{0}; // Always use PR mode (command joint angles)
        const std::uint8_t mode_machine_{2}; // 29 DoF Unitree G1

        LowState low_state_;
        std::mutex low_state_mutex_;

        unitree::robot::ChannelPublisherPtr<LowCmd> low_cmd_pub_;
        unitree::robot::ChannelSubscriberPtr<LowState> low_state_sub_;
    };

} // namespace unitree_interface

#endif // VECTOR_UNITREE_SDK_WRAPPER_HPP
