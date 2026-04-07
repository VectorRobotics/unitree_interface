#ifndef VECTOR_UNITREE_SDK_WRAPPER_HPP
#define VECTOR_UNITREE_SDK_WRAPPER_HPP

#include "unitree_interface/topology.hpp"

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/HandCmd_.hpp>
#include <unitree/idl/hg/HandState_.hpp>

#include <rclcpp/logger.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

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
    using HandCmd = unitree_hg::msg::dds_::HandCmd_;
    using HandState = unitree_hg::msg::dds_::HandState_;

    class UnitreeSDKWrapper {
    public:
        explicit UnitreeSDKWrapper(
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

        void set_joint_states_publisher(
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher
        );

        // ========== General capabilities ==========
        bool release_mode();

        bool select_mode(const std::string& mode_name);

        void reset_integral_error();

        // ========== High-level capabilities ==========
        bool send_velocity_command(
            float vx,
            float vy,
            float vyaw
        );

        bool damp_high();

        bool stand_up();

        bool set_balance_mode(const std::uint8_t balance_mode);

        bool start();

        void send_arm_commands(
            const std::array<bool, embodiment::num_joints>& active,
            const std::array<float, embodiment::num_joints>& position,
            const std::array<float, embodiment::num_joints>& velocity,
            const std::array<float, embodiment::num_joints>& effort,
            const std::array<float, embodiment::num_joints>& kp,
            const std::array<float, embodiment::num_joints>& kd,
            const std::array<float, embodiment::num_joints>& ki
        );

        void release_arms(int steps, int interval_ms);

        // ========== Low-level capabilities ==========
        void damp_low();

        void send_low_commands(
            const std::array<bool, embodiment::num_joints>& active,
            const std::array<float, embodiment::num_joints>& position,
            const std::array<float, embodiment::num_joints>& velocity,
            const std::array<float, embodiment::num_joints>& effort,
            const std::array<float, embodiment::num_joints>& kp,
            const std::array<float, embodiment::num_joints>& kd,
            const std::array<float, embodiment::num_joints>& ki
        );

        // ========== Hand capabilities ==========
        void send_hand_command(
            hands::Side side,
            const std::array<float, hands::num_joints>& positions
        );

        std::array<float, hands::num_joints> get_hand_position(hands::Side side);

        void set_hand_states_publishers(
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_pub,
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_pub
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

        void initialize_arm_sdk_machinery();

        void initialize_low_level_machinery();

        void initialize_hand_machinery();

        // ========== Low-level capabilities ==========
        LowCmd construct_low_cmd(
            const std::array<bool, embodiment::num_joints>& active,
            const std::array<float, embodiment::num_joints>& position,
            const std::array<float, embodiment::num_joints>& velocity,
            const std::array<float, embodiment::num_joints>& effort,
            const std::array<float, embodiment::num_joints>& kp,
            const std::array<float, embodiment::num_joints>& kd,
            float weight = 0.0F
        );

        // ========== Callbacks ==========
        void low_state_callback(const void* message);
        void left_hand_state_callback(const void* message);
        void right_hand_state_callback(const void* message);

        // ========== Member variables ==========
        rclcpp::Logger logger_;
        bool initialized_{false};

        // ========== Clients ==========
        std::unique_ptr<unitree::robot::b2::MotionSwitcherClient> msc_;
        std::unique_ptr<unitree::robot::g1::LocoClient> loco_client_;
        std::unique_ptr<unitree::robot::g1::AudioClient> audio_client_;

        // ========== Low-level stuff ==========
        const std::string arm_sdk_topic_{"rt/arm_sdk"};
        const std::string low_cmd_topic_{"rt/lowcmd"};
        const std::string low_state_topic_{"rt/lowstate"};
        // const std::string torso_imu_topic_{"rt/secondary_imu"};

        const std::uint8_t mode_pr_{0}; // Always use PR mode (command joint angles)
        std::uint8_t mode_machine_{0};

        mutable std::mutex position_mutex_;
        std::array<float, embodiment::num_joints> actual_position_{};

        mutable std::mutex integral_mutex_;
        std::array<float, embodiment::num_joints> integral_error_{};

        unitree::robot::ChannelPublisherPtr<LowCmd> arm_sdk_pub_;
        unitree::robot::ChannelPublisherPtr<LowCmd> low_cmd_pub_;
        unitree::robot::ChannelSubscriberPtr<LowState> low_state_sub_;

        // ========== Hand stuff ==========
        const std::string left_hand_cmd_topic_{"rt/dex3/left/cmd"};
        const std::string right_hand_cmd_topic_{"rt/dex3/right/cmd"};
        const std::string left_hand_state_topic_{"rt/lf/dex3/left/state"};
        const std::string right_hand_state_topic_{"rt/lf/dex3/right/state"};

        mutable std::mutex hand_position_mutex_;
        std::array<float, hands::num_joints> left_hand_position_{};
        std::array<float, hands::num_joints> right_hand_position_{};

        unitree::robot::ChannelPublisherPtr<HandCmd> left_hand_cmd_pub_;
        unitree::robot::ChannelPublisherPtr<HandCmd> right_hand_cmd_pub_;
        unitree::robot::ChannelSubscriberPtr<HandState> left_hand_state_sub_;
        unitree::robot::ChannelSubscriberPtr<HandState> right_hand_state_sub_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_hand_states_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_hand_states_pub_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    };

} // namespace unitree_interface

#endif // VECTOR_UNITREE_SDK_WRAPPER_HPP
