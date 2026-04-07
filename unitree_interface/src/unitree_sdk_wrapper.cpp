#include "unitree_interface/unitree_sdk_wrapper.hpp"
#include "unitree_interface/profiles.hpp"
#include "unitree_interface/topology.hpp"
#include "unitree_interface/crc.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp> // Seems misleading, but is actually correct
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cstddef>
#include <utility>

namespace unitree_interface {

    UnitreeSDKWrapper::UnitreeSDKWrapper(
        rclcpp::Logger logger
    ) : logger_(std::move(logger)) {
    }

    UnitreeSDKWrapper::~UnitreeSDKWrapper() = default;

    bool UnitreeSDKWrapper::initialize(
        const float msc_timeout,
        const float loco_client_timeout,
        const float audio_client_timeout
    ) {
        if (initialized_) {
            RCLCPP_WARN(logger_, "UnitreeSDKWrapper already initialized");
            return true;
        }

        try {
            initialize_clients(
                msc_timeout,
                loco_client_timeout,
                audio_client_timeout
            );
            initialize_arm_sdk_machinery();
            initialize_low_level_machinery();
            initialize_hand_machinery();

            initialized_ = true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Failed to initialize UnitreeSDKWrapper: %s", e.what());
            initialized_ = false;
        }

        return initialized_;
    }

    void UnitreeSDKWrapper::initialize_clients(
        const float msc_timeout,
        const float loco_client_timeout,
        const float audio_client_timeout
    ) {
        // Initialize ChannelFactory
        unitree::robot::ChannelFactory::Instance()->Init(0);
        RCLCPP_INFO(logger_, "ChannelFactory initialized");

        // Initialize MotionSwitcherClient
        msc_ = std::make_unique<unitree::robot::b2::MotionSwitcherClient>();
        msc_->SetTimeout(msc_timeout);
        msc_->Init();
        RCLCPP_INFO(logger_, "MotionSwitcherClient initialized");

        // Initialize LocoClient
        loco_client_ = std::make_unique<unitree::robot::g1::LocoClient>();
        loco_client_->SetTimeout(loco_client_timeout);
        loco_client_->Init();
        RCLCPP_INFO(logger_, "LocoClient initialized");

        // Initialize AudioClient
        audio_client_ = std::make_unique<unitree::robot::g1::AudioClient>();
        audio_client_->SetTimeout(audio_client_timeout);
        audio_client_->Init();
        RCLCPP_INFO(logger_, "AudioClient initialized");
    }

    void UnitreeSDKWrapper::initialize_arm_sdk_machinery() {
        // Initialize the arm-sdk command publisher
        arm_sdk_pub_ = std::make_shared<unitree::robot::ChannelPublisher<LowCmd>>(arm_sdk_topic_);
        arm_sdk_pub_->InitChannel();
        RCLCPP_INFO(logger_, "Arm-SDK command publisher initialized");
    }

    void UnitreeSDKWrapper::initialize_low_level_machinery() {
        // Initialize the low-level command publisher
        low_cmd_pub_ = std::make_shared<unitree::robot::ChannelPublisher<LowCmd>>(low_cmd_topic_);
        low_cmd_pub_->InitChannel();
        RCLCPP_INFO(logger_, "Low-level command publisher initialized");

        // Initialize the low-level state subscription
        low_state_sub_ = std::make_shared<unitree::robot::ChannelSubscriber<LowState>>(low_state_topic_);
        low_state_sub_->InitChannel(
            [this](const void* message){
                low_state_callback(message);
            },
            10 // NOLINT
        );
        RCLCPP_INFO(logger_, "Low-level state subscription created");
    }

    void UnitreeSDKWrapper::initialize_hand_machinery() {
        left_hand_cmd_pub_ = std::make_shared<unitree::robot::ChannelPublisher<HandCmd>>(left_hand_cmd_topic_);
        left_hand_cmd_pub_->InitChannel();

        right_hand_cmd_pub_ = std::make_shared<unitree::robot::ChannelPublisher<HandCmd>>(right_hand_cmd_topic_);
        right_hand_cmd_pub_->InitChannel();

        RCLCPP_INFO(logger_, "Hand command publishers initialized");

        left_hand_state_sub_ = std::make_shared<unitree::robot::ChannelSubscriber<HandState>>(left_hand_state_topic_);
        left_hand_state_sub_->InitChannel(
            [this](const void* message) { left_hand_state_callback(message); },
            10 // NOLINT
        );

        right_hand_state_sub_ = std::make_shared<unitree::robot::ChannelSubscriber<HandState>>(right_hand_state_topic_);
        right_hand_state_sub_->InitChannel(
            [this](const void* message) { right_hand_state_callback(message); },
            10 // NOLINT
        );

        RCLCPP_INFO(logger_, "Hand state subscriptions created");
    }

    std::pair<std::string, std::string> UnitreeSDKWrapper::get_current_mode() const {
        if (!initialized_ || !msc_) {
            RCLCPP_ERROR(logger_, "UnitreeSDKWrapper not initialized");
            return {};
        }

        std::string form;
        std::string name;

        const std::int32_t ret = msc_->CheckMode(form, name);

        if (ret != 0) {
            RCLCPP_WARN(logger_, "CheckMode failed with error code: %d", ret);
            return {};
        }

        return {form, name};
    }

    bool UnitreeSDKWrapper::has_active_mode() const {
        const auto [form, name] = get_current_mode();

        return !name.empty();
    }

    // ========== General capabilities ==========
    bool UnitreeSDKWrapper::release_mode() {
        if (!initialized_ || !msc_) {
            RCLCPP_ERROR(logger_, "UnitreeSDKWrapper not initialized");
            return false;
        }

        std::string form;
        std::string name;

        msc_->CheckMode(form, name);

        if (name.empty()) {
            RCLCPP_INFO(logger_, "No active motion control mode - already released");
            return true;
        }

        RCLCPP_INFO(logger_, "Attempting to release mode: %s (form: %s)", name.c_str(), form.c_str());
        const std::int32_t ret = msc_->ReleaseMode();

        if (ret == 0) {
            RCLCPP_INFO(logger_, "ReleaseMode succeeded");
            return true;
        }

        RCLCPP_WARN(logger_, "ReleaseMode failed with error code: %d", ret);
        return false;
    }

    bool UnitreeSDKWrapper::select_mode(const std::string& mode_name) {
        if (!initialized_ || !msc_) {
            RCLCPP_ERROR(logger_, "UnitreeSDKWrapper not initialized");
            return false;
        }

        if (mode_name.empty()) {
            RCLCPP_ERROR(logger_, "Mode name cannot be empty");
            return false;
        }

        RCLCPP_INFO(logger_, "Selecting mode: %s", mode_name.c_str());
        const std::int32_t ret = msc_->SelectMode(mode_name);

        if (ret == 0) {
            RCLCPP_INFO(logger_, "Successfully selected mode: %s", mode_name.c_str());
            return true;
        }
 
        RCLCPP_ERROR(logger_, "SelectMode failed with error code: %d", ret);
        return false;
    }

    // ========== High-level capabilities ==========
    bool UnitreeSDKWrapper::send_velocity_command(
        const float vx,
        const float vy,
        const float vyaw
    ) {
        if (!initialized_ || !loco_client_) {
            RCLCPP_ERROR(logger_, "UnitreeSDKWrapper not initialized");
            return false;
        }

        // With continuous_move = false, the velocity command only takes effect for 1 second
        const std::int32_t ret = loco_client_->Move(vx, vy, vyaw, false);

        if (ret == 0) {
            return true;
        }

        RCLCPP_WARN(logger_, "Move command failed with error code: %d", ret);
        return false;
    }

    bool UnitreeSDKWrapper::damp_high() {
        if (!initialized_ || !loco_client_) {
            RCLCPP_ERROR(logger_, "UnitreeSDKWrapper not initialized");
            return false;
        }

        RCLCPP_INFO(logger_, "Entering high-level damp mode");
        const std::int32_t ret = loco_client_->Damp();

        if (ret == 0) {
            RCLCPP_INFO(logger_, "High-level damp mode activated");
            return true;
        }

        RCLCPP_WARN(logger_, "Damp failed with error code: %d", ret);
        return false;
    }

    void UnitreeSDKWrapper::damp_low() {
        if (!initialized_ || !low_cmd_pub_) {
            RCLCPP_ERROR(logger_, "UnitreeSDKWrapper not initialized");
            return;
        }

        std::array<bool, embodiment::num_joints> active{};
        const std::array<float, embodiment::num_joints> zeros{};

        for (const auto& joint : embodiment::all_joints) {
            active[static_cast<std::uint8_t>(joint)] = true;
        }

        auto command = construct_low_cmd(
            active,
            zeros,
            zeros,
            zeros,
            Damp::kp,
            Damp::kd
        );

        low_cmd_pub_->Write(command);

        RCLCPP_INFO(logger_, "Low-level damp command sent");
    }

    bool UnitreeSDKWrapper::stand_up() {
        if (!initialized_ || !loco_client_) {
            RCLCPP_ERROR(logger_, "UnitreeSDKWrapper not initialized");
            return false;
        }

        const std::int32_t ret = loco_client_->StandUp();

        if (ret == 0) {
            RCLCPP_INFO(logger_, "Stand command succeeded");
            return true;
        }

        RCLCPP_WARN(logger_, "Stand command failed with error code: %d", ret);
        return false;
    }

    bool UnitreeSDKWrapper::set_balance_mode(const std::uint8_t balance_mode) {
        if (!initialized_ || !loco_client_) {
            RCLCPP_ERROR(logger_, "UnitreeSDKWrapper not initialized");
            return false;
        }

        const std::int32_t ret = loco_client_->SetBalanceMode(static_cast<int>(balance_mode));

        if (ret == 0) {
            RCLCPP_INFO(logger_, "Set balance mode to %d", balance_mode);
            return true;
        }

        RCLCPP_WARN(logger_, "Failed to set balance mode with error code: %d", ret);
        return false;
    }

    bool UnitreeSDKWrapper::start() {
        if (!initialized_ || !loco_client_) {
            RCLCPP_ERROR(logger_, "UnitreeSDKWrapper not initialized");
            return false;
        }

        const std::int32_t ret = loco_client_->SetFsmId(801);

        if (ret == 0) {
            RCLCPP_INFO(logger_, "Start command succeeded");
            return true;
        }

        RCLCPP_WARN(logger_, "Start command failed with error code: %d", ret);
        return false;
    }

    void UnitreeSDKWrapper::release_arms(const int steps, const int interval_ms) {
        if (!initialized_ || !arm_sdk_pub_) {
            RCLCPP_ERROR(logger_, "UnitreeSDKWrapper not initialized");
            return;
        }

        RCLCPP_INFO(logger_, "Releasing arm SDK control (%d steps, %dms interval)", steps, interval_ms);

        const float weight_per_step = 1.0F / static_cast<float>(steps);
        const auto interval = std::chrono::milliseconds(interval_ms);

        std::array<bool, embodiment::num_joints> active{};
        std::array<float, embodiment::num_joints> hold_position{};
        const std::array<float, embodiment::num_joints> zeros{};

        {
            std::lock_guard lock(position_mutex_);
            for (const auto& joint : embodiment::upper_body) {
                const auto index = static_cast<std::uint8_t>(joint);
                active[index] = true;
                hold_position[index] = actual_position_[index];
            }
        }

        for (int i = 0; i <= steps; ++i) {
            const float weight = 1.0F - (static_cast<float>(i) * weight_per_step);
            auto command = construct_low_cmd(
                active,
                hold_position,
                zeros,
                zeros,
                Default::kp,
                Default::kd,
                weight
            );

            arm_sdk_pub_->Write(command);

            std::this_thread::sleep_for(interval);
        }

        RCLCPP_INFO(logger_, "Arm SDK control released");
    }

    // ========== Low-level capabilities ==========
    LowCmd UnitreeSDKWrapper::construct_low_cmd(
        const std::array<bool, embodiment::num_joints>& active,
        const std::array<float, embodiment::num_joints>& position,
        const std::array<float, embodiment::num_joints>& velocity,
        const std::array<float, embodiment::num_joints>& effort,
        const std::array<float, embodiment::num_joints>& kp,
        const std::array<float, embodiment::num_joints>& kd,
        const float weight
    ) {
        LowCmd command{};

        command.mode_pr() = mode_pr_;
        command.mode_machine() = mode_machine_;

        command.motor_cmd().at(static_cast<std::uint8_t>(embodiment::JointIndex::WeightParameter)).q() = weight;

        for (std::size_t i = 0; i < embodiment::num_joints; ++i) {
            if (!active[i]) { continue; }

            command.motor_cmd().at(i).mode() = 1;
            command.motor_cmd().at(i).q()    = position[i];
            command.motor_cmd().at(i).dq()   = velocity[i];
            command.motor_cmd().at(i).tau()   = std::clamp(effort[i], -embodiment::effort_limit[i], embodiment::effort_limit[i]);
            command.motor_cmd().at(i).kp()   = kp[i];
            command.motor_cmd().at(i).kd()   = kd[i];
        }

        static_assert(sizeof(LowCmd) % 4 == 0);

        // Calculate CRC
        command.crc() = 0; // Zero it out first
        command.crc() = crc_32_core(
            reinterpret_cast<const std::uint32_t*>(&command),
            (sizeof(LowCmd) / sizeof(std::uint32_t)) - 1
        );

        return command;
    }

    void UnitreeSDKWrapper::send_arm_commands(
        const std::array<bool, embodiment::num_joints>& active,
        const std::array<float, embodiment::num_joints>& position,
        const std::array<float, embodiment::num_joints>& velocity,
        const std::array<float, embodiment::num_joints>& effort,
        const std::array<float, embodiment::num_joints>& kp,
        const std::array<float, embodiment::num_joints>& kd,
        const std::array<float, embodiment::num_joints>& ki
    ) {
        if (!initialized_ || !arm_sdk_pub_) {
            RCLCPP_ERROR(logger_, "UnitreeSDKWrapper not initialized");
            return;
        }

        std::array<float, embodiment::num_joints> actual_pos;
        {
            std::lock_guard lock(position_mutex_);
            actual_pos = actual_position_;
        }

        std::array<float, embodiment::num_joints> adjusted_effort = effort;

        {
            std::lock_guard lock(integral_mutex_);
            for (std::size_t i = 0; i < embodiment::num_joints; ++i) {
                if (!active[i]) { continue; }

                const auto error = position[i] - actual_pos[i];
                integral_error_[i] += error;

                adjusted_effort[i] += ki[i] * integral_error_[i];
            }
        }

        auto command = construct_low_cmd(
            active,
            position,
            velocity,
            adjusted_effort,
            kp,
            kd,
            1.0F
        );

        arm_sdk_pub_->Write(command);
    }

    void UnitreeSDKWrapper::send_low_commands(
        const std::array<bool, embodiment::num_joints>& active,
        const std::array<float, embodiment::num_joints>& position,
        const std::array<float, embodiment::num_joints>& velocity,
        const std::array<float, embodiment::num_joints>& effort,
        const std::array<float, embodiment::num_joints>& kp,
        const std::array<float, embodiment::num_joints>& kd,
        const std::array<float, embodiment::num_joints>& ki
    ) {
        if (!initialized_ || !low_cmd_pub_) {
            RCLCPP_ERROR(logger_, "UnitreeSDKWrapper not initialized");
            return;
        }

        std::array<float, embodiment::num_joints> actual_pos;
        {
            std::lock_guard lock(position_mutex_);
            actual_pos = actual_position_;
        }

        std::array<float, embodiment::num_joints> adjusted_effort = effort;

        {
            std::lock_guard lock(integral_mutex_);
            for (std::size_t i = 0; i < embodiment::num_joints; ++i) {
                if (!active[i]) { continue; }

                const auto error = position[i] - actual_pos[i];
                integral_error_[i] += error;

                adjusted_effort[i] += ki[i] * integral_error_[i];
            }
        }

        auto command = construct_low_cmd(
            active,
            position,
            velocity,
            adjusted_effort,
            kp,
            kd
        );

        low_cmd_pub_->Write(command);
    }

    // ========== Audio capabilities ==========
    bool UnitreeSDKWrapper::set_volume(const std::uint8_t volume) {
        if (!initialized_ || !audio_client_) {
            RCLCPP_ERROR(logger_, "UnitreeSDKWrapper not initialized");
            return false;
        }

        const int32_t ret = audio_client_->SetVolume(volume);

        if (ret == 0) {
            RCLCPP_INFO(logger_, "Set speaker volume to %d", volume);
            return true;
        }

        RCLCPP_WARN(logger_, "Failed to set volume with error code: %d", ret);
        return false;
    }

    bool UnitreeSDKWrapper::send_speech_command(const std::string& message) {
        if (!initialized_ || !audio_client_) {
            RCLCPP_ERROR(logger_, "UnitreeSDKWrapper not initialized");
            return false;
        }

        if (message.empty()) {
            RCLCPP_WARN(logger_, "Cannot send empty TTS message");
            return false;
        }

        const std::int32_t ret = audio_client_->TtsMaker(message, 1);

        if (ret == 0) {
            RCLCPP_INFO(logger_, "TTS command sent successfully");
            return true;
        }

        RCLCPP_WARN(logger_, "TTS command failed with error code: %d", ret);
        return false;
    }

    // ========== Hand capabilities ==========
    void UnitreeSDKWrapper::send_hand_command(
        const hands::Side side,
        const std::array<float, hands::num_joints>& positions
    ) {
        auto& pub = (side == hands::Side::Left) ? left_hand_cmd_pub_ : right_hand_cmd_pub_;

        if (!initialized_ || !pub) {
            RCLCPP_ERROR(logger_, "UnitreeSDKWrapper not initialized");
            return;
        }

        HandCmd cmd{};
        cmd.motor_cmd().resize(hands::num_joints);

        for (std::size_t i = 0; i < hands::num_joints; ++i) {
            cmd.motor_cmd()[i].mode(hands::encode_motor_mode(static_cast<std::uint8_t>(i)));
            cmd.motor_cmd()[i].q(positions[i]);
            cmd.motor_cmd()[i].dq(0.0F);
            cmd.motor_cmd()[i].tau(0.0F);
            cmd.motor_cmd()[i].kp(hands::kp[i]);
            cmd.motor_cmd()[i].kd(hands::kd[i]);
        }

        pub->Write(cmd);
    }

    std::array<float, hands::num_joints> UnitreeSDKWrapper::get_hand_position(const hands::Side side) {
        std::lock_guard lock(hand_position_mutex_);
        return (side == hands::Side::Left) ? left_hand_position_ : right_hand_position_;
    }

    void UnitreeSDKWrapper::set_hand_states_publishers(
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_pub,
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_pub
    ) {
        left_hand_states_pub_ = std::move(left_pub);
        right_hand_states_pub_ = std::move(right_pub);
    }

    // ========== Joint state feedback ==========
    void UnitreeSDKWrapper::set_joint_states_publisher(
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher
    ) {
        joint_states_pub_ = std::move(publisher);
    }

    void UnitreeSDKWrapper::reset_integral_error() {
        {
            std::lock_guard lock(integral_mutex_);
            integral_error_.fill(0.0F);
        }
        RCLCPP_INFO(logger_, "Integral error reset");
    }

    // ========== Callbacks ==========
    void UnitreeSDKWrapper::low_state_callback(const void* message) {
        const auto& state = *static_cast<const LowState*>(message);

        if (mode_machine_ != state.mode_machine()) {
            mode_machine_ = state.mode_machine();
        }

        {
            std::lock_guard lock(position_mutex_);
            for (std::size_t i = 0; i < embodiment::num_joints; ++i) {
                actual_position_[i] = state.motor_state()[i].q();
            }
        }

        if (joint_states_pub_) {
            sensor_msgs::msg::JointState joint_state;
            joint_state.header.stamp = rclcpp::Clock{}.now();
            joint_state.name.resize(embodiment::num_joints);
            joint_state.position.resize(embodiment::num_joints);
            joint_state.velocity.resize(embodiment::num_joints);
            joint_state.effort.resize(embodiment::num_joints);

            for (std::size_t i = 0; i < embodiment::num_joints; ++i) {
                joint_state.name[i] = embodiment::joint_names[i];
                joint_state.position[i] = state.motor_state()[i].q();
                joint_state.velocity[i] = state.motor_state()[i].dq();
                joint_state.effort[i] = state.motor_state()[i].tau_est();
            }

            joint_states_pub_->publish(joint_state);
        }
    }

    void UnitreeSDKWrapper::left_hand_state_callback(const void* message) {
        const auto& state = *static_cast<const HandState*>(message);

        {
            std::lock_guard lock(hand_position_mutex_);
            for (std::size_t i = 0; i < hands::num_joints; ++i) {
                left_hand_position_[i] = state.motor_state()[i].q();
            }
        }

        if (left_hand_states_pub_) {
            sensor_msgs::msg::JointState joint_state;
            joint_state.header.stamp = rclcpp::Clock{}.now();
            joint_state.position.resize(hands::num_joints);

            for (std::size_t i = 0; i < hands::num_joints; ++i) {
                joint_state.position[i] = state.motor_state()[i].q();
            }

            left_hand_states_pub_->publish(joint_state);
        }
    }

    void UnitreeSDKWrapper::right_hand_state_callback(const void* message) {
        const auto& state = *static_cast<const HandState*>(message);

        {
            std::lock_guard lock(hand_position_mutex_);
            for (std::size_t i = 0; i < hands::num_joints; ++i) {
                right_hand_position_[i] = state.motor_state()[i].q();
            }
        }

        if (right_hand_states_pub_) {
            sensor_msgs::msg::JointState joint_state;
            joint_state.header.stamp = rclcpp::Clock{}.now();
            joint_state.position.resize(hands::num_joints);

            for (std::size_t i = 0; i < hands::num_joints; ++i) {
                joint_state.position[i] = state.motor_state()[i].q();
            }

            right_hand_states_pub_->publish(joint_state);
        }
    }

} // namespace unitree_interface
