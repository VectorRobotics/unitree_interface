#include "unitree_interface/unitree_sdk_wrapper.hpp"
#include "unitree_interface/topology.hpp"
#include "unitree_interface/crc.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp> // Seems misleading, but is actually correct
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

#include <rclcpp/logging.hpp>

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

    bool UnitreeSDKWrapper::damp() {
        if (!initialized_ || !loco_client_) {
            RCLCPP_ERROR(logger_, "UnitreeSDKWrapper not initialized");
            return false;
        }

        RCLCPP_INFO(logger_, "Entering damp mode");
        const std::int32_t ret = loco_client_->Damp();

        if (ret == 0) {
            RCLCPP_INFO(logger_, "Damp mode activated");
            return true;
        }

        RCLCPP_WARN(logger_, "Damp failed with error code: %d", ret);
        return false;
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

        const std::int32_t ret = loco_client_->SetFsmId(501);

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

        const std::vector<std::uint8_t> empty_indices{};
        const std::vector<float> empty{};

        for (int i = 0; i <= steps; ++i) {
            const float weight = 1.0F - (static_cast<float>(i) * weight_per_step);
            auto command = construct_low_cmd(
                empty_indices,
                empty,
                empty,
                empty,
                empty,
                empty,
                weight
            );

            arm_sdk_pub_->Write(command);

            std::this_thread::sleep_for(interval);
        }

        RCLCPP_INFO(logger_, "Arm SDK control released");
    }

    // ========== Low-level capabilities ==========
    LowCmd UnitreeSDKWrapper::construct_low_cmd(
        const std::vector<std::uint8_t>& indices,
        const std::vector<float>& position,
        const std::vector<float>& velocity,
        const std::vector<float>& effort,
        const std::vector<float>& kp,
        const std::vector<float>& kd,
        const float weight
    ) {
        LowCmd command{};

        command.mode_pr() = mode_pr_;
        command.mode_machine() = mode_machine_;

        command.motor_cmd().at(static_cast<std::uint8_t>(joints::JointIndex::WeightParameter)).q() = weight;

        for (std::size_t i = 0; i < indices.size(); ++i) {
            const auto joint_index = indices[i];

            command.motor_cmd().at(joint_index).mode() = 1;
            command.motor_cmd().at(joint_index).q()    = position[i];
            command.motor_cmd().at(joint_index).dq()   = velocity[i];
            command.motor_cmd().at(joint_index).tau()   = effort[i];
            command.motor_cmd().at(joint_index).kp()   = kp[i];
            command.motor_cmd().at(joint_index).kd()   = kd[i];
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
        const std::vector<std::uint8_t>& indices,
        const std::vector<float>& position,
        const std::vector<float>& velocity,
        const std::vector<float>& effort,
        const std::vector<float>& kp,
        const std::vector<float>& kd
    ) {
        if (!initialized_ || !arm_sdk_pub_) {
            RCLCPP_ERROR(logger_, "UnitreeSDKWrapper not initialized");
            return;
        }

        auto command = construct_low_cmd(
            indices,
            position,
            velocity,
            effort,
            kp,
            kd,
            1.0F
        );

        arm_sdk_pub_->Write(command);
    }

    void UnitreeSDKWrapper::send_low_commands(
        const std::vector<std::uint8_t>& indices,
        const std::vector<float>& position,
        const std::vector<float>& velocity,
        const std::vector<float>& effort,
        const std::vector<float>& kp,
        const std::vector<float>& kd
    ) {
        if (!initialized_ || !low_cmd_pub_) {
            RCLCPP_ERROR(logger_, "UnitreeSDKWrapper not initialized");
            return;
        }

        auto command = construct_low_cmd(
            indices,
            position,
            velocity,
            effort,
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

    // ========== Joint state feedback ==========
    void UnitreeSDKWrapper::set_joint_states_publisher(
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher
    ) {
        joint_states_pub_ = std::move(publisher);
    }

    // ========== Callbacks ==========
    void UnitreeSDKWrapper::low_state_callback(const void* message) {
        const auto& state = *static_cast<const LowState*>(message);

        if (mode_machine_ != state.mode_machine()) {
            mode_machine_ = state.mode_machine();
        }

        if (joint_states_pub_) {
            sensor_msgs::msg::JointState joint_state;
            joint_state.name.resize(joints::num_joints);
            joint_state.position.resize(joints::num_joints);
            joint_state.velocity.resize(joints::num_joints);
            joint_state.effort.resize(joints::num_joints);

            for (std::size_t i = 0; i < joints::num_joints; ++i) {
                joint_state.name[i] = joints::joint_names[i];
                joint_state.position[i] = state.motor_state()[i].q();
                joint_state.velocity[i] = state.motor_state()[i].dq();
                joint_state.effort[i] = state.motor_state()[i].tau_est();
            }

            joint_states_pub_->publish(joint_state);
        }
    }

} // namespace unitree_interface
