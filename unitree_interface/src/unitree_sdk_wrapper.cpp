#include "unitree_interface/unitree_sdk_wrapper.hpp"

#include "unitree_interface_msgs/msg/joint_commands.hpp"

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

    // TODO: Unit test to check if this behaves the same as Unitree's implementation
    static std::uint32_t crc_32_core(const std::uint32_t* ptr, std::uint32_t len) noexcept {
        std::uint32_t xbit = 0;
        std::uint32_t data = 0;
        std::uint32_t crc32 = 0xFFFFFFFF; // NOLINT

        const std::uint32_t dw_polynomial = 0x04c11db7;
        for (std::uint32_t i = 0; i < len; i++) {
            xbit = 1 << 31; // NOLINT
            data = ptr[i];

            for (uint32_t bits = 0; bits < 32; bits++) { // NOLINT
                if (crc32 & 0x80000000) { // NOLINT
                    crc32 <<= 1;
                    crc32 ^= dw_polynomial;
                } else {
                    crc32 <<= 1;
                }

                if ((data & xbit) != 0) {
                    crc32 ^= dw_polynomial;
                }
      
                xbit >>= 1;
            }
        }
        return crc32;
    }

    UnitreeSDKWrapper::UnitreeSDKWrapper(
        std::string network_interface,
        rclcpp::Logger logger
    ) : network_interface_(std::move(network_interface)),
        logger_(std::move(logger)) {
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
        unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);
        RCLCPP_INFO(logger_, "ChannelFactory initialized with interface: %s", network_interface_.c_str());

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

    LowState UnitreeSDKWrapper::get_low_state() {
        std::lock_guard<std::mutex> lock(low_state_mutex_);

        return low_state_;
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

        const std::int32_t ret = loco_client_->Start();

        if (ret == 0) {
            RCLCPP_INFO(logger_, "Start command succeeded");
            return true;
        }

        RCLCPP_WARN(logger_, "Start command failed with error code: %d", ret);
        return false;
    }

    // ========== Low-level capabilities ==========
    void UnitreeSDKWrapper::send_joint_commands(const unitree_interface_msgs::msg::JointCommands& message) {
        if (!initialized_ || !low_cmd_pub_) {
            RCLCPP_ERROR(logger_, "UnitreeSDKWrapper not initialized");
            return;
        }

        LowCmd command{};

        command.mode_pr() = mode_pr_;
        command.mode_machine() = mode_machine_;

        // Fill in the commands
        for (const auto& command_spec : message.commands) {
            const auto joint_index = static_cast<size_t>(command_spec.joint_index);

            command.motor_cmd().at(joint_index).mode() = command_spec.mode;
            command.motor_cmd().at(joint_index).q()    = command_spec.q;
            command.motor_cmd().at(joint_index).dq()   = command_spec.dq;
            command.motor_cmd().at(joint_index).tau()  = command_spec.tau;
            command.motor_cmd().at(joint_index).kp()   = command_spec.kp;
            command.motor_cmd().at(joint_index).kd()   = command_spec.kd;
        }

        static_assert(sizeof(LowCmd) % 4 == 0);

        // Calculate CRC
        command.crc() = 0; // Zero it out first
        command.crc() = crc_32_core(
            reinterpret_cast<const std::uint32_t*>(&command),
            (sizeof(LowCmd) / sizeof(std::uint32_t)) - 1
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

    // ========== Callbacks ==========
    void UnitreeSDKWrapper::low_state_callback(const void* message) {
        std::lock_guard<std::mutex> lock(low_state_mutex_);

        low_state_ = *static_cast<const LowState*>(message);
    }

} // namespace unitree_interface
