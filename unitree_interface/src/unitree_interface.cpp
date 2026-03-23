#include "unitree_interface/unitree_interface.hpp"

#include "unitree_interface/control_modes.hpp"
#include "unitree_interface/mode_transitions.hpp"
#include "unitree_interface/profiles.hpp"
#include "unitree_interface/unitree_sdk_wrapper.hpp"
#include "unitree_interface/topology.hpp"
#include "unitree_interface_msgs/msg/profile.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <thread>

namespace unitree_interface {

    template <typename ProfileType>
    void UnitreeInterface::declare_profile_gains(const bool dynamic) {
        using dynamic_params::FloatRange;

        const std::string prefix = std::string(ProfileTraits<ProfileType>::name()) + "/";

        for (std::size_t i = 0; i < joints::num_joints; ++i) {
            params_.declare(
                prefix + "kp/" + joints::joint_names[i],
                static_cast<double>(ProfileType::kp[i]),
                FloatRange{0.0, 200.0},
                "",
                dynamic,
                [this, i](const rclcpp::Parameter& p) {
                    if (ProfileTraits<ProfileType>::id == get_active_profile_id()) {
                        current_kp_[i] = static_cast<float>(p.as_double());
                    }
                }
            );

            params_.declare(
                prefix + "kd/" + joints::joint_names[i],
                static_cast<double>(ProfileType::kd[i]),
                FloatRange{0.0, 50.0},
                "",
                dynamic,
                [this, i](const rclcpp::Parameter& p) {
                    if (ProfileTraits<ProfileType>::id == get_active_profile_id()) {
                        current_kd_[i] = static_cast<float>(p.as_double());
                    }
                }
            );

            params_.declare(
                prefix + "ki/" + joints::joint_names[i],
                static_cast<double>(ProfileType::ki[i]),
                FloatRange{0.0, 12.0},
                "",
                dynamic,
                [this, i](const rclcpp::Parameter& p) {
                    if (ProfileTraits<ProfileType>::id == get_active_profile_id()) {
                        current_ki_[i] = static_cast<float>(p.as_double());
                    }
                }
            );
        }
    }

    UnitreeInterface::UnitreeInterface(const rclcpp::NodeOptions& options)
        : Node("unitree_interface", options),
          logger_(get_logger()),
          params_(this),
          current_mode_(std::monostate{}),
          current_profile_(Default{}) {
        using dynamic_params::FloatRange;
        using dynamic_params::IntRange;

        // ========== Static parameters ==========
        params_.declare("mode_change_service_name", "~/change_mode", "", false);
        params_.declare("ready_locomotion_service_name", "~/ready_locomotion", "", false);
        params_.declare("release_arms_service_name", "~/release_arms", "", false);
        params_.declare("reset_integral_error_service_name", "~/reset_integral_error", "", false);
        params_.declare("set_profile_service_name", "~/set_profile", "", false);

        params_.declare("current_mode_topic", "~/current_mode", "", false);
        params_.declare("current_profile_topic", "~/current_profile", "", false);
        params_.declare("cmd_vel_topic", "~/cmd_vel", "", false);
        params_.declare("cmd_arm_topic", "~/cmd_arm", "", false);
        params_.declare("joint_states_topic", "~/joint_states", "", false);
        params_.declare("tts_topic", "~/tts", "", false);
#ifdef UNITREE_INTERFACE_ENABLE_LOW_LEVEL_MODE
        params_.declare("cmd_low_topic", "~/cmd_low", "", false);
#endif
        params_.declare("estop_topic", "/estop", "", false);

        params_.declare("motion_switcher_client_timeout", 5.0, FloatRange{1.0, 30.0, 0.5}, "", false);
        params_.declare("loco_client_timeout", 10.0, FloatRange{1.0, 30.0, 0.5}, "", false);
        params_.declare("audio_client_timeout", 5.0, FloatRange{1.0, 30.0, 0.5}, "", false);

        // ========== Dynamic parameters ==========
        params_.declare("volume", 100, IntRange{0, 100});
        params_.declare("ready_locomotion_stand_up_delay", 5, IntRange{0, 30});
        params_.declare("ready_locomotion_start_delay", 10, IntRange{0, 30});
        params_.declare("release_arms_steps", 250, IntRange{1, 1000});
        params_.declare("release_arms_interval_ms", 20, IntRange{1, 100});
        params_.declare("controller_time_step_ms", 20);

        // ========== Gain parameters (per-profile) ==========
        declare_profile_gains<Default>();
        declare_profile_gains<Damp>();
        declare_profile_gains<VisualServo>();
        declare_profile_gains<WholeBodyControl>();
        declare_profile_gains<EffortOnly>(false);

        // Load initial gains from the default profile
        apply_profile_gains(current_profile_);
    }

    // ========== Initialization ==========
    void UnitreeInterface::initialize() {
        if (sdk_wrapper_) {
            RCLCPP_INFO(logger_, "UnitreeInterface already intialized");
            return;
        }

        const auto msc_timeout = static_cast<float>(params_.get_double("motion_switcher_client_timeout"));
        const auto loco_client_timeout = static_cast<float>(params_.get_double("loco_client_timeout"));
        const auto audio_client_timeout = static_cast<float>(params_.get_double("audio_client_timeout"));

        sdk_wrapper_ = std::make_unique<UnitreeSDKWrapper>(logger_);

        if (!sdk_wrapper_->initialize(msc_timeout, loco_client_timeout, audio_client_timeout)) {
            RCLCPP_ERROR(logger_, "Failed to initialize SDK wrapper");
            throw std::runtime_error("Unitree SDK initialization failed");
        }

        sdk_wrapper_->set_volume(static_cast<std::uint8_t>(params_.get_int("volume")));

        current_mode_ = Transition<std::monostate, IdleMode>::execute(*sdk_wrapper_);
        RCLCPP_INFO(
            logger_,
            "Initialized to control mode: %s",
            ControlModeTraits<IdleMode>::name()
        );

        RCLCPP_INFO(
            logger_,
            "Initialized to profile: %s",
            ProfileTraits<Default>::name()
        );

        initialize_services();
        initialize_publishers();
        sdk_wrapper_->set_joint_states_publisher(joint_states_pub_);
        create_subscriptions();

        setup_mode_dependent_subscriptions();
        setup_mode_dependent_timers();
        publish_current_mode();
        publish_current_profile();
    }

    void UnitreeInterface::initialize_services() {
        mode_change_service_ = create_service<unitree_interface_msgs::srv::ChangeControlMode>(
            params_.get_string("mode_change_service_name"),
            [this](
                const unitree_interface_msgs::srv::ChangeControlMode::Request::SharedPtr request, // NOLINT
                unitree_interface_msgs::srv::ChangeControlMode::Response::SharedPtr response
            ) {
                handle_mode_change_request(request, response); // NOLINT
            }
        );

        ready_locomotion_service_ = create_service<std_srvs::srv::Trigger>(
            params_.get_string("ready_locomotion_service_name"),
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr request, // NOLINT
                std_srvs::srv::Trigger::Response::SharedPtr response
            ) {
                handle_ready_locomotion_request(request, response); // NOLINT
            }
        );

        release_arms_service_ = create_service<std_srvs::srv::Trigger>(
            params_.get_string("release_arms_service_name"),
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr request, // NOLINT
                std_srvs::srv::Trigger::Response::SharedPtr response
            ) {
                handle_release_arms_request(request, response); // NOLINT
            }
        );

        reset_integral_error_service_ = create_service<std_srvs::srv::Trigger>(
            params_.get_string("reset_integral_error_service_name"),
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr request, // NOLINT
                std_srvs::srv::Trigger::Response::SharedPtr response
            ) {
                handle_reset_integral_error_request(request, response); // NOLINT
            }
        );

        set_profile_service_ = create_service<unitree_interface_msgs::srv::SetProfile>(
            params_.get_string("set_profile_service_name"),
            [this](
                const unitree_interface_msgs::srv::SetProfile::Request::SharedPtr request, // NOLINT
                unitree_interface_msgs::srv::SetProfile::Response::SharedPtr response
            ) {
                handle_set_profile_request(request, response); // NOLINT
            }
        );

        RCLCPP_INFO(logger_, "Services created");
    }

    void UnitreeInterface::initialize_publishers() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                        .transient_local()
                        .reliable();

        current_mode_pub_ = create_publisher<unitree_interface_msgs::msg::ControlMode>(
            params_.get_string("current_mode_topic"),
            qos
        );

        current_profile_pub_ = create_publisher<unitree_interface_msgs::msg::Profile>(
            params_.get_string("current_profile_topic"),
            qos
        );

        joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            params_.get_string("joint_states_topic"),
            rclcpp::QoS(10) // NOLINT
        );

        // error_pub_ = create_publisher<std_msgs::msg::Float64>(
        //     params_.get_string("error_topic"),
        //     rclcpp::QoS(10) // NOLINT
        // );

        // int_error_pub_ = create_publisher<std_msgs::msg::Float64>(
        //     params_.get_string("int_error_topic"),
        //     rclcpp::QoS(10) // NOLINT
        // );

        RCLCPP_INFO(logger_, "Publishers created");
    }

    void UnitreeInterface::create_subscriptions() {
        estop_sub_ = create_subscription<std_msgs::msg::Empty>(
            params_.get_string("estop_topic"),
            rclcpp::QoS(1),
            [this](const std_msgs::msg::Empty::SharedPtr) { // NOLINT
                estop_callback();
            }
        );

        tts_sub_ = create_subscription<std_msgs::msg::String>(
            params_.get_string("tts_topic"),
            rclcpp::QoS(10), // NOLINT
            [this](const std_msgs::msg::String::SharedPtr message) { // NOLINT
                tts_callback(message);
            }
        );
    }

    void UnitreeInterface::setup_mode_dependent_subscriptions() {
        // Reset all subscriptions
        // NOTE: Do not reset estop_sub_ if you value your life
        cmd_vel_sub_.reset();
        cmd_arm_sub_.reset();
#ifdef UNITREE_INTERFACE_ENABLE_LOW_LEVEL_MODE
        cmd_low_sub_.reset();
#endif

        // Create subscriptions based on current mode
        std::visit(
            [this](auto&& mode){
                using ModeType = std::decay_t<decltype(mode)>;

                if constexpr (
                    std::is_same_v<ModeType, std::monostate> ||
                    std::is_same_v<ModeType, IdleMode> ||
                    std::is_same_v<ModeType, EmergencyMode>
                ) {
                    RCLCPP_INFO(
                        logger_,
                        "%s: No subscriptions created",
                        ControlModeTraits<ModeType>::name()
                    );
                } else if constexpr (std::is_same_v<ModeType, HighLevelMode>) {
                    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
                        params_.get_string("cmd_vel_topic"),
                        rclcpp::QoS(10), // NOLINT
                        [this](const geometry_msgs::msg::TwistStamped::SharedPtr message) { // NOLINT
                            cmd_vel_callback(message);
                        }
                    );

                    cmd_arm_sub_ = create_subscription<sensor_msgs::msg::JointState>(
                        params_.get_string("cmd_arm_topic"),
                        rclcpp::QoS(10), // NOLINT
                        [this](const sensor_msgs::msg::JointState::SharedPtr message) { // NOLINT
                            cmd_arm_callback(message);
                        }
                    );

                    RCLCPP_INFO(
                        logger_,
                        "%s: subscriptions created",
                        ControlModeTraits<HighLevelMode>::name()
                    );
#ifdef UNITREE_INTERFACE_ENABLE_LOW_LEVEL_MODE
                } else if constexpr (std::is_same_v<ModeType, LowLevelMode>) {
                    cmd_low_sub_ = create_subscription<sensor_msgs::msg::JointState>(
                        params_.get_string("cmd_low_topic"),
                        rclcpp::QoS(10), // NOLINT
                        [this](const sensor_msgs::msg::JointState::SharedPtr message) { // NOLINT
                            cmd_low_callback(message);
                        }
                    );
#endif
                } else {
                    static_assert(always_false<ModeType>::value, "Illegal mode");
                }
            },
            current_mode_
        );
    }

    void UnitreeInterface::setup_mode_dependent_timers() {

        // Create timer based on current mode
        std::visit(
            [this](auto&& mode){
                using ModeType = std::decay_t<decltype(mode)>;

                if constexpr (
                    std::is_same_v<ModeType, std::monostate> ||
                    std::is_same_v<ModeType, IdleMode> ||
                    std::is_same_v<ModeType, EmergencyMode> ||
                    std::is_same_v<ModeType, LowLevelMode>
                ) {
                    RCLCPP_INFO(
                        logger_,
                        "%s: No timer created",
                        ControlModeTraits<ModeType>::name()
                    );
                } else if constexpr (std::is_same_v<ModeType, HighLevelMode>) {
                    using namespace std::chrono_literals;
                    control_loop_ = this->create_wall_timer(
                        std::chrono::milliseconds(params_.get_int("controller_time_step_ms")), 
                        [this](){ controller(); }
                    );

                    RCLCPP_INFO(
                        logger_,
                        "%s: timers created with time step %d ms",
                        ControlModeTraits<HighLevelMode>::name(),
                        std::chrono::milliseconds(params_.get_int("controller_time_step_ms"))
                    );
                } else {
                    static_assert(always_false<ModeType>::value, "Illegal mode");
                }
            },
            current_mode_
        );
    }

    std::uint8_t UnitreeInterface::get_active_profile_id() const {
        return std::visit(
            [](const auto& p) -> std::uint8_t {
                using ProfileType = std::decay_t<decltype(p)>;
                return ProfileTraits<ProfileType>::id;
            },
            current_profile_
        );
    }

    void UnitreeInterface::apply_profile_gains(const Profile& profile) {
        const auto profile_name = std::visit(
            [](const auto& p) -> const char* {
                using ProfileType = std::decay_t<decltype(p)>;
                return ProfileTraits<ProfileType>::name();
            },
            profile
        );

        const std::string prefix = std::string(profile_name) + "/";

        for (std::size_t i = 0; i < joints::num_joints; ++i) {
            current_kp_[i] = static_cast<float>(
                params_.get_double(prefix + "kp/" + joints::joint_names[i])
            );
            current_kd_[i] = static_cast<float>(
                params_.get_double(prefix + "kd/" + joints::joint_names[i])
            );
            current_ki_[i] = static_cast<float>(
                params_.get_double(prefix + "ki/" + joints::joint_names[i])
            );
        }
    }

    // ========== Callbacks ==========
    void UnitreeInterface::handle_mode_change_request(
        const unitree_interface_msgs::srv::ChangeControlMode::Request::SharedPtr request, // NOLINT
        unitree_interface_msgs::srv::ChangeControlMode::Response::SharedPtr response // NOLINT
    ) {
        const std::uint8_t requested_mode = request->requested_mode;

        RCLCPP_INFO(logger_, "Mode change request: %d", requested_mode);

        auto [new_mode, success] = try_transition_to(
            current_mode_,
            requested_mode,
            *sdk_wrapper_
        );

        current_mode_ = new_mode;

        response->success = success;

        if (success) {
            RCLCPP_INFO(logger_, "Mode transition successful");

            sdk_wrapper_->reset_integral_error();
            setup_mode_dependent_subscriptions();
            setup_mode_dependent_timers();
            publish_current_mode();

            response->success = success;
            response->message = "Mode transition successful";
        } else {
            response->message = "Mode transition failed";
            RCLCPP_WARN(logger_, "Mode transition failed");
        }
    }

    void UnitreeInterface::handle_ready_locomotion_request(
        const std_srvs::srv::Trigger::Request::SharedPtr, // NOLINT
        std_srvs::srv::Trigger::Response::SharedPtr response // NOLINT
    ) {
        if (std::holds_alternative<HighLevelMode>(current_mode_)) {
            RCLCPP_INFO(logger_, "Ready locomotion sequence requested");

            const auto stand_up_delay = params_.get_int("ready_locomotion_stand_up_delay");
            const auto start_delay = params_.get_int("ready_locomotion_start_delay");

            if (!sdk_wrapper_->damp_high()) {
                response->success = false;
                response->message = "damp() failed";
                return;
            }

            std::this_thread::sleep_for(std::chrono::seconds(stand_up_delay));

            if (!sdk_wrapper_->stand_up()) {
                response->success = false;
                response->message = "stand_up() failed";
                return;
            }

            std::this_thread::sleep_for(std::chrono::seconds(start_delay));

            if (!sdk_wrapper_->start()) {
                response->success = false;
                response->message = "start() failed";
                return;
            }

            response->success = true;
            response->message = "Locomotion ready";
            RCLCPP_INFO(logger_, "Locomotion ready sequence completed");
        } else {
            response->success = false;
            response->message = "Ready locomotion sequence attempted while not in HighLevelMode";
            RCLCPP_WARN(logger_, "Ready locomotion sequence attempted while not in HighLevelMode");
        }
    }

    void UnitreeInterface::handle_release_arms_request(
        const std_srvs::srv::Trigger::Request::SharedPtr, // NOLINT
        std_srvs::srv::Trigger::Response::SharedPtr response // NOLINT
    ) {
        sdk_wrapper_->reset_integral_error();
        if (std::holds_alternative<HighLevelMode>(current_mode_)) {
            const auto steps = params_.get_int("release_arms_steps");
            const auto interval_ms = params_.get_int("release_arms_interval_ms");

            releasing_arms_ = true;
            sdk_wrapper_->release_arms(steps, interval_ms);
            releasing_arms_ = false;

            response->success = true;
            response->message = "Arms released";
        } else {
            response->success = false;
            response->message = "Not in HighLevelMode";
            RCLCPP_WARN(logger_, "Release arms requested while not in HighLevelMode");
        }
    }

    void UnitreeInterface::handle_set_profile_request(
        const unitree_interface_msgs::srv::SetProfile::Request::SharedPtr request, // NOLINT
        unitree_interface_msgs::srv::SetProfile::Response::SharedPtr response // NOLINT
    ) {
        const std::uint8_t requested = request->requested_profile;

        switch (requested) {
            case unitree_interface_msgs::msg::Profile::PROFILE_DEFAULT:
                current_profile_ = Default{};
                break;
            case unitree_interface_msgs::msg::Profile::PROFILE_DAMP:
                current_profile_ = Damp{};
                break;
            case unitree_interface_msgs::msg::Profile::PROFILE_VISUAL_SERVO:
                current_profile_ = VisualServo{};
                break;
            case unitree_interface_msgs::msg::Profile::PROFILE_WHOLE_BODY_CONTROL:
                current_profile_ = WholeBodyControl{};
                break;
            case unitree_interface_msgs::msg::Profile::PROFILE_EFFORT_ONLY:
                current_profile_ = EffortOnly{};
                break;
            default:
                response->success = false;
                response->message = "Unknown profile ID";
                RCLCPP_WARN(logger_, "Unknown profile ID: %d", requested);
                return;
        }

        sdk_wrapper_->reset_integral_error();
        apply_profile_gains(current_profile_);
        publish_current_profile();

        response->success = true;
        response->message = std::visit(
            [](const auto& p) -> std::string {
                using ProfileType = std::decay_t<decltype(p)>;

                return std::string("Profile set to: ") + ProfileTraits<ProfileType>::name();
            },
            current_profile_
        );

        sdk_wrapper_->send_speech_command(response->message);

        RCLCPP_INFO(logger_, "%s", response->message.c_str());
    }

    void UnitreeInterface::handle_reset_integral_error_request(
        const std_srvs::srv::Trigger::Request::SharedPtr, // NOLINT
        std_srvs::srv::Trigger::Response::SharedPtr response // NOLINT
    ) {
        sdk_wrapper_->reset_integral_error();
        response->success = true;
        response->message = "Integral error reset";
    }

    void UnitreeInterface::cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr message) { // NOLINT
        if (std::holds_alternative<HighLevelMode>(current_mode_)) {
            const auto vx = static_cast<float>(message->twist.linear.x); // m/s
            const auto vy = static_cast<float>(message->twist.linear.y); // m/s
            const auto vyaw = static_cast<float>(message->twist.angular.z); // rad/s

            sdk_wrapper_->send_velocity_command(vx, vy, vyaw);
        } else {
            RCLCPP_WARN_THROTTLE(
                logger_,
                *get_clock(),
                1000,
                "Received cmd_vel but not in HighLevelMode"
            );
        }
    }

    void UnitreeInterface::cmd_arm_callback(const sensor_msgs::msg::JointState::SharedPtr message) { // NOLINT
        if (releasing_arms_) {
            RCLCPP_WARN_THROTTLE(
                logger_,
                *get_clock(),
                1000,
                "Arm commands blocked while releasing arms"
            );
            return;
        }

        start_arm_cmd_ = true;

        if (std::holds_alternative<HighLevelMode>(current_mode_)) {

            message_.header = message->header;
            message_.name = message->name;
            message_.position = message->position;
            message_.velocity = message->velocity;
            message_.effort = message->effort;
            
        } else {
            RCLCPP_WARN_THROTTLE(
                logger_,
                *get_clock(),
                1000,
                "Received cmd_arm but not in HighLevelMode"
            );
        }
    }

    void UnitreeInterface::controller() { // NOLINT
        if (releasing_arms_) {
            RCLCPP_WARN_THROTTLE(
                logger_,
                *get_clock(),
                1000,
                "Arm commands blocked while releasing arms"
            );
            return;
        }

        if (!start_arm_cmd_) {
            RCLCPP_WARN_THROTTLE(
                logger_,
                *get_clock(),
                1000,
                "Arm commands blocked while no setpoint received"
            );
            return;
        }

        if (std::holds_alternative<HighLevelMode>(current_mode_)) {
            auto [indices, position, velocity, effort, kp, kd, ki] =
                joints::resolve_joint_commands(
                    message_.name,
                    message_.position,
                    message_.velocity,
                    message_.effort,
                    current_kp_,
                    current_kd_,
                    current_ki_
                );

            for (std::size_t i = 0; i < indices.size(); ++i) {
                const auto joint_index = static_cast<joints::JointIndex>(indices[i]);

                if (!joints::contains(joints::upper_body, joint_index)) {
                    position[i] = 0.0F;
                    velocity[i] = 0.0F;
                    effort[i] = 0.0F;
                    kp[i] = 0.0F;
                    kd[i] = 0.0F;
                    ki[i] = 0.0F;
                }
            }

            sdk_wrapper_->send_arm_commands(
                indices,
                position,
                velocity,
                effort,
                kp,
                kd,
                ki,
                params_.get_int("controller_time_step_ms")
            );

        } else {
            RCLCPP_WARN_THROTTLE(
                logger_,
                *get_clock(),
                1000,
                "Received cmd_arm but not in HighLevelMode"
            );
        }
    }

#ifdef UNITREE_INTERFACE_ENABLE_LOW_LEVEL_MODE
    void UnitreeInterface::cmd_low_callback(const sensor_msgs::msg::JointState::SharedPtr message) { // NOLINT
        if (std::holds_alternative<LowLevelMode>(current_mode_)) {
            auto [indices, position, velocity, effort, kp, kd, ki] =
                joints::resolve_joint_commands(
                    message->name,
                    message->position,
                    message->velocity,
                    message->effort,
                    current_kp_,
                    current_kd_,
                    current_ki_
                );

            sdk_wrapper_->send_low_commands(
                indices,
                position,
                velocity,
                effort,
                kp,
                kd,
                ki
            );
        } else {
            RCLCPP_WARN_THROTTLE(
                logger_,
                *get_clock(),
                1000,
                "Received cmd_low but not in LowLevelMode"
            );
        }
    }
#endif

    void UnitreeInterface::estop_callback() {
        // TODO: Figure out how to implement the low-level estop here
        auto [new_mode, _] = try_transition<EmergencyMode>(current_mode_, *sdk_wrapper_);

        current_mode_ = new_mode;

        setup_mode_dependent_subscriptions();
        publish_current_mode();
    }

    void UnitreeInterface::tts_callback(const std_msgs::msg::String::SharedPtr message) { // NOLINT
        sdk_wrapper_->send_speech_command(message->data);
    }

    // ========== Publish methods ==========
    void UnitreeInterface::publish_current_mode() const {
        auto [mode_id, mode_name] = std::visit(
            [](const auto& mode) {
                using ModeType = std::decay_t<decltype(mode)>;

                return std::make_tuple(
                    ControlModeTraits<ModeType>::id,
                    ControlModeTraits<ModeType>::name()
                );
            },
            current_mode_
        );

        unitree_interface_msgs::msg::ControlMode message;
        message.header.stamp = now();
        message.current_mode_id = mode_id;
        message.current_mode_name = mode_name;

        current_mode_pub_->publish(message);
    }

    void UnitreeInterface::publish_current_profile() const {
        auto [profile_id, profile_name] = std::visit(
            [](const auto& profile) {
                using ProfileType = std::decay_t<decltype(profile)>;

                return std::make_tuple(
                    ProfileTraits<ProfileType>::id,
                    ProfileTraits<ProfileType>::name()
                );
            },
            current_profile_
        );

        unitree_interface_msgs::msg::Profile message;
        message.header.stamp = now();
        message.current_profile_id = profile_id;
        message.current_profile_name = profile_name;

        current_profile_pub_->publish(message);
    }

} // namespace unitree_interface
