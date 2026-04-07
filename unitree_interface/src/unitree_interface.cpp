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
#include <tuple>

namespace unitree_interface {

    template <typename ProfileType>
    void UnitreeInterface::declare_profile_gains(const bool dynamic) {
        using dynamic_params::FloatRange;

        const std::string prefix = std::string(ProfileTraits<ProfileType>::name()) + "/";

        for (std::size_t i = 0; i < embodiment::num_joints; ++i) {
            params_.declare(
                prefix + "kp/" + embodiment::joint_names[i],
                static_cast<double>(ProfileType::kp[i]),
                FloatRange{0.0, 200.0},
                "",
                dynamic,
                [this, i](const rclcpp::Parameter& param) {
                    std::unique_lock lock(state_mutex_);

                    if (ProfileTraits<ProfileType>::id == get_active_profile_id()) {
                        current_kp_[i] = static_cast<float>(param.as_double());
                    }
                }
            );

            params_.declare(
                prefix + "kd/" + embodiment::joint_names[i],
                static_cast<double>(ProfileType::kd[i]),
                FloatRange{0.0, 50.0},
                "",
                dynamic,
                [this, i](const rclcpp::Parameter& param) {
                    std::unique_lock lock(state_mutex_);

                    if (ProfileTraits<ProfileType>::id == get_active_profile_id()) {
                        current_kd_[i] = static_cast<float>(param.as_double());
                    }
                }
            );

            params_.declare(
                prefix + "ki/" + embodiment::joint_names[i],
                static_cast<double>(ProfileType::ki[i]),
                FloatRange{0.0, 12.0},
                "",
                dynamic,
                [this, i](const rclcpp::Parameter& param) {
                    std::unique_lock lock(state_mutex_);

                    if (ProfileTraits<ProfileType>::id == get_active_profile_id()) {
                        current_ki_[i] = static_cast<float>(param.as_double());
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

        // ========== Callback groups ==========
        estop_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        command_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        service_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // ========== Static parameters ==========
        params_.declare("mode_change_service_name", "~/change_mode", "", false);
        params_.declare("ready_locomotion_service_name", "~/ready_locomotion", "", false);
        params_.declare("release_arms_service_name", "~/release_arms", "", false);
        params_.declare("reset_integral_error_service_name", "~/reset_integral_error", "", false);
        params_.declare("set_profile_service_name", "~/set_profile", "", false);
        params_.declare("set_hand_pose_service_name", "~/set_hand_pose", "", false);

        params_.declare("current_mode_topic", "~/current_mode", "", false);
        params_.declare("current_profile_topic", "~/current_profile", "", false);
        params_.declare("cmd_vel_topic", "~/cmd_vel", "", false);
        params_.declare("cmd_arm_topic", "~/cmd_arm", "", false);
        params_.declare("joint_states_topic", "~/joint_states", "", false);
        params_.declare("tts_topic", "~/tts", "", false);
        params_.declare("cmd_low_topic", "~/cmd_low", "", false);
        params_.declare("estop_topic", "/estop", "", false);
        params_.declare("left_hand_states_topic", "~/left_hand_states", "", false);
        params_.declare("right_hand_states_topic", "~/right_hand_states", "", false);

        params_.declare("motion_switcher_client_timeout", 5.0, FloatRange{1.0, 30.0, 0.5}, "", false);
        params_.declare("loco_client_timeout", 10.0, FloatRange{1.0, 30.0, 0.5}, "", false);
        params_.declare("audio_client_timeout", 5.0, FloatRange{1.0, 30.0, 0.5}, "", false);

        // ========== Dynamic parameters ==========
        params_.declare("volume", 100, IntRange{0, 100});
        params_.declare("ready_locomotion_stand_up_delay", 5, IntRange{0, 30});
        params_.declare("ready_locomotion_start_delay", 10, IntRange{0, 30});
        params_.declare("release_arms_steps", 250, IntRange{1, 1000});
        params_.declare("release_arms_interval_ms", 20, IntRange{1, 100});
        params_.declare("hand_pose_steps", 100, IntRange{1, 500});
        params_.declare("hand_pose_interval_ms", 10, IntRange{1, 100});

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
        sdk_wrapper_->set_hand_states_publishers(left_hand_states_pub_, right_hand_states_pub_);
        create_subscriptions();

        setup_mode_dependent_subscriptions();
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
            },
            rclcpp::ServicesQoS(),
            service_cbg_
        );

        ready_locomotion_service_ = create_service<std_srvs::srv::Trigger>(
            params_.get_string("ready_locomotion_service_name"),
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr request, // NOLINT
                std_srvs::srv::Trigger::Response::SharedPtr response
            ) {
                handle_ready_locomotion_request(request, response); // NOLINT
            },
            rclcpp::ServicesQoS(),
            service_cbg_
        );

        release_arms_service_ = create_service<std_srvs::srv::Trigger>(
            params_.get_string("release_arms_service_name"),
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr request, // NOLINT
                std_srvs::srv::Trigger::Response::SharedPtr response
            ) {
                handle_release_arms_request(request, response); // NOLINT
            },
            rclcpp::ServicesQoS(),
            service_cbg_
        );

        reset_integral_error_service_ = create_service<std_srvs::srv::Trigger>(
            params_.get_string("reset_integral_error_service_name"),
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr request, // NOLINT
                std_srvs::srv::Trigger::Response::SharedPtr response
            ) {
                handle_reset_integral_error_request(request, response); // NOLINT
            },
            rclcpp::ServicesQoS(),
            service_cbg_
        );

        set_profile_service_ = create_service<unitree_interface_msgs::srv::SetProfile>(
            params_.get_string("set_profile_service_name"),
            [this](
                const unitree_interface_msgs::srv::SetProfile::Request::SharedPtr request, // NOLINT
                unitree_interface_msgs::srv::SetProfile::Response::SharedPtr response
            ) {
                handle_set_profile_request(request, response); // NOLINT
            },
            rclcpp::ServicesQoS(),
            service_cbg_
        );

        set_hand_pose_service_ = create_service<unitree_interface_msgs::srv::SetHandPose>(
            params_.get_string("set_hand_pose_service_name"),
            [this](
                const unitree_interface_msgs::srv::SetHandPose::Request::SharedPtr request, // NOLINT
                unitree_interface_msgs::srv::SetHandPose::Response::SharedPtr response
            ) {
                handle_set_hand_pose_request(request, response); // NOLINT
            },
            rclcpp::ServicesQoS(),
            service_cbg_
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

        left_hand_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            params_.get_string("left_hand_states_topic"),
            rclcpp::QoS(10) // NOLINT
        );

        right_hand_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            params_.get_string("right_hand_states_topic"),
            rclcpp::QoS(10) // NOLINT
        );

        RCLCPP_INFO(logger_, "Publishers created");
    }

    void UnitreeInterface::create_subscriptions() {
        rclcpp::SubscriptionOptions estop_options;
        estop_options.callback_group = estop_cbg_;

        estop_sub_ = create_subscription<std_msgs::msg::Empty>(
            params_.get_string("estop_topic"),
            rclcpp::QoS(1),
            [this](const std_msgs::msg::Empty::SharedPtr) { // NOLINT
                estop_callback();
            },
            estop_options
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
        cmd_low_sub_.reset();

        rclcpp::SubscriptionOptions cmd_options;
        cmd_options.callback_group = command_cbg_;

        // Create subscriptions based on current mode
        std::visit(
            [this, &cmd_options](auto&& mode){
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
                        },
                        cmd_options
                    );

                    cmd_arm_sub_ = create_subscription<sensor_msgs::msg::JointState>(
                        params_.get_string("cmd_arm_topic"),
                        rclcpp::QoS(10), // NOLINT
                        [this](const sensor_msgs::msg::JointState::SharedPtr message) { // NOLINT
                            cmd_arm_callback(message);
                        },
                        cmd_options
                    );

                    RCLCPP_INFO(
                        logger_,
                        "%s: subscriptions created",
                        ControlModeTraits<HighLevelMode>::name()
                    );
                } else if constexpr (std::is_same_v<ModeType, LowLevelMode>) {
                    cmd_low_sub_ = create_subscription<sensor_msgs::msg::JointState>(
                        params_.get_string("cmd_low_topic"),
                        rclcpp::QoS(10), // NOLINT
                        [this](const sensor_msgs::msg::JointState::SharedPtr message) { // NOLINT
                            cmd_low_callback(message);
                        },
                        cmd_options
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

        for (std::size_t i = 0; i < embodiment::num_joints; ++i) {
            current_kp_[i] = static_cast<float>(
                params_.get_double(prefix + "kp/" + embodiment::joint_names[i])
            );
            current_kd_[i] = static_cast<float>(
                params_.get_double(prefix + "kd/" + embodiment::joint_names[i])
            );
            current_ki_[i] = static_cast<float>(
                params_.get_double(prefix + "ki/" + embodiment::joint_names[i])
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

        {
            std::unique_lock lock(state_mutex_);

            bool success{false};
            std::tie(current_mode_, success) = try_transition_to(
                current_mode_,
                requested_mode,
                *sdk_wrapper_
            );

            response->success = success;

            if (success) {
                RCLCPP_INFO(logger_, "Mode transition successful");

                sdk_wrapper_->reset_integral_error();
                setup_mode_dependent_subscriptions();

                response->success = success;
                response->message = "Mode transition successful";
            } else {
                response->message = "Mode transition failed";
                RCLCPP_WARN(logger_, "Mode transition failed");
            }
        }

        if (response->success) {
            publish_current_mode();
        }
    }

    void UnitreeInterface::handle_ready_locomotion_request(
        const std_srvs::srv::Trigger::Request::SharedPtr, // NOLINT
        std_srvs::srv::Trigger::Response::SharedPtr response // NOLINT
    ) {
        {
            std::shared_lock lock(state_mutex_);

            if (!std::holds_alternative<HighLevelMode>(current_mode_)) {
                response->success = false;
                response->message = "Ready locomotion sequence attempted while not in HighLevelMode";
                RCLCPP_WARN(logger_, "Ready locomotion sequence attempted while not in HighLevelMode");
                return;
            }
        }

        RCLCPP_INFO(logger_, "Ready locomotion sequence requested");

        const auto stand_up_delay = params_.get_int("ready_locomotion_stand_up_delay");
        const auto start_delay = params_.get_int("ready_locomotion_start_delay");

        if (!sdk_wrapper_->damp_high()) {
            response->success = false;
            response->message = "damp() failed";
            return;
        }

        std::this_thread::sleep_for(std::chrono::seconds(stand_up_delay));

        {
            std::shared_lock lock(state_mutex_);

            if (std::holds_alternative<EmergencyMode>(current_mode_)) {
                response->success = false;
                response->message = "Aborted: estop triggered during ready_locomotion";
                return;
            }
        }

        if (!sdk_wrapper_->stand_up()) {
            response->success = false;
            response->message = "stand_up() failed";
            return;
        }

        std::this_thread::sleep_for(std::chrono::seconds(start_delay));

        {
            std::shared_lock lock(state_mutex_);

            if (std::holds_alternative<EmergencyMode>(current_mode_)) {
                response->success = false;
                response->message = "Aborted: estop triggered during ready_locomotion";
                return;
            }
        }

        if (!sdk_wrapper_->start()) {
            response->success = false;
            response->message = "start() failed";
            return;
        }

        response->success = true;
        response->message = "Locomotion ready";
        RCLCPP_INFO(logger_, "Locomotion ready sequence completed");
    }

    void UnitreeInterface::handle_release_arms_request(
        const std_srvs::srv::Trigger::Request::SharedPtr, // NOLINT
        std_srvs::srv::Trigger::Response::SharedPtr response // NOLINT
    ) {
        {
            std::shared_lock lock(state_mutex_);

            if (!std::holds_alternative<HighLevelMode>(current_mode_)) {
                response->success = false;
                response->message = "Not in HighLevelMode";
                RCLCPP_WARN(logger_, "Release arms requested while not in HighLevelMode");
                return;
            }
        }

        const auto steps = params_.get_int("release_arms_steps");
        const auto interval_ms = params_.get_int("release_arms_interval_ms");

        releasing_arms_ = true;
        sdk_wrapper_->release_arms(steps, interval_ms);
        releasing_arms_ = false;

        response->success = true;
        response->message = "Arms released";
    }

    void UnitreeInterface::handle_set_profile_request(
        const unitree_interface_msgs::srv::SetProfile::Request::SharedPtr request, // NOLINT
        unitree_interface_msgs::srv::SetProfile::Response::SharedPtr response // NOLINT
    ) {
        const std::uint8_t requested = request->requested_profile;

        {
            std::unique_lock lock(state_mutex_);

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
        }
        publish_current_profile();

        response->success = true;
        response->message = std::visit(
            [](const auto& profile) -> std::string {
                using ProfileType = std::decay_t<decltype(profile)>;

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

    void UnitreeInterface::handle_set_hand_pose_request(
        const unitree_interface_msgs::srv::SetHandPose::Request::SharedPtr request, // NOLINT
        unitree_interface_msgs::srv::SetHandPose::Response::SharedPtr response // NOLINT
    ) {
        {
            std::shared_lock lock(state_mutex_);

            if (!std::holds_alternative<HighLevelMode>(current_mode_)) {
                response->success = false;
                response->message = "Not in HighLevelMode";
                RCLCPP_WARN(logger_, "Hand pose requested while not in HighLevelMode");
                return;
            }
        }

        const auto side = (request->target_hand == unitree_interface_msgs::srv::SetHandPose::Request::HAND_LEFT)
            ? hands::Side::Left
            : hands::Side::Right;

        const std::array<float, hands::num_joints>* target = nullptr;

        switch (request->target_pose) {
            case unitree_interface_msgs::srv::SetHandPose::Request::POSE_BALL:
                target = (side == hands::Side::Left) ? &hands::ball_left : &hands::ball_right;
                break;
            case unitree_interface_msgs::srv::SetHandPose::Request::POSE_POINT:
                target = (side == hands::Side::Left) ? &hands::point_left : &hands::point_right;
                break;
            default:
                response->success = false;
                response->message = "Unknown pose ID";
                return;
        }

        const auto steps = params_.get_int("hand_pose_steps");
        const auto interval_ms = params_.get_int("hand_pose_interval_ms");
        const auto interval = std::chrono::milliseconds(interval_ms);

        const auto start_pos = sdk_wrapper_->get_hand_position(side);

        for (int step = 0; step <= steps; ++step) {
            const float t = static_cast<float>(step) / static_cast<float>(steps);

            std::array<float, hands::num_joints> interpolated{};
            for (std::size_t i = 0; i < hands::num_joints; ++i) {
                interpolated[i] = start_pos[i] + t * ((*target)[i] - start_pos[i]);
            }

            sdk_wrapper_->send_hand_command(side, interpolated);
    
            std::this_thread::sleep_for(interval);
        }

        const auto side_name = (side == hands::Side::Left) ? "left" : "right";
        response->success = true;
        response->message = std::string("Hand pose set for ") + side_name;
        RCLCPP_INFO(logger_, "%s", response->message.c_str());
    }

    void UnitreeInterface::cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr message) { // NOLINT
        ControlMode mode_snapshot;
        {
            std::shared_lock lock(state_mutex_);

            mode_snapshot = current_mode_;
        }

        if (std::holds_alternative<HighLevelMode>(mode_snapshot)) {
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

        std::array<float, embodiment::num_joints> kp_snapshot;
        std::array<float, embodiment::num_joints> kd_snapshot;
        std::array<float, embodiment::num_joints> ki_snapshot;
        {
            std::shared_lock lock(state_mutex_);

            if (!std::holds_alternative<HighLevelMode>(current_mode_)) {
                RCLCPP_WARN_THROTTLE(
                    logger_,
                    *get_clock(),
                    1000,
                    "Received cmd_arm but not in HighLevelMode"
                );
                return;
            }
            kp_snapshot = current_kp_;
            kd_snapshot = current_kd_;
            ki_snapshot = current_ki_;
        }

        auto cmd = embodiment::resolve_names(
            message->name,
            message->position,
            message->velocity,
            message->effort,
            embodiment::upper_body
        );

        sdk_wrapper_->send_arm_commands(
            cmd.active,
            cmd.position,
            cmd.velocity,
            cmd.effort,
            kp_snapshot,
            kd_snapshot,
            ki_snapshot
        );
    }

    void UnitreeInterface::cmd_low_callback(const sensor_msgs::msg::JointState::SharedPtr message) { // NOLINT
        std::array<float, embodiment::num_joints> kp_snapshot;
        std::array<float, embodiment::num_joints> kd_snapshot;
        std::array<float, embodiment::num_joints> ki_snapshot;
        {
            std::shared_lock lock(state_mutex_);

            if (!std::holds_alternative<LowLevelMode>(current_mode_)) {
                RCLCPP_WARN_THROTTLE(
                    logger_,
                    *get_clock(),
                    1000,
                    "Received cmd_low but not in LowLevelMode"
                );
                return;
            }
            kp_snapshot = current_kp_;
            kd_snapshot = current_kd_;
            ki_snapshot = current_ki_;
        }

        auto cmd = embodiment::resolve_names(
            message->name,
            message->position,
            message->velocity,
            message->effort,
            embodiment::all_joints
        );

        sdk_wrapper_->send_low_commands(
            cmd.active,
            cmd.position,
            cmd.velocity,
            cmd.effort,
            kp_snapshot,
            kd_snapshot,
            ki_snapshot
        );
    }

    void UnitreeInterface::estop_callback() {
        std::unique_lock lock(state_mutex_);

        std::tie(current_mode_, std::ignore) = try_transition<EmergencyMode>(current_mode_, *sdk_wrapper_);

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
