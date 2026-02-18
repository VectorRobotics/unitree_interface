#include "unitree_interface/unitree_interface.hpp"

#include "unitree_interface/control_modes.hpp"
#include "unitree_interface/mode_transitions.hpp"
#include "unitree_interface/unitree_sdk_wrapper.hpp"
#include "unitree_interface/topology.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <thread>
#include <tuple>
#include <type_traits>
#include <variant>

namespace unitree_interface {

    UnitreeInterface::UnitreeInterface(const rclcpp::NodeOptions& options)
        : Node("unitree_interface", options),
          logger_(get_logger()),
          current_mode_(std::monostate{}) {
        // ========== Parameters ==========
        declare_parameter("motion_switcher_client_timeout", 5.0F); // NOLINT
        declare_parameter("loco_client_timeout", 10.0F); // NOLINT
        declare_parameter("audio_client_timeout", 5.0F); // NOLINT
        declare_parameter("volume", 100); // NOLINT
        declare_parameter("ready_locomotion_stand_up_delay", 5); // NOLINT
        declare_parameter("ready_locomotion_start_delay", 10); // NOLINT

        declare_parameter("mode_change_service_name", "~/change_mode");
        declare_parameter("ready_locomotion_service_name", "~/ready_locomotion");
        declare_parameter("current_mode_topic", "~/current_mode");
        declare_parameter("cmd_vel_topic", "~/cmd_vel");
        declare_parameter("cmd_arm_topic", "~/cmd_arm");
        declare_parameter("joint_states_topic", "~/joint_states");
        declare_parameter("tts_topic", "~/tts");
#ifdef UNITREE_INTERFACE_ENABLE_LOW_LEVEL_MODE
        declare_parameter("cmd_low_topic", "~/cmd_low");
#endif
        declare_parameter("estop_topic", "/estop");

        // ========== Grab parameters ==========
        mode_change_service_name_ = get_parameter("mode_change_service_name").as_string();
        ready_locomotion_service_name_ = get_parameter("ready_locomotion_service_name").as_string();
        current_mode_topic_ = get_parameter("current_mode_topic").as_string();
        cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
        cmd_arm_topic_ = get_parameter("cmd_arm_topic").as_string();
        joint_states_topic_ = get_parameter("joint_states_topic").as_string();
        tts_topic_ = get_parameter("tts_topic").as_string();
#ifdef UNITREE_INTERFACE_ENABLE_LOW_LEVEL_MODE
        cmd_low_topic_ = get_parameter("cmd_low_topic").as_string();
#endif
        estop_topic_ = get_parameter("estop_topic").as_string();
        volume_ = static_cast<std::uint8_t>(get_parameter("volume").as_int());
    }

    // ========== Initialization ==========
    void UnitreeInterface::initialize() {
        if (sdk_wrapper_) {
            RCLCPP_INFO(logger_, "UnitreeInterface already intialized");
            return;
        }

        const float msc_timeout = static_cast<float>(
            get_parameter("motion_switcher_client_timeout").as_double()
        );
        const float loco_client_timeout = static_cast<float>(
            get_parameter("loco_client_timeout").as_double()
        );
        const float audio_client_timeout = static_cast<float>(
            get_parameter("audio_client_timeout").as_double()
        );

        sdk_wrapper_ = std::make_unique<UnitreeSDKWrapper>(logger_);
        // At this point, sdk_wrapper_ is guaranteed to not be a nullptr

        if (!sdk_wrapper_->initialize(msc_timeout, loco_client_timeout, audio_client_timeout)) {
            /*
            sdk_wrapper_.initialize() might be able to throw, if the SDK's init
            methods themselves can throw. Unfortunately, we don't know if that's
            the case here. It most likely isn't, since the sdk seems to use errors
            as returns.
            */
            RCLCPP_ERROR(logger_, "Failed to initialize SDK wrapper");
            throw std::runtime_error("Unitree SDK initialization failed");
        }

        sdk_wrapper_->set_volume(volume_);

        current_mode_ = Transition<std::monostate, IdleMode>::execute(*sdk_wrapper_);
        RCLCPP_INFO(
            logger_,
            "Initialized to %s",
            ControlModeTraits<IdleMode>::name()
        );

        initialize_services();
        initialize_publishers();
        sdk_wrapper_->set_joint_states_publisher(joint_states_pub_);
        create_subscriptions();

        setup_mode_dependent_subscriptions();
        publish_current_mode();
    }

    void UnitreeInterface::initialize_services() {
        mode_change_service_ = create_service<unitree_interface_msgs::srv::ChangeControlMode>(
            mode_change_service_name_,
            [this](
                const unitree_interface_msgs::srv::ChangeControlMode::Request::SharedPtr request, // NOLINT
                unitree_interface_msgs::srv::ChangeControlMode::Response::SharedPtr response
            ) {
                handle_mode_change_request(request, response); // NOLINT
            }
        );
        ready_locomotion_service_ = create_service<std_srvs::srv::Trigger>(
            ready_locomotion_service_name_,
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr request, // NOLINT
                std_srvs::srv::Trigger::Response::SharedPtr response
            ) {
                handle_ready_locomotion_request(request, response); // NOLINT
            }
        );

        RCLCPP_INFO(logger_, "Services created");
    }

    void UnitreeInterface::initialize_publishers() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                        .transient_local()
                        .reliable();

        current_mode_pub_ = create_publisher<unitree_interface_msgs::msg::ControlMode>(
            current_mode_topic_,
            qos
        );

        joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            joint_states_topic_,
            rclcpp::QoS(10) // NOLINT
        );

        RCLCPP_INFO(logger_, "Publishers created");
    }

    void UnitreeInterface::create_subscriptions() {
        estop_sub_ = create_subscription<std_msgs::msg::Empty>(
            estop_topic_,
            rclcpp::QoS(1),
            [this](const std_msgs::msg::Empty::SharedPtr) { // NOLINT
                estop_callback();
            }
        );

        tts_sub_ = create_subscription<std_msgs::msg::String>(
            tts_topic_,
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
                    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
                        cmd_vel_topic_,
                        rclcpp::QoS(10), // NOLINT
                        [this](const geometry_msgs::msg::Twist::SharedPtr message) { // NOLINT
                            cmd_vel_callback(message);
                        }
                    );

                    cmd_arm_sub_ = create_subscription<sensor_msgs::msg::JointState>(
                        cmd_arm_topic_,
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
                        cmd_low_topic_,
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

        if (success) {
            RCLCPP_INFO(logger_, "Mode transition successful");

            setup_mode_dependent_subscriptions();
            publish_current_mode();

            response->success = success;
            response->message = "Mode transition successful";
        } else {
            RCLCPP_WARN(logger_, "Mode transition failed");
        }
    }

    void UnitreeInterface::handle_ready_locomotion_request(
        const std_srvs::srv::Trigger::Request::SharedPtr, // NOLINT
        std_srvs::srv::Trigger::Response::SharedPtr response // NOLINT
    ) {
        if (std::holds_alternative<HighLevelMode>(current_mode_)) {
            RCLCPP_INFO(logger_, "Ready locomotion sequence requested");

            const std::int64_t stand_up_delay = get_parameter("ready_locomotion_stand_up_delay").as_int();
            const std::int64_t start_delay = get_parameter("ready_locomotion_start_delay").as_int();

            if (!sdk_wrapper_->damp()) {
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

    void UnitreeInterface::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr message) { // NOLINT
        if (std::holds_alternative<HighLevelMode>(current_mode_)) {
            const auto vx = static_cast<float>(message->linear.x); // m/s
            const auto vy = static_cast<float>(message->linear.y); // m/s
            const auto vyaw = static_cast<float>(message->angular.z); // rad/s

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
        if (std::holds_alternative<HighLevelMode>(current_mode_)) {
            auto [indices, position, velocity, effort, kp, kd] =
                joints::resolve_joint_commands(
                    message->name,
                    message->position,
                    message->velocity,
                    message->effort

                );

            for (std::size_t i = 0; i < indices.size(); ++i) {
                const auto joint_index = static_cast<joints::JointIndex>(indices[i]);

                if (!joints::contains(joints::upper_body, joint_index)) {
                    position[i] = 0.0F;
                    velocity[i] = 0.0F;
                    effort[i] = 0.0F;
                    kp[i] = 0.0F;
                    kd[i] = 0.0F;
                }
            }

            sdk_wrapper_->send_arm_commands(
                indices,
                position,
                velocity,
                effort,
                kp,
                kd
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
            auto [indices, position, velocity, effort, kp, kd] =
                joints::resolve_joint_commands(
                    message->name,
                    message->position,
                    message->velocity,
                    message->effort
                );

            sdk_wrapper_->send_low_commands(
                indices,
                position,
                velocity,
                effort,
                kp,
                kd
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
    void UnitreeInterface::publish_current_mode() {
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

} // namespace unitree_interface