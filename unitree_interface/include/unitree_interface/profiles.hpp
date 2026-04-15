#ifndef VECTOR_PROFILES_HPP
#define VECTOR_PROFILES_HPP

#include "unitree_interface/topology.hpp"
#include "unitree_interface_msgs/msg/profile.hpp"

#include <array>
#include <variant>

namespace unitree_interface {

    template <typename T>
    struct ProfileTraits;

    /*
    For each Profile, the joint kp / kd values must be set in the same order as the
    unitree_interface::embodiment::JointIndex definition.
    */

    // ========== Default ==========
    struct Default {
        static constexpr std::array<float, embodiment::num_joints> kp {
            // Left leg
            10.0F,
            10.0F,
            10.0F,
            100.0F,
            10.0F,
            10.0F,

            // Right leg
            10.0F,
            10.0F,
            10.0F,
            100.0F,
            10.0F,
            10.0F,

            // Waist
            150.0F,
            150.0F,
            150.0F,

            // Left arm
            80.0F,
            80.0F,
            80.0F,
            150.0F,
            40.0F,
            40.0F,
            40.0F,

            // Right arm
            80.0F,
            80.0F,
            80.0F,
            150.0F,
            40.0F,
            40.0F,
            40.0F
        };

        static constexpr std::array<float, embodiment::num_joints> kd {
            // Left leg
            2.0F,
            2.0F,
            2.0F,
            2.0F,
            2.0F,
            2.0F,

            // Right leg
            2.0F,
            2.0F,
            2.0F,
            2.0F,
            2.0F,
            2.0F,

            // Waist
            2.0F,
            3.5F,
            2.0F,

            // Left arm
            4.0F,
            4.0F,
            4.0F,
            4.0F,
            1.5F,
            1.5F,
            1.5F,

            // Right arm
            4.0F,
            4.0F,
            4.0F,
            4.0F,
            1.5F,
            1.5F,
            1.5F,
        };

        static constexpr std::array<float, embodiment::num_joints> ki {};
    };

    template <>
    struct ProfileTraits<Default> {
        static constexpr std::uint8_t id = unitree_interface_msgs::msg::Profile::PROFILE_DEFAULT;

        static constexpr const char* name() { return "Default"; }
    };

    // ========== Damp ==========
    // TODO: Test these
    struct Damp {
        static constexpr std::array<float, embodiment::num_joints> kp {};

        static constexpr std::array<float, embodiment::num_joints> kd {
            // Left leg
            8.0F,
            8.0F,
            8.0F,
            8.0F,
            8.0F,
            8.0F,

            // Right leg
            8.0F,
            8.0F,
            8.0F,
            8.0F,
            8.0F,
            8.0F,

            // Waist
            8.0F,
            8.0F,
            8.0F,

            // Left arm
            8.0F,
            8.0F,
            8.0F,
            8.0F,
            8.0F,
            8.0F,
            8.0F,

            // Right arm
            8.0F,
            8.0F,
            8.0F,
            8.0F,
            8.0F,
            8.0F,
            8.0F,
        };

        static constexpr std::array<float, embodiment::num_joints> ki {};
    };

    template <>
    struct ProfileTraits<Damp> {
        static constexpr std::uint8_t id = unitree_interface_msgs::msg::Profile::PROFILE_DAMP;

        static constexpr const char* name() { return "Damp"; }
    };

    // ========== VisualServo ==========
    // TODO: Tune these
    struct VisualServo {
        static constexpr std::array<float, embodiment::num_joints> kp = Default::kp;

        static constexpr std::array<float, embodiment::num_joints> kd = Default::kd;

        static constexpr std::array<float, embodiment::num_joints> ki {
            // Left leg
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,

            // Right leg
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,

            // Waist
            0.0F,
            0.0F,
            0.0F,

            // Left arm
            100.0F,
            100.0F,
            60.0F,
            150.0F,
            100.0F,
            60.0F,
            60.0F,

            // Right arm
            100.0F,
            100.0F,
            60.0F,
            150.0F,
            100.0F,
            60.0F,
            60.0F,
        };
    };

    template <>
    struct ProfileTraits<VisualServo> {
        static constexpr std::uint8_t id = unitree_interface_msgs::msg::Profile::PROFILE_VISUAL_SERVO;

        static constexpr const char* name() { return "VisualServo"; }
    };

    // ========== WholeBodyControl ==========
    // TODO: Tune these
    struct WholeBodyControl {
        static constexpr std::array<float, embodiment::num_joints> kp = Default::kp;

        static constexpr std::array<float, embodiment::num_joints> kd = Default::kd;

        static constexpr std::array<float, embodiment::num_joints> ki = Default::ki;
    };

    template <>
    struct ProfileTraits<WholeBodyControl> {
        static constexpr std::uint8_t id = unitree_interface_msgs::msg::Profile::PROFILE_WHOLE_BODY_CONTROL;

        static constexpr const char* name() { return "WholeBodyControl"; }
    };

    // ========== EffortOnly ==========
    struct EffortOnly {
        static constexpr std::array<float, embodiment::num_joints> kp {};

        static constexpr std::array<float, embodiment::num_joints> kd {};

        static constexpr std::array<float, embodiment::num_joints> ki {};
    };

    template <>
    struct ProfileTraits<EffortOnly> {
        static constexpr std::uint8_t id = unitree_interface_msgs::msg::Profile::PROFILE_EFFORT_ONLY;

        static constexpr const char* name() { return "EffortOnly"; }
    };

    // clang-format off
    using Profile = std::variant<
        Default,
        Damp,
        VisualServo,
        WholeBodyControl,
        EffortOnly
    >;
    // clang-format on

} // namespace unitree_interface

#endif // VECTOR_PROFILES_HPP
