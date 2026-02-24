#ifndef VECTOR_PROFILES_HPP
#define VECTOR_PROFILES_HPP

// #include "unitree_interface/control_modes.hpp"
#include "unitree_interface/topology.hpp"
#include "unitree_interface_msgs/msg/profile.hpp"

#include <variant>

namespace unitree_interface {

    template <typename T>
    struct ProfileTraits;

    /*
    For each Profile, the joint kp / kd values must be set in the same order as the
    unitree_interface::joints::JointIndex definition.
    */

    // ========== Default ==========
    struct Default {
        static constexpr std::array<float, joints::num_joints> kp {
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
            10.0F,
            10.0F,
            10.0F,

            // Left arm
            10.0F,
            10.0F,
            10.0F,
            10.0F,
            10.0F,
            10.0F,
            10.0F,

            // Right arm
            10.0F,
            10.0F,
            10.0F,
            10.0F,
            10.0F,
            10.0F,
            10.0F,
        };

        static constexpr std::array<float, joints::num_joints> kd {
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
            2.0F,
            2.0F,

            // Left arm
            2.0F,
            2.0F,
            2.0F,
            2.0F,
            2.0F,
            2.0F,
            2.0F,

            // Right arm
            2.0F,
            2.0F,
            2.0F,
            2.0F,
            2.0F,
            2.0F,
            2.0F,
        };
    };

    template <>
    struct ProfileTraits<Default> {
        static constexpr std::uint8_t id = unitree_interface_msgs::msg::Profile::PROFILE_DEFAULT;

        static constexpr const char* name() { return "Default"; }
    };

    // ========== Damp ==========
    // TODO: Test these
    struct Damp {
        static constexpr std::array<float, joints::num_joints> kp {};

        static constexpr std::array<float, joints::num_joints> kd {
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
    };

    template <>
    struct ProfileTraits<Damp> {
        static constexpr std::uint8_t id = unitree_interface_msgs::msg::Profile::PROFILE_DAMP;

        static constexpr const char* name() { return "Damp"; }
    };

    // ========== VisualServo ==========
    // TODO: Tune these
    struct VisualServo {
        static constexpr std::array<float, joints::num_joints> kp = Default::kp;

        static constexpr std::array<float, joints::num_joints> kd = Default::kd;
    };

    template <>
    struct ProfileTraits<VisualServo> {
        static constexpr std::uint8_t id = unitree_interface_msgs::msg::Profile::PROFILE_VISUAL_SERVO;

        static constexpr const char* name() { return "VisualServo"; }
    };

    // ========== WholeBodyControl ==========
    // TODO: Tune these
    struct WholeBodyControl {
        static constexpr std::array<float, joints::num_joints> kp = Default::kp;

        static constexpr std::array<float, joints::num_joints> kd = Default::kd;
    };

    template <>
    struct ProfileTraits<WholeBodyControl> {
        static constexpr std::uint8_t id = unitree_interface_msgs::msg::Profile::PROFILE_WHOLE_BODY_CONTROL;

        static constexpr const char* name() { return "WholeBodyControl"; }
    };

    // ========== EffortOnly ==========
    struct EffortOnly {
        static constexpr std::array<float, joints::num_joints> kp {};

        static constexpr std::array<float, joints::num_joints> kd {};
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

    using GainArrays = std::pair<
        const std::array<float, joints::num_joints>&,
        const std::array<float, joints::num_joints>&
    >;

    inline GainArrays get_profile_gains(const Profile& profile) {
        return std::visit([](const auto& p) -> GainArrays {
            return {p.kp, p.kd};
        }, profile);
    }

} // namespace unitree_interface

#endif // VECTOR_PROFILES_HPP
