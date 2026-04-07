#ifndef VECTOR_TOPOLOGY_HPP
#define VECTOR_TOPOLOGY_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace unitree_interface {

    namespace embodiment {

        namespace detail {

            template <typename T, std::size_t N1, std::size_t N2>
            constexpr std::array<T, N1 + N2> concat(
                const std::array<T, N1>& a,
                const std::array<T, N2>& b
            ) {
                std::array<T, N1 + N2> result{};
                for (std::size_t i = 0; i < N1; ++i) {
                    result[i] = a[i];
                }
                for (std::size_t i = 0; i < N2; ++i) {
                    result[N1 + i] = b[i];
                }
                return result;
            }

            template <typename T, std::size_t N1, std::size_t N2, std::size_t... Ns>
            constexpr auto concat(
                const std::array<T, N1>& a,
                const std::array<T, N2>& b,
                const std::array<T, Ns>&... rest
            ) {
                return concat(concat(a, b), rest...);
            }

        }

        constexpr std::size_t num_joints = 29;

        enum class JointIndex : std::uint8_t {
            // Left leg
            LeftHipPitch       = 0,
            LeftHipRoll        = 1,
            LeftHipYaw         = 2,
            LeftKnee           = 3,
            LeftAnklePitch     = 4,
            LeftAnkleRoll      = 5,

            // Right leg
            RightHipPitch      = 6,
            RightHipRoll       = 7,
            RightHipYaw        = 8,
            RightKnee          = 9,
            RightAnklePitch    = 10,
            RightAnkleRoll     = 11,

            // Waist
            WaistYaw           = 12,
            WaistRoll          = 13,
            WaistPitch         = 14,

            // Left arm
            LeftShoulderPitch  = 15,
            LeftShoulderRoll   = 16,
            LeftShoulderYaw    = 17,
            LeftElbow          = 18,
            LeftWristRoll      = 19,
            LeftWristPitch     = 20,
            LeftWristYaw       = 21,

            // Right arm
            RightShoulderPitch = 22,
            RightShoulderRoll  = 23,
            RightShoulderYaw   = 24,
            RightElbow         = 25,
            RightWristRoll     = 26,
            RightWristPitch    = 27,
            RightWristYaw      = 28,

            // Arm SDK weight parameter
            WeightParameter    = 29,
        };

        // ========== Helpers ==========
        template <std::size_t N>
        constexpr bool contains(const std::array<JointIndex, N>& group, JointIndex index) {
            for (std::size_t i = 0; i < N; ++i) {
                if (group[i] == index) { return true; }
            }
            return false;
        }

        // ========== Groups ==========
        // clang-format off
        constexpr std::array left_leg = {
            JointIndex::LeftHipPitch,
            JointIndex::LeftHipRoll,
            JointIndex::LeftHipYaw,
            JointIndex::LeftKnee,
            JointIndex::LeftAnklePitch,
            JointIndex::LeftAnkleRoll,
        };

        constexpr std::array right_leg = {
            JointIndex::RightHipPitch,
            JointIndex::RightHipRoll,
            JointIndex::RightHipYaw,
            JointIndex::RightKnee,
            JointIndex::RightAnklePitch,
            JointIndex::RightAnkleRoll,
        };

        constexpr std::array waist = {
            JointIndex::WaistYaw,
            JointIndex::WaistRoll,
            JointIndex::WaistPitch,
        };

        constexpr std::array left_arm = {
            JointIndex::LeftShoulderPitch,
            JointIndex::LeftShoulderRoll,
            JointIndex::LeftShoulderYaw,
            JointIndex::LeftElbow,
            JointIndex::LeftWristRoll,
            JointIndex::LeftWristPitch,
            JointIndex::LeftWristYaw,
        };

        constexpr std::array right_arm = {
            JointIndex::RightShoulderPitch,
            JointIndex::RightShoulderRoll,
            JointIndex::RightShoulderYaw,
            JointIndex::RightElbow,
            JointIndex::RightWristRoll,
            JointIndex::RightWristPitch,
            JointIndex::RightWristYaw,
        };
        // clang-format on

        constexpr auto legs = detail::concat(left_leg, right_leg);
        constexpr auto arms = detail::concat(left_arm, right_arm);
        constexpr auto upper_body = detail::concat(arms, waist);
        constexpr auto all_joints = detail::concat(legs, upper_body);

        // ========== Effort limits ==========
        // clang-format off
        constexpr std::array<float, num_joints> effort_limit {
            // Left leg
            88.0F,
            88.0F, // 139.0F in the URDF
            88.0F,
            139.0F,
            35.0F,
            35.0F,

            // Right leg
            88.0F,
            88.0F, // 139.0F in the URDF
            88.0F,
            139.0F,
            35.0F,
            35.0F,

            // Waist
            88.0F,
            35.0F,
            35.0F,

            // Left arm
            25.0F,
            25.0F,
            25.0F,
            25.0F,
            5.0F,
            5.0F,
            5.0F,

            // Right arm
            25.0F,
            25.0F,
            25.0F,
            25.0F,
            5.0F,
            5.0F,
            5.0F,
        };
        // clang-format on

        // ========== Name mapping ==========
        // clang-format off
        constexpr std::array<const char*, num_joints> joint_names = {
            "left_hip_pitch_joint",       // 0
            "left_hip_roll_joint",        // 1
            "left_hip_yaw_joint",         // 2
            "left_knee_joint",            // 3
            "left_ankle_pitch_joint",     // 4
            "left_ankle_roll_joint",      // 5

            "right_hip_pitch_joint",      // 6
            "right_hip_roll_joint",       // 7
            "right_hip_yaw_joint",        // 8
            "right_knee_joint",           // 9
            "right_ankle_pitch_joint",    // 10
            "right_ankle_roll_joint",     // 11

            "waist_yaw_joint",            // 12
            "waist_roll_joint",           // 13
            "waist_pitch_joint",          // 14

            "left_shoulder_pitch_joint",  // 15
            "left_shoulder_roll_joint",   // 16
            "left_shoulder_yaw_joint",    // 17
            "left_elbow_joint",           // 18
            "left_wrist_roll_joint",      // 19
            "left_wrist_pitch_joint",     // 20
            "left_wrist_yaw_joint",       // 21

            "right_shoulder_pitch_joint", // 22
            "right_shoulder_roll_joint",  // 23
            "right_shoulder_yaw_joint",   // 24
            "right_elbow_joint",          // 25
            "right_wrist_roll_joint",     // 26
            "right_wrist_pitch_joint",    // 27
            "right_wrist_yaw_joint",      // 28
        };
        // clang-format on

        inline const char* to_name(JointIndex index) {
            const auto i = static_cast<std::uint8_t>(index);

            if (i >= num_joints) {
                return nullptr;
            }

            return joint_names[i];
        }

        inline std::optional<JointIndex> from_name(const std::string& name) {
            for (std::size_t i = 0; i < num_joints; ++i) {
                if (name == joint_names[i]) {
                    return static_cast<JointIndex>(i);
                }
            }
            return std::nullopt;
        }

        // ========== Conversion ==========
        inline
        std::tuple<
            std::vector<std::uint8_t>,
            std::vector<float>,
            std::vector<float>,
            std::vector<float>,
            std::vector<float>,
            std::vector<float>,
            std::vector<float>
        >
        resolve_joint_commands(
            const std::vector<std::string>& names,
            const std::vector<double>& position,
            const std::vector<double>& velocity,
            const std::vector<double>& effort,
            const std::array<float, num_joints>& profile_kp,
            const std::array<float, num_joints>& profile_kd,
            const std::array<float, num_joints>& profile_ki
        ) {
            const std::size_t n = names.size();

            std::vector<std::uint8_t> indices{};
            std::vector<float> pos{};
            std::vector<float> vel{};
            std::vector<float> eff{};
            std::vector<float> kp{};
            std::vector<float> kd{};
            std::vector<float> ki{};

            indices.reserve(n);
            pos.reserve(n);
            vel.reserve(n);
            eff.reserve(n);
            kp.reserve(n);
            kd.reserve(n);
            ki.reserve(n);

            for (std::size_t i = 0; i < n; ++i) {
                auto joint_index = from_name(names[i]);

                // Skip unresolved names
                if (!joint_index.has_value()) {
                    continue;
                }

                // Narrow doubles to floats
                const auto index = static_cast<std::uint8_t>(*joint_index);

                indices.push_back(index);

                pos.push_back(i < position.size() ? static_cast<float>(position[i]) : 0.0F);
                vel.push_back(i < velocity.size() ? static_cast<float>(velocity[i]) : 0.0F);
                eff.push_back(i < effort.size()   ? static_cast<float>(effort[i])   : 0.0F);

                kp.push_back(profile_kp[index]);
                kd.push_back(profile_kd[index]);
                ki.push_back(profile_ki[index]);
            }

            return {indices, pos, vel, eff, kp, kd, ki};
        }

    } // namespace embodiment

    namespace hands {

        namespace detail {

            template <size_t N>
            constexpr std::array<float, N> negate(const std::array<float, N>& input) {
                std::array<float, N> result{};
                for (size_t i = 0; i < N; ++i) {
                    result[i] = -input[i];
                }

                return result;
            }

        }

        constexpr std::size_t num_joints = 7;

        enum class Side : std::uint8_t {
            Left  = 0,
            Right = 1,
        };

        // ========== Motor mode encoding ==========
        constexpr std::uint8_t encode_motor_mode(
            std::uint8_t motor_id,
            std::uint8_t status = 0x01,
            std::uint8_t timeout = 0x00
        ) {
            return (motor_id & 0x0F)
                | ((status & 0x07) << 4)
                | ((timeout & 0x01) << 7);
        }

        constexpr std::uint8_t stop_mode(std::uint8_t motor_id) {
            return encode_motor_mode(motor_id, 0x00, 0x01);
        }

        // ========== Default gains ==========
        constexpr std::array<float, num_joints> kp {
            1.5F,
            1.5F,
            1.5F,
            1.5F,
            1.5F,
            1.5F,
            1.5F,
        };

        constexpr std::array<float, num_joints> kd {
            0.1F,
            0.1F,
            0.1F,
            0.1F,
            0.1F,
            0.1F,
            0.1F,
        };

        // ========== Poses ==========
        // clang-format off
        constexpr std::array<float, num_joints> ball_left {
            0.0F,
            1.0F,
            1.5F,
            -1.75F,
            -1.75F,
            -1.75F,
            -1.75F,
        };

        constexpr std::array<float, num_joints> point_left {
            0.0F,
            1.0F,
            1.5F,
            -1.75F,
            -1.75F,
            -0.05F,
            -0.05F,
        };

        constexpr std::array<float, num_joints> ball_right = detail::negate(ball_left);

        constexpr std::array<float, num_joints> point_right = detail::negate(point_left);
        // clang-format on

    } // namespace hands

} // namespace unitree_interface

#endif // VECTOR_TOPOLOGY_HPP
