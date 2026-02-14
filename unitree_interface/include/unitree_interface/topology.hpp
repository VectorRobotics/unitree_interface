#ifndef VECTOR_TOPOLOGY_HPP
#define VECTOR_TOPOLOGY_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace unitree_interface::joints {

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

    constexpr auto legs = concat(left_leg, right_leg);
    constexpr auto arms = concat(left_arm, right_arm);
    constexpr auto upper_body = concat(arms, waist);
    constexpr auto all_joints = concat(legs, upper_body);

    // ========== Motor gains ==========
    enum class Gearbox : std::uint8_t {
        Small,
        Medium,
        Large
    };

    // clang-format off
    constexpr std::array<Gearbox, num_joints> joint_gearbox = {
        // Left leg
        Gearbox::Medium,
        Gearbox::Medium,
        Gearbox::Medium,
        Gearbox::Large,
        Gearbox::Small,
        Gearbox::Small,

        // Right leg
        Gearbox::Medium,
        Gearbox::Medium,
        Gearbox::Medium,
        Gearbox::Large,
        Gearbox::Small,
        Gearbox::Small,

        // Waist
        Gearbox::Medium,
        Gearbox::Small,
        Gearbox::Small,

        // Left arm
        Gearbox::Small,
        Gearbox::Small,
        Gearbox::Small,
        Gearbox::Small,
        Gearbox::Small,
        Gearbox::Small,
        Gearbox::Small,

        // Right arm
        Gearbox::Small,
        Gearbox::Small,
        Gearbox::Small,
        Gearbox::Small,
        Gearbox::Small,
        Gearbox::Small,
        Gearbox::Small,
    };
    // clang-format on

    constexpr float default_kp(Gearbox gearbox) {
        switch (gearbox) {
            case Gearbox::Small: return 10.F;
            case Gearbox::Medium: return 10.F;
            case Gearbox::Large: return 100.F;
        }
        return 0.F;
    }

    constexpr float default_kd(Gearbox gearbox) {
        switch (gearbox) {
            case Gearbox::Small: return 2.F;
            case Gearbox::Medium: return 2.F;
            case Gearbox::Large: return 2.F;
        }
        return 0.F;
    }

    constexpr float default_kp(JointIndex index) {
        return default_kp(joint_gearbox[static_cast<std::uint8_t>(index)]);
    }

    constexpr float default_kd(JointIndex index) {
        return default_kd(joint_gearbox[static_cast<std::uint8_t>(index)]);
    }

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

    // ========== Validation ==========
    template <std::size_t N>
    inline bool all_in_group(
        const std::vector<std::string>& names,
        const std::array<JointIndex, N>& group
    ) {
        for (const auto& name : names) {
            auto index = from_name(name);
            if (!index || !contains(group, *index)) {
                return false;
            }
        }
        return true;
    }

    inline bool has_duplicates(const std::vector<std::string>& names) {
        for (std::size_t i = 0; i < names.size(); ++i) {
            for (std::size_t j = i + 1; j < names.size(); ++j) {
                if (names[i] == names[j]) { return true; }
            }
        }
        return false;
    }

    // ========== Conversion ==========
    inline std::tuple<
        std::vector<std::uint8_t>,
        std::vector<float>,
        std::vector<float>,
        std::vector<float>,
        std::vector<float>,
        std::vector<float>
    > resolve_joint_commands(
        const std::vector<std::string>& names,
        const std::vector<double>& position,
        const std::vector<double>& velocity,
        const std::vector<double>& effort
    ) {
        const std::size_t n = names.size();

        std::vector<std::uint8_t> indices{};
        std::vector<float> pos{};
        std::vector<float> vel{};
        std::vector<float> eff{};
        std::vector<float> kp{};
        std::vector<float> kd{};

        indices.reserve(n);
        pos.reserve(n);
        vel.reserve(n);
        eff.reserve(n);
        kp.reserve(n);
        kd.reserve(n);

        for (std::size_t i = 0; i < n; ++i) {
            auto index = from_name(names[i]);

            // Skip unresolved names
            if (!index.has_value()) {
                continue;
            }

            // Narrow doubles to floats
            indices.push_back(static_cast<std::uint8_t>(*index));
            pos.push_back(i < position.size() ? static_cast<float>(position[i]) : 0.0f);
            vel.push_back(i < velocity.size() ? static_cast<float>(velocity[i]) : 0.0f);
            eff.push_back(i < effort.size()   ? static_cast<float>(effort[i])   : 0.0f);
            kp.push_back(default_kp(*index));
            kd.push_back(default_kd(*index));
        }

        return {indices, pos, vel, eff, kp, kd};
    }

} // namespace unitree_interface::joints

#endif // VECTOR_TOPOLOGY_HPP
