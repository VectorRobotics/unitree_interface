#ifndef VECTOR_CONTROL_MODES_HPP
#define VECTOR_CONTROL_MODES_HPP

#include <cstdint>
#include <string>
#include <type_traits>
#include <variant>

namespace unitree_interface {

    class UnitreeSDKWrapper;

#ifdef UNITREE_INTERFACE_TESTING_ENABLED
    namespace testing {
        struct ControlModeFactory;
    }
#endif

    // ========== Control modes ==========
    class IdleMode {
    private:
        friend class UnitreeSDKWrapper;

#ifdef UNITREE_INTERFACE_TESTING_ENABLED
        friend struct testing::ControlModeFactory;
#endif

        /*
        Aggregate intilization can bypass private constructors. This was changed in
        C++20, but since we're using C++17, we need to mark the default constructors
        as explicit (and, therefore, make it a non-aggregate type) to disable this.
        */
        explicit IdleMode() = default;
    };

    // ====================
    class HighLevelMode {
    public:
        void send_velocity_command(
            UnitreeSDKWrapper& sdk_wrapper,
            float vx,
            float vy,
            float vyaw
        );

        void send_speech_command(
            UnitreeSDKWrapper& sdk_wrapper,
            const std::string& message
        );

    private:
        friend class UnitreeSDKWrapper;

#ifdef UNITREE_INTERFACE_TESTING_ENABLED
        friend struct testing::ControlModeFactory;
#endif

        explicit HighLevelMode() = default;
    };

    // ====================
//     class HybridMode {
//     public:
//         // TODO: Add public capabilities

//     private:
//         friend class UnitreeSDKWrapper;

// #ifdef UNITREE_INTERFACE_TESTING_ENABLED
//         friend struct testing::ControlModeFactory;
// #endif

//         HybridMode() = default;
//     };

    // ====================
    class LowLevelMode {
    public:
        void set_joint_motor_gains(
            UnitreeSDKWrapper& sdk_wrapper
        );

        void send_joint_control_command(
            UnitreeSDKWrapper& sdk_wrapper
        );

    private:
        friend class UnitreeSDKWrapper;

#ifdef UNITREE_INTERFACE_TESTING_ENABLED
        friend struct testing::ControlModeFactory;
#endif

        explicit LowLevelMode() = default;
    };

    // ====================
    class EmergencyMode {
    public:
        bool damp(UnitreeSDKWrapper& sdk_wrapper);

    private:
        friend class UnitreeSDKWrapper;

#ifdef UNITREE_INTERFACE_TESTING_ENABLED
        friend struct testing::ControlModeFactory;
#endif

        explicit EmergencyMode() = default;
    };

    // ====================
    // clang-format off
    using ControlMode = std::variant<
        std::monostate,
        IdleMode,
        HighLevelMode,
        // HybridMode,
        LowLevelMode,
        EmergencyMode
    >;
    // clang-format on

    // ========== Control mode traits ==========
    enum class ControlModeID : std::uint8_t {
        monostate = 0,
        Idle      = 1,
        HighLevel = 2,
        // Hybrid    = 3,
        LowLevel  = 3,
        Emergency = 4
    };

    template <typename T>
    struct ControlModeTraits;

    template <>
    struct ControlModeTraits<std::monostate> {
        static constexpr ControlModeID id = ControlModeID::monostate;

        static constexpr const char* name() { return "std::monostate"; }
    };

    template <>
    struct ControlModeTraits<IdleMode> {
        static constexpr ControlModeID id = ControlModeID::Idle;

        static constexpr const char* name() { return "Idle"; }
    };

    template <>
    struct ControlModeTraits<HighLevelMode> {
        static constexpr ControlModeID id = ControlModeID::HighLevel;

        static constexpr const char* name() { return "HighLevel"; }
    };

    // template <>
    // struct ControlModeTraits<HybridMode> {
    //     static constexpr ControlModeID id = ControlModeID::Hybrid;

    //     static constexpr const char* name() { return "Hybrid"; }
    // };

    template <>
    struct ControlModeTraits<LowLevelMode> {
        static constexpr ControlModeID id = ControlModeID::LowLevel;

        static constexpr const char* name() { return "LowLevel"; }
    };

    template <>
    struct ControlModeTraits<EmergencyMode> {
        static constexpr ControlModeID id = ControlModeID::Emergency;

        static constexpr const char* name() { return "Emergency"; }
    };

    template <typename... Ts>
    struct always_false : std::false_type {};

    // ========== Helper functions ==========
    bool can_transition(const ControlMode& from, const ControlMode& to);

    ControlMode execute_transition(
        const ControlMode& from,
        const ControlMode& to,
        UnitreeSDKWrapper& sdk_wrapper
    );

} // namespace unitree_interface

#endif // VECTOR_CONTROL_MODES_HPP