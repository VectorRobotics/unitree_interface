#include "unitree_interface/control_modes.hpp"

#include "unitree_interface/mode_transitions.hpp"
#include "unitree_interface/unitree_sdk_wrapper.hpp"

#include <type_traits>

namespace unitree_interface {

    // ========== HighLevelMode ==========

    // ========== LowLevelMode ==========

    // ========== EmergencyStop ==========
    bool EmergencyStop::execute(UnitreeSDKWrapper& sdk_wrapper) {
        return sdk_wrapper.emergency_stop();
    }

    // Runtime helpers
    bool can_transition(const ControlMode& from, const ControlMode& to) {
        return std::visit(
            [](auto&& f, auto&& t) -> bool {
                using FromType = std::decay_t<decltype(f)>;
                using ToType = std::decay_t<decltype(t)>;

                return Transition<FromType, ToType>::allowed;
            },
            from, to
        );
    }

    // TODO: Handle transition errors
    ControlMode execute_transition(
        const ControlMode& from,
        const ControlMode& to,
        UnitreeSDKWrapper& sdk_wrapper
    ) {
        if (!sdk_wrapper.is_initialized()) {
            return from;
        }

        return std::visit(
            [&](auto&& f, auto&& t) -> ControlMode {
                using FromType = std::decay_t<decltype(f)>;
                using ToType = std::decay_t<decltype(t)>;

                if constexpr (Transition<FromType, ToType>::allowed) {
                    return Transition<FromType, ToType>::execute(sdk_wrapper);
                }

                // Transition not allowed or failed
                return from;
            },
            from, to
        );
    }

} // namespace unitree_interface