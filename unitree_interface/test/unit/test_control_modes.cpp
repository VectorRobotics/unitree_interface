#include <gtest/gtest.h>

#include "unitree_interface/control_modes.hpp"
#include "unitree_interface_msgs/msg/control_mode.hpp"

namespace ui = unitree_interface;

TEST(ControlModeTraitsTest, IdsMatchMessageConstants) {
    EXPECT_EQ(
        ui::ControlModeTraits<std::monostate>::id,
        unitree_interface_msgs::msg::ControlMode::CONTROL_MODE_MONOSTATE
    );

    EXPECT_EQ(
        ui::ControlModeTraits<ui::IdleMode>::id,
        unitree_interface_msgs::msg::ControlMode::CONTROL_MODE_IDLE
    );

    EXPECT_EQ(
        ui::ControlModeTraits<ui::HighLevelMode>::id,
        unitree_interface_msgs::msg::ControlMode::CONTROL_MODE_HIGH_LEVEL
    );

    // TODO: Enable when HybridMode is ready
    // EXPECT_EQ(
    //     ui::ControlModeTraits<ui::HybridMode>::id,
    //     unitree_interface_msgs::msg::ControlMode::CONTROL_MODE_HYBRID
    // );

#ifdef UNITREE_INTERFACE_ENABLE_LOW_LEVEL_MODE
    EXPECT_EQ(
        ui::ControlModeTraits<ui::LowLevelMode>::id,
        unitree_interface_msgs::msg::ControlMode::CONTROL_MODE_LOW_LEVEL
    );
#endif

    EXPECT_EQ(
        ui::ControlModeTraits<ui::EmergencyMode>::id,
        unitree_interface_msgs::msg::ControlMode::CONTROL_MODE_EMERGENCY
    );
}
