#include <gtest/gtest.h>

#include "unitree_interface/control_modes.hpp"
#include "unitree_interface/mode_transitions.hpp"
#include "unitree_interface_msgs/msg/control_mode.hpp"

namespace ui = unitree_interface;

// ========== Transition legality tests ==========
// ========== std::monostate ==========
TEST(TransitionLegalityTest, FromMonostate_ToMonostate_NotAllowed) {
    EXPECT_FALSE(ui::can_transition<std::monostate>(ui::ControlMode{std::monostate{}}));
}

TEST(TransitionLegalityTest, FromMonostate_ToIdle_Allowed) {
    EXPECT_TRUE(ui::can_transition<ui::IdleMode>(ui::ControlMode{std::monostate{}}));
}

TEST(TransitionLegalityTest, FromMonostate_ToHighLevel_NotAllowed) {
    EXPECT_FALSE(ui::can_transition<ui::HighLevelMode>(ui::ControlMode{std::monostate{}}));
}

TEST(TransitionLegalityTest, FromMonostate_ToLowLevel_NotAllowed) {
    EXPECT_FALSE(ui::can_transition<ui::LowLevelMode>(ui::ControlMode{std::monostate{}}));
}

TEST(TransitionLegalityTest, FromMonostate_ToEmergency_Allowed) {
    EXPECT_TRUE(ui::can_transition<ui::EmergencyMode>(ui::ControlMode{std::monostate{}}));
}

// ========== ui::IdleMode ==========
TEST(TransitionLegalityTest, FromIdle_ToMonostate_NotAllowed) {
    EXPECT_FALSE(ui::can_transition<std::monostate>(ui::ControlMode{ui::IdleMode{}}));
}

TEST(TransitionLegalityTest, FromIdle_ToIdle_NotAllowed) {
    EXPECT_FALSE(ui::can_transition<ui::IdleMode>(ui::ControlMode{ui::IdleMode{}}));
}

TEST(TransitionLegalityTest, FromIdle_ToHighLevel_Allowed) {
    EXPECT_TRUE(ui::can_transition<ui::HighLevelMode>(ui::ControlMode{ui::IdleMode{}}));
}

TEST(TransitionLegalityTest, FromIdle_ToLowLevel_Allowed) {
    EXPECT_TRUE(ui::can_transition<ui::LowLevelMode>(ui::ControlMode{ui::IdleMode{}}));
}

TEST(TransitionLegalityTest, FromIdle_ToEmergency_Allowed) {
    EXPECT_TRUE(ui::can_transition<ui::EmergencyMode>(ui::ControlMode{ui::IdleMode{}}));
}

// ========== ui::HighLevelMode ==========
TEST(TransitionLegalityTest, FromHighLevel_ToMonostate_NotAllowed) {
    EXPECT_FALSE(ui::can_transition<std::monostate>(ui::ControlMode{ui::HighLevelMode{}}));
}

TEST(TransitionLegalityTest, FromHighLevel_ToIdle_Allowed) {
    EXPECT_TRUE(ui::can_transition<ui::IdleMode>(ui::ControlMode{ui::HighLevelMode{}}));
}

TEST(TransitionLegalityTest, FromHighLevel_ToHighLevel_NotAllowed) {
    EXPECT_FALSE(ui::can_transition<ui::HighLevelMode>(ui::ControlMode{ui::HighLevelMode{}}));
}

TEST(TransitionLegalityTest, FromHighLevel_ToLowLevel_Allowed) {
    EXPECT_TRUE(ui::can_transition<ui::LowLevelMode>(ui::ControlMode{ui::HighLevelMode{}}));
}

TEST(TransitionLegalityTest, FromHighLevel_ToEmergency_Allowed) {
    EXPECT_TRUE(ui::can_transition<ui::EmergencyMode>(ui::ControlMode{ui::HighLevelMode{}}));
}

// TODO: Add tests for hybrid mode once it's ready

// ========== ui::LowLevelMode ==========
TEST(TransitionLegalityTest, FromLowLevel_ToMonostate_NotAllowed) {
    EXPECT_FALSE(ui::can_transition<std::monostate>(ui::ControlMode{ui::LowLevelMode{}}));
}

TEST(TransitionLegalityTest, FromLowLevel_ToIdle_Allowed) {
    EXPECT_TRUE(ui::can_transition<ui::IdleMode>(ui::ControlMode{ui::LowLevelMode{}}));
}

TEST(TransitionLegalityTest, FromLowLevel_ToHighLevel_Allowed) {
    EXPECT_TRUE(ui::can_transition<ui::HighLevelMode>(ui::ControlMode{ui::LowLevelMode{}}));
}

TEST(TransitionLegalityTest, FromLowLevel_ToLowLevel_NotAllowed) {
    EXPECT_FALSE(ui::can_transition<ui::LowLevelMode>(ui::ControlMode{ui::LowLevelMode{}}));
}

TEST(TransitionLegalityTest, FromLowLevel_ToEmergency_Allowed) {
    EXPECT_TRUE(ui::can_transition<ui::EmergencyMode>(ui::ControlMode{ui::LowLevelMode{}}));
}

// ========== ui::EmergencyMode ==========
TEST(TransitionLegalityTest, FromEmergency_ToMonostate_NotAllowed) {
    EXPECT_FALSE(ui::can_transition<std::monostate>(ui::ControlMode{ui::EmergencyMode{}}));
}

TEST(TransitionLegalityTest, FromEmergency_ToIdle_NotAllowed) {
    EXPECT_FALSE(ui::can_transition<ui::IdleMode>(ui::ControlMode{ui::EmergencyMode{}}));
}

TEST(TransitionLegalityTest, FromEmergency_ToHighLevel_NotAllowed) {
    EXPECT_FALSE(ui::can_transition<ui::HighLevelMode>(ui::ControlMode{ui::EmergencyMode{}}));
}

TEST(TransitionLegalityTest, FromEmergency_ToLowLevel_NotAllowed) {
    EXPECT_FALSE(ui::can_transition<ui::LowLevelMode>(ui::ControlMode{ui::EmergencyMode{}}));
}

TEST(TransitionLegalityTest, FromEmergency_ToEmergency_NotAllowed) {
    EXPECT_FALSE(ui::can_transition<ui::EmergencyMode>(ui::ControlMode{ui::EmergencyMode{}}));
}

// ========== Runtime transition tests ==========
class TryTransitionToTest : public ::testing::Test {
protected:
    // Uninitialized wrapper - can only test invalid ID handling
    // Valid IDs will fail due to uninitialized wrapper, not dispatch logic
    rclcpp::Logger logger_ = rclcpp::get_logger("test");
};

TEST_F(TryTransitionToTest, InvalidIDReturnsFailure) {
    ui::UnitreeSDKWrapper wrapper("eth0", logger_);
    ui::ControlMode from{ui::IdleMode{}};

    constexpr std::uint8_t invalid_id = 255;
    auto [new_mode, success] = ui::try_transition_to(from, invalid_id, wrapper);

    EXPECT_FALSE(success);
    EXPECT_TRUE(std::holds_alternative<ui::IdleMode>(new_mode));
}

TEST_F(TryTransitionToTest, MonostateIDReturnsFailure) {
    ui::UnitreeSDKWrapper wrapper("eth0", logger_);
    ui::ControlMode from{ui::IdleMode{}};

    // Transitioning to monostate is never valid
    auto [new_mode, success] = ui::try_transition_to(
        from,
        unitree_interface_msgs::msg::ControlMode::CONTROL_MODE_MONOSTATE,
        wrapper
    );

    EXPECT_FALSE(success);
    EXPECT_TRUE(std::holds_alternative<ui::IdleMode>(new_mode));
}

TEST_F(TryTransitionToTest, EmergencyIDNotInSwitchReturnsFailure) {
    ui::UnitreeSDKWrapper wrapper("eth0", logger_);
    ui::ControlMode from{ui::IdleMode{}};

    // Emergency is not in the switch (intentionally - use estop)
    auto [new_mode, success] = ui::try_transition_to(
        from,
        unitree_interface_msgs::msg::ControlMode::CONTROL_MODE_EMERGENCY,
        wrapper
    );

    EXPECT_FALSE(success);
    EXPECT_TRUE(std::holds_alternative<ui::IdleMode>(new_mode));
}
