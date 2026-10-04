#include <gtest/gtest.h>

#include "gait/gait_torso_roll.hpp"
#include "gait/pose_model.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_helpers.hpp"

using namespace brain;

class TorsoRollGaitTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        rclcpp::NodeOptions options;
        auto overrides = test_helpers::defaultRobotParameters();
        options.parameter_overrides(overrides);

        node_ = std::make_shared<rclcpp::Node>("test_gait_torso_roll_node", options);

        kinematics_ = std::make_shared<CPoseModel>(node_);
        params_ = test_helpers::makeDeclaredParameters(node_);
        gait_ = std::make_unique<CTorsoRollGait>(node_, kinematics_, params_.torso_roll);
    }

    void TearDown() override {
        gait_.reset();
        kinematics_.reset();
        if (rclcpp::ok()) rclcpp::shutdown();
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CPoseModel> kinematics_;
    std::unique_ptr<CTorsoRollGait> gait_;
    Parameters params_;
};

TEST_F(TorsoRollGaitTest, StateTransitionsCoverAllStates) {
    // Initial state should be Stopped
    EXPECT_EQ(gait_->state(), EGaitState::Stopped);

    // Start -> Starting
    gait_->start(1.0, 0);
    EXPECT_EQ(gait_->state(), EGaitState::Starting);

    // update until Running
    int max_iters = 1000;
    int iters = 0;
    while (gait_->state() != EGaitState::Running && ++iters < max_iters) {
        gait_->update();
    }
    EXPECT_EQ(gait_->state(), EGaitState::Running);
    EXPECT_LT(iters, max_iters);

    // requestStop -> StopPending
    gait_->requestStop();
    EXPECT_EQ(gait_->state(), EGaitState::StopPending);

    // cancelStop should return to Running
    gait_->cancelStop();
    EXPECT_EQ(gait_->state(), EGaitState::Running);

    // requestStop again and let it progress to Stopping and Stopped
    gait_->requestStop();
    EXPECT_EQ(gait_->state(), EGaitState::StopPending);

    // advance until final Stopped
    iters = 0;
    while (gait_->state() != EGaitState::Stopped && ++iters < max_iters) {
        gait_->update();
    }
    EXPECT_EQ(gait_->state(), EGaitState::Stopped);
    EXPECT_LT(iters, max_iters);
}

TEST_F(TorsoRollGaitTest, StopDuringStartupCanBeCancelled) {
    gait_->start(1.0, 0);
    gait_->requestStop();
    gait_->requestStop();
    ASSERT_EQ(gait_->state(), EGaitState::StopPending);
    gait_->cancelStop();
    EXPECT_EQ(gait_->state(), EGaitState::Starting);
    EXPECT_TRUE(gait_->update());
}

TEST_F(TorsoRollGaitTest, StopDuringStartupFinishesAtNeutralTorso) {
    gait_->start(1.0, 0);
    gait_->requestStop();
    ASSERT_EQ(gait_->state(), EGaitState::StopPending);
    for (int i = 0; i < 1000 && gait_->state() != EGaitState::Stopped; ++i) gait_->update();
    EXPECT_EQ(gait_->state(), EGaitState::Stopped);
    EXPECT_EQ(kinematics_->getTorsoPose(), CPose());
}

TEST_F(TorsoRollGaitTest, StopWaitsForClosedCycleWithoutChangingPoseOnRequest) {
    gait_->start(2.0, 0);
    gait_->update();
    const auto before = kinematics_->getTorsoPose();
    gait_->requestStop();
    EXPECT_EQ(kinematics_->getTorsoPose(), before);
    for (int i = 0; i < 100 && gait_->state() != EGaitState::Stopped; ++i) gait_->update();
    EXPECT_EQ(gait_->state(), EGaitState::Stopped);
    EXPECT_EQ(kinematics_->getTorsoPose(), CPose());
}
