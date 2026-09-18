#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "requester/gait_bodyroll.hpp"
#include "requester/kinematics.hpp"
#include "test_helpers.hpp"

using namespace rumblex_movement;

class BodyRollGaitTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        rclcpp::NodeOptions options;
        auto overrides = test_helpers::defaultRobotParameters();
        options.parameter_overrides(overrides);

        node_ = std::make_shared<rclcpp::Node>("test_gait_bodyroll_node", options);

        kinematics_ = std::make_shared<CKinematics>(node_);
        params_ = test_helpers::makeDeclaredParameters(node_);
        gait_ = std::make_unique<CGaitBodyRoll>(node_, kinematics_, params_.bodyRoll);
    }

    void TearDown() override {
        gait_.reset();
        kinematics_.reset();
        if (rclcpp::ok()) rclcpp::shutdown();
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    std::unique_ptr<CGaitBodyRoll> gait_;
    Parameters params_;
};

TEST_F(BodyRollGaitTest, StateTransitionsCoverAllStates) {
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

    // advance until Stopping
    iters = 0;
    while (gait_->state() != EGaitState::Stopping && ++iters < max_iters) {
        gait_->update();
    }
    EXPECT_EQ(gait_->state(), EGaitState::Stopping);
    EXPECT_LT(iters, max_iters);

    // advance until final Stopped
    iters = 0;
    while (gait_->state() != EGaitState::Stopped && ++iters < max_iters) {
        gait_->update();
    }
    EXPECT_EQ(gait_->state(), EGaitState::Stopped);
    EXPECT_LT(iters, max_iters);
}
