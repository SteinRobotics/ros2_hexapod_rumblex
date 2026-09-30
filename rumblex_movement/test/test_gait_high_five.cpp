#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "requester/gait_high_five.hpp"
#include "requester/kinematics.hpp"
#include "test_helpers.hpp"

using namespace rumblex_movement;

namespace {
constexpr int kMaxIterations = 200;
constexpr double kAngleTolerance = 1e-3;
constexpr double kHeadTolerance = 1e-3;
constexpr double kLiftThresholdDegrees = 5.0;
}  // namespace

class HighFiveGaitTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        rclcpp::NodeOptions options;
        auto overrides = test_helpers::defaultRobotParameters();
        options.parameter_overrides(overrides);

        node_ = std::make_shared<rclcpp::Node>("test_gait_high_five_node", options);
        kinematics_ = std::make_shared<CKinematics>(node_);
        params_ = test_helpers::makeDeclaredParameters(node_);

        // Move legs from default laydown to standing (gaits assume robot is standing)
        for (const auto& [idx, pos] : kinematics_->getStandingToePositions()) {
            kinematics_->setToePosition(idx, pos);
        }
    }

    void TearDown() override {
        kinematics_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    Parameters params_;
};

TEST_F(HighFiveGaitTest, RaisesRightFrontLegAndReturns) {
    CHighFiveGait gait(node_, kinematics_, params_.high_five);
    const auto initial_angles = kinematics_->getLegAngles(ELegIndex::RightFront);
    const auto initial_head = kinematics_->getHeadOrientation();

    gait.start(5.0, 0);

    bool raised = false;
    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        gait.update();
        const auto current_angles = kinematics_->getLegAngles(ELegIndex::RightFront);
        if (current_angles.coxa_femur >= initial_angles.coxa_femur + kLiftThresholdDegrees * units::deg) {
            raised = true;
        }
    }

    EXPECT_TRUE(raised);
    EXPECT_LT(iterations, kMaxIterations);

    const auto final_angles = kinematics_->getLegAngles(ELegIndex::RightFront);
    EXPECT_NEAR(final_angles.torso_coxa.numerical_value_in(units::deg),
                initial_angles.torso_coxa.numerical_value_in(units::deg), kAngleTolerance);
    EXPECT_NEAR(final_angles.coxa_femur.numerical_value_in(units::deg),
                initial_angles.coxa_femur.numerical_value_in(units::deg), kAngleTolerance);
    EXPECT_NEAR(final_angles.femur_tibia.numerical_value_in(units::deg),
                initial_angles.femur_tibia.numerical_value_in(units::deg), kAngleTolerance);

    const auto final_head = kinematics_->getHeadOrientation();
    EXPECT_NEAR(final_head.pitch.numerical_value_in(units::deg),
                initial_head.pitch.numerical_value_in(units::deg), kHeadTolerance);
    EXPECT_NEAR(final_head.yaw.numerical_value_in(units::deg),
                initial_head.yaw.numerical_value_in(units::deg), kHeadTolerance);
}

TEST_F(HighFiveGaitTest, RequestStopReturnsToNeutralQuickly) {
    CHighFiveGait gait(node_, kinematics_, params_.high_five);
    const auto initial_angles = kinematics_->getLegAngles(ELegIndex::RightFront);

    gait.start(5.0, 0);

    // Begin the raise phase for a few iterations.
    for (int i = 0; i < 3; ++i) {
        gait.update();
    }

    gait.requestStop();

    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        gait.update();
    }

    EXPECT_LT(iterations, kMaxIterations);

    const auto final_angles = kinematics_->getLegAngles(ELegIndex::RightFront);
    EXPECT_NEAR(final_angles.torso_coxa.numerical_value_in(units::deg),
                initial_angles.torso_coxa.numerical_value_in(units::deg), kAngleTolerance);
    EXPECT_NEAR(final_angles.coxa_femur.numerical_value_in(units::deg),
                initial_angles.coxa_femur.numerical_value_in(units::deg), kAngleTolerance);
    EXPECT_NEAR(final_angles.femur_tibia.numerical_value_in(units::deg),
                initial_angles.femur_tibia.numerical_value_in(units::deg), kAngleTolerance);
}
