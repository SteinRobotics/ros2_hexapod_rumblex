#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "requester/gait_clap.hpp"
#include "requester/kinematics.hpp"
#include "test_helpers.hpp"

using namespace rumblex_movement;

namespace {
constexpr int kMaxIterations = 500;
constexpr double kAngleTolerance = 1e-2;
constexpr double kPositionTolerance = 1e-3;
}  // namespace

class ClapGaitTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        rclcpp::NodeOptions options;
        auto overrides = test_helpers::defaultRobotParameters();
        options.parameter_overrides(overrides);

        node_ = std::make_shared<rclcpp::Node>("test_gait_clap_node", options);
        kinematics_ = std::make_shared<CKinematics>(node_);
        params_ = test_helpers::makeDeclaredParameters(node_);
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

TEST_F(ClapGaitTest, CompletesCycleAndReturnsToInitialPose) {
    CClapGait gait(node_, kinematics_, params_.clap);
    const auto initialBody = kinematics_->getBody();
    const auto initialPositions = kinematics_->getLegsPositions();

    gait.start(3.0, 0);

    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        gait.update();
    }

    EXPECT_LT(iterations, kMaxIterations) << "Gait should complete within max iterations";
    EXPECT_EQ(gait.state(), EGaitState::Stopped);

    // Verify body position returned to initial state
    const auto finalBody = kinematics_->getBody();
    EXPECT_NEAR(finalBody.position.x, initialBody.position.x, kPositionTolerance);
    EXPECT_NEAR(finalBody.position.y, initialBody.position.y, kPositionTolerance);
    EXPECT_NEAR(finalBody.position.z, initialBody.position.z, kPositionTolerance);
}

TEST_F(ClapGaitTest, BackLegsLiftDuringSequence) {
    CClapGait gait(node_, kinematics_, params_.clap);
    const auto initialPositions = kinematics_->getLegsPositions();

    gait.start(3.0, 0);

    bool rightBackLifted = false;
    bool leftBackLifted = false;

    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        gait.update();

        const auto currentPositions = kinematics_->getLegsPositions();

        // Check if right back leg was lifted
        if (currentPositions.at(ELegIndex::RightBack).z >
            initialPositions.at(ELegIndex::RightBack).z + 0.01) {
            rightBackLifted = true;
        }

        // Check if left back leg was lifted
        if (currentPositions.at(ELegIndex::LeftBack).z > initialPositions.at(ELegIndex::LeftBack).z + 0.01) {
            leftBackLifted = true;
        }
    }

    EXPECT_TRUE(rightBackLifted) << "Right back leg should lift during clap sequence";
    EXPECT_TRUE(leftBackLifted) << "Left back leg should lift during clap sequence";
}

TEST_F(ClapGaitTest, FrontLegsPerformClapMovement) {
    CClapGait gait(node_, kinematics_, params_.clap);
    const auto initialLeftAngles = kinematics_->getAngles(ELegIndex::LeftFront);
    const auto initialRightAngles = kinematics_->getAngles(ELegIndex::RightFront);

    gait.start(3.0, 0);

    bool frontLegsMovedForClap = false;

    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        gait.update();

        const auto currentLeftAngles = kinematics_->getAngles(ELegIndex::LeftFront);
        const auto currentRightAngles = kinematics_->getAngles(ELegIndex::RightFront);

        // Check if front legs moved their coxa angles for clapping
        double leftDiff = std::abs(currentLeftAngles.coxa_deg - initialLeftAngles.coxa_deg);
        double rightDiff = std::abs(currentRightAngles.coxa_deg - initialRightAngles.coxa_deg);

        if (leftDiff > 5.0 || rightDiff > 5.0) {
            frontLegsMovedForClap = true;
        }
    }

    EXPECT_TRUE(frontLegsMovedForClap) << "Front legs should move for clapping";
}

TEST_F(ClapGaitTest, RequestStopReturnsToInitialState) {
    CClapGait gait(node_, kinematics_, params_.clap);
    const auto initialBody = kinematics_->getBody();

    gait.start(3.0, 0);

    // Run for a few iterations
    for (int i = 0; i < 10; ++i) {
        gait.update();
    }

    // Request stop
    gait.requestStop();

    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        gait.update();
    }

    EXPECT_LT(iterations, kMaxIterations) << "Gait should stop within max iterations";
    EXPECT_EQ(gait.state(), EGaitState::Stopped);

    // Verify body returned to initial position
    const auto finalBody = kinematics_->getBody();
    EXPECT_NEAR(finalBody.position.x, initialBody.position.x, kPositionTolerance);
}
