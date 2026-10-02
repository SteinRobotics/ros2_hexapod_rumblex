#include <gtest/gtest.h>

#include "gait/gait_clap.hpp"
#include "gait/pose_model.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_helpers.hpp"

using namespace brain;

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
        kinematics_ = std::make_shared<CPoseModel>(node_);
        params_ = test_helpers::makeDeclaredParameters(node_);
    }

    void TearDown() override {
        kinematics_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CPoseModel> kinematics_;
    Parameters params_;
};

TEST_F(ClapGaitTest, CompletesCycleAndReturnsToInitialPose) {
    CClapGait gait(node_, kinematics_, params_.clap);
    const auto initial_torso = kinematics_->getTorsoPose();
    const auto initial_positions = kinematics_->getToePositions();

    gait.start(3.0, 0);

    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        gait.update();
    }

    EXPECT_LT(iterations, kMaxIterations) << "Gait should complete within max iterations";
    EXPECT_EQ(gait.state(), EGaitState::Stopped);

    // Verify torso position returned to initial state
    const auto final_torso = kinematics_->getTorsoPose();
    EXPECT_NEAR(final_torso.position.x.numerical_value_in(units::m),
                initial_torso.position.x.numerical_value_in(units::m), kPositionTolerance);
    EXPECT_NEAR(final_torso.position.y.numerical_value_in(units::m),
                initial_torso.position.y.numerical_value_in(units::m), kPositionTolerance);
    EXPECT_NEAR(final_torso.position.z.numerical_value_in(units::m),
                initial_torso.position.z.numerical_value_in(units::m), kPositionTolerance);
}

TEST_F(ClapGaitTest, BackLegsLiftDuringSequence) {
    CClapGait gait(node_, kinematics_, params_.clap);
    const auto initial_positions = kinematics_->getToePositions();

    gait.start(3.0, 0);

    bool right_back_lifted = false;
    bool left_back_lifted = false;

    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        gait.update();

        const auto current_positions = kinematics_->getToePositions();

        // Check if right back leg was lifted
        if (current_positions.at(ELegIndex::RightBack).z >
            initial_positions.at(ELegIndex::RightBack).z + 0.01 * units::m) {
            right_back_lifted = true;
        }

        // Check if left back leg was lifted
        if (current_positions.at(ELegIndex::LeftBack).z >
            initial_positions.at(ELegIndex::LeftBack).z + 0.01 * units::m) {
            left_back_lifted = true;
        }
    }

    EXPECT_TRUE(right_back_lifted) << "Right back leg should lift during clap sequence";
    EXPECT_TRUE(left_back_lifted) << "Left back leg should lift during clap sequence";
}

TEST_F(ClapGaitTest, FrontLegsPerformClapMovement) {
    CClapGait gait(node_, kinematics_, params_.clap);
    const auto initial_left_angles = kinematics_->getLegAngles(ELegIndex::LeftFront);
    const auto initial_right_angles = kinematics_->getLegAngles(ELegIndex::RightFront);

    gait.start(3.0, 0);

    bool front_legs_moved_for_clap = false;

    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        gait.update();

        const auto current_left_angles = kinematics_->getLegAngles(ELegIndex::LeftFront);
        const auto current_right_angles = kinematics_->getLegAngles(ELegIndex::RightFront);

        // Check if front legs moved their torso_coxa angles for clapping
        const auto left_diff = mp_units::abs(current_left_angles.torso_coxa - initial_left_angles.torso_coxa);
        const auto right_diff =
            mp_units::abs(current_right_angles.torso_coxa - initial_right_angles.torso_coxa);

        if (left_diff > 5.0 * units::deg || right_diff > 5.0 * units::deg) {
            front_legs_moved_for_clap = true;
        }
    }

    EXPECT_TRUE(front_legs_moved_for_clap) << "Front legs should move for clapping";
}

TEST_F(ClapGaitTest, RequestStopReturnsToInitialState) {
    CClapGait gait(node_, kinematics_, params_.clap);
    const auto initial_torso = kinematics_->getTorsoPose();

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

    // Verify torso returned to initial position
    const auto final_torso = kinematics_->getTorsoPose();
    EXPECT_NEAR(final_torso.position.x.numerical_value_in(units::m),
                initial_torso.position.x.numerical_value_in(units::m), kPositionTolerance);
}
