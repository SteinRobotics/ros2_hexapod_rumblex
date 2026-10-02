#include <gtest/gtest.h>

#include "gait/gait_lay_down.hpp"
#include "gait/gait_stand_up.hpp"
#include "gait/pose_model.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_helpers.hpp"

using namespace brain;

namespace {
constexpr double kPositionTolerance = 0.06;
constexpr int kMaxIterations = 500;
}  // namespace

class GaitStandUpTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        rclcpp::NodeOptions options;
        auto overrides = test_helpers::defaultRobotParameters();
        options.parameter_overrides(overrides);

        node_ = std::make_shared<rclcpp::Node>("test_gait_vertical_node", options);
        kinematics_ = std::make_shared<CPoseModel>(node_);
        params_ = test_helpers::makeDeclaredParameters(node_);
    }

    void TearDown() override {
        kinematics_.reset();
        if (rclcpp::ok()) rclcpp::shutdown();
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CPoseModel> kinematics_;
    Parameters params_;
};

TEST_F(GaitStandUpTest, StandUpStopsAtStandingHeight) {
    const auto laydown_targets = kinematics_->getLaydownToePositions();
    const auto standing_targets = kinematics_->getStandingToePositions();

    // Ensure we start from laydown pose.
    kinematics_->moveTorso(laydown_targets, CPose());
    const auto initial_positions = kinematics_->getToePositions();

    CStandUpGait gait(node_, kinematics_, params_.stand_up);
    EXPECT_EQ(gait.state(), EGaitState::Stopped);

    gait.start(3.0, 0);

    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        gait.update();
    }

    EXPECT_EQ(gait.state(), EGaitState::Stopped);
    EXPECT_LT(iterations, kMaxIterations);

    const auto final_positions = kinematics_->getToePositions();
    for (const auto& [leg_index, target] : standing_targets) {
        const auto& actual = final_positions.at(leg_index);
        EXPECT_NEAR(actual.z.numerical_value_in(units::m), target.z.numerical_value_in(units::m),
                    kPositionTolerance);
    }
}
