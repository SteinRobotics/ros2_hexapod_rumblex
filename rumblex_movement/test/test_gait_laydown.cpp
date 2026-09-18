#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "requester/gait_laydown.hpp"
#include "requester/gait_standup.hpp"
#include "requester/kinematics.hpp"
#include "test_helpers.hpp"

using namespace rumblex_movement;

namespace {
constexpr double kPositionTolerance = 0.06;
constexpr int kMaxIterations = 500;
}  // namespace

class GaitLayDownTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        rclcpp::NodeOptions options;
        auto overrides = test_helpers::defaultRobotParameters();
        options.parameter_overrides(overrides);

        node_ = std::make_shared<rclcpp::Node>("test_gait_vertical_node", options);
        kinematics_ = std::make_shared<CKinematics>(node_);
        params_ = test_helpers::makeDeclaredParameters(node_);
    }

    void TearDown() override {
        kinematics_.reset();
        if (rclcpp::ok()) rclcpp::shutdown();
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    Parameters params_;
};

TEST_F(GaitLayDownTest, LayDownStopsAtLaydownHeight) {
    const auto laydownTargets = kinematics_->getLegsLayDownPositions();
    const auto standingTargets = kinematics_->getLegsStandingPositions();

    // Ensure we start from standing pose.
    kinematics_->moveBody(standingTargets, CPose());
    const auto initialPositions = kinematics_->getLegsPositions();

    CLayDownGait gait(node_, kinematics_, params_.layDown);
    EXPECT_EQ(gait.state(), EGaitState::Stopped);

    gait.start(3.0, 0);

    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        gait.update();
    }

    EXPECT_EQ(gait.state(), EGaitState::Stopped);
    EXPECT_LT(iterations, kMaxIterations);

    const auto finalPositions = kinematics_->getLegsPositions();
    for (const auto& [legIndex, target] : laydownTargets) {
        const auto& actual = finalPositions.at(legIndex);
        EXPECT_NEAR(actual.z, target.z, kPositionTolerance);
    }

    /// manuall check forward kinematics by setting angles for front legs
    CLegAngles angles;
    angles.coxa_deg = 0.0;
    angles.femur_deg = 70.0;
    angles.tibia_deg = -53.0;
    kinematics_->setLegAngles(ELegIndex::RightFront, angles);
    kinematics_->setLegAngles(ELegIndex::RightMid, angles);
}
