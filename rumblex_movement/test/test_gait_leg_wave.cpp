#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "requester/gait_leg_wave.hpp"
#include "requester/kinematics.hpp"
#include "test_helpers.hpp"

using namespace rumblex_movement;

namespace {
constexpr int kMaxIterations = 400;
constexpr double kLegLiftHeight = 0.025;
constexpr double kTolerance = 1e-3;
}  // namespace

class LegWaveGaitTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        rclcpp::NodeOptions options;
        auto overrides = test_helpers::defaultRobotParameters();
        options.parameter_overrides(overrides);

        node_ = std::make_shared<rclcpp::Node>("test_gait_leg_wave_node", options);
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

TEST_F(LegWaveGaitTest, LiftOccursDuringRun) {
    CLegWaveGait gait(node_, kinematics_, params_.leg_wave);
    const auto standing = kinematics_->getStandingToePositions();

    gait.start(3.0, 0);

    bool seen_lift = false;
    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        gait.update();
        const auto pos = kinematics_->getToePositions();
        for (const auto& kv : standing) {
            const auto& idx = kv.first;
            const auto& base = kv.second;
            const auto& now = pos.at(idx);
            if (now.z >= base.z + (0.5 * kLegLiftHeight * units::m)) {
                seen_lift = true;
                break;
            }
        }
        if (seen_lift) break;
    }

    EXPECT_TRUE(seen_lift);
    EXPECT_LT(iterations, kMaxIterations);
}

TEST_F(LegWaveGaitTest, StopRequestReturnsToNeutral) {
    CLegWaveGait gait(node_, kinematics_, params_.leg_wave);
    const auto standing = kinematics_->getStandingToePositions();

    gait.start(3.0, 0);

    for (int i = 0; i < 3; ++i) {
        gait.update();
    }

    gait.requestStop();

    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        gait.update();
    }

    EXPECT_LT(iterations, kMaxIterations);

    const auto final_pos = kinematics_->getToePositions();
    for (const auto& kv : standing) {
        const auto& idx = kv.first;
        const auto& base = kv.second;
        const auto& now = final_pos.at(idx);
        EXPECT_NEAR(now.x.numerical_value_in(units::m), base.x.numerical_value_in(units::m), kTolerance);
        EXPECT_NEAR(now.y.numerical_value_in(units::m), base.y.numerical_value_in(units::m), kTolerance);
        EXPECT_NEAR(now.z.numerical_value_in(units::m), base.z.numerical_value_in(units::m),
                    1e-2);  // allow slightly larger tolerance for IK/clamping
    }
}
