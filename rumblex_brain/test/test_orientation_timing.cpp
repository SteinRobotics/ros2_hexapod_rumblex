#include <gtest/gtest.h>

#include "gait/gait_continuous_pose.hpp"
#include "gait/gait_single_pose.hpp"
#include "gait/gait_torso_roll.hpp"
#include "gait/gait_yaw_sequence.hpp"
#include "movement_request.hpp"
#include "rumblex_utils/body_pose.hpp"
#include "test_helpers.hpp"

namespace {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using mp_units::angular::unit_symbols::deg;
using namespace brain;

class OrientationTimingTest : public ::testing::Test {
   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        rclcpp::NodeOptions options;
        options.parameter_overrides(test_helpers::defaultRobotParameters());
        node = std::make_shared<rclcpp::Node>("orientation_timing_test", options);
        model = std::make_shared<CPoseModel>(node);
        model->moveTorso(model->getStandingToePositions());
        params = Parameters::declare(node);
    }
    void TearDown() override {
        rclcpp::shutdown();
    }
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<CPoseModel> model;
    Parameters params;
};

TEST_F(OrientationTimingTest, PoseSamplingUsesElapsedSecondsAndHasSmallMonotonicSteps) {
    for (bool continuous : {false, true}) {
        for (bool jitter : {false, true}) {
            model->moveTorso(model->getStandingToePositions());
            model->setHeadOrientation(COrientation());
            std::unique_ptr<IContinuousGait> gait;
            if (continuous)
                gait = std::make_unique<CContinuousPoseGait>(node, model, params.continuous_pose);
            else {
                rumblex_interfaces::msg::BodyPose target;
                target.has_torso_pose = target.has_head_orientation = true;
                target.torso_pose.orientation.yaw = 20.0;
                target.head_orientation.pitch = 10.0;
                target.head_orientation.yaw = 20.0;
                gait = std::make_unique<CSinglePoseGait>(node, model, params.single_pose, target);
            }
            gait->start(1.0 * s, 0);
            const CPose torso(0.0, 0.0, 0.0, 0.0, 0.0, 20.0);
            const COrientation head(0.0, 10.0, 20.0);
            double previous = 0.0;
            for (int i = 0; i < 50; ++i) {
                const double dt = jitter ? (i % 2 ? 0.025 : 0.015) : 0.02;
                ASSERT_TRUE(gait->updateTimed(Velocity(), torso, head, dt * s));
                const auto pose = bodyPose(*model);
                const double yaw = pose.head_orientation.yaw;
                EXPECT_GE(yaw + 1e-12, previous);
                EXPECT_LT(yaw - previous, continuous ? 2.0 : 1.0);
                if (i == 24) {
                    EXPECT_NEAR(yaw, continuous ? 20.0 : 10.0, 0.25);
                }
                if (!continuous) {
                    EXPECT_NEAR(pose.torso_pose.orientation.yaw, yaw, 1e-10);
                }
                previous = yaw;
            }
            EXPECT_EQ(model->getHeadOrientation(), head);
            EXPECT_EQ(model->getTorsoPose(), torso);
        }
    }
}

TEST_F(OrientationTimingTest, ContinuousHeadAcceleratesSoonerAndHoldsUntilSegmentBoundary) {
    CContinuousPoseGait gait(node, model, params.continuous_pose);
    const CPose torso(0.0, 0.0, 0.0, 0.0, 0.0, 20.0);
    const COrientation head(0.0, 0.0, 20.0);
    const COrientation next_head(0.0, 0.0, -20.0);
    gait.start(1.0 * s, 0);
    ASSERT_TRUE(gait.updateTimed(Velocity(), torso, head, 0.1 * s));
    EXPECT_NEAR(bodyPose(*model).head_orientation.yaw, 1.1584, 1e-10);
    EXPECT_NEAR(bodyPose(*model).torso_pose.orientation.yaw, 0.1712, 1e-10);

    // A replacement target must not alter the captured trajectory.
    for (int i = 0; i < 4; ++i) ASSERT_TRUE(gait.updateTimed(Velocity(), torso, next_head, 0.1 * s));
    EXPECT_EQ(model->getHeadOrientation(), head);
    EXPECT_NEAR(bodyPose(*model).torso_pose.orientation.yaw, 10.0, 1e-10);
    gait.requestStop();
    for (int i = 0; i < 5; ++i) {
        ASSERT_TRUE(gait.updateTimed(Velocity(), torso, next_head, 0.1 * s));
        EXPECT_EQ(model->getHeadOrientation(), head);
        if (i < 4) {
            EXPECT_EQ(gait.state(), EGaitState::Stopping);
        }
    }
    EXPECT_EQ(model->getTorsoPose(), torso);
    EXPECT_EQ(gait.state(), EGaitState::Stopped);
}

TEST_F(OrientationTimingTest, YawAndTorsoSequencesKeepDurationAtHigherSamplingRate) {
    for (bool yaw : {false, true}) {
        std::unique_ptr<ISequenceGait> gait;
        model->moveTorso(model->getStandingToePositions());
        model->setHeadOrientation(COrientation());
        const auto origin = bodyPose(*model);
        if (yaw)
            gait = std::make_unique<CYawSequenceGait>(node, model, 30.0 * deg, 0.0 * deg);
        else
            gait = std::make_unique<CTorsoRollGait>(node, model, params.torso_roll);
        gait->start(1.0 * s, MovementRequest::CLOCKWISE);
        for (int i = 0; i < 50; ++i) {
            const double dt = i % 2 ? 0.025 : 0.015;
            ASSERT_TRUE(gait->updateTimed(dt * s));
            if (i == 12) {
                EXPECT_NE(bodyPose(*model), origin);
            }
            if (i < 49) {
                EXPECT_NE(gait->state(), EGaitState::Stopped);
            }
        }
        EXPECT_EQ(bodyPose(*model), origin);
        if (yaw)
            EXPECT_EQ(gait->state(), EGaitState::Stopped);
        else {
            gait->requestStop();
            for (int i = 0; i < 50; ++i) gait->updateTimed(0.02 * s);
            EXPECT_EQ(gait->state(), EGaitState::Stopped);
        }
    }
}

}  // namespace
