#include <gtest/gtest.h>

#include "gait/gait_continuous_pose.hpp"
#include "gait/gait_single_pose.hpp"
#include "gait/gait_torso_roll.hpp"
#include "gait/gait_yaw_sequence.hpp"
#include "movement_request.hpp"
#include "rumblex_utils/body_pose.hpp"
#include "test_helpers.hpp"

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
            else
                gait = std::make_unique<CSinglePoseGait>(node, model, params.single_pose);
            gait->start(1.0 * units::s, 0);
            const CPose torso(0.0, 0.0, 0.0, 0.0, 0.0, 20.0);
            const COrientation head(0.0, 10.0, 20.0);
            double previous = 0.0;
            for (int i = 0; i < 50; ++i) {
                const double dt = jitter ? (i % 2 ? 0.025 : 0.015) : 0.02;
                ASSERT_TRUE(gait->updateTimed(Velocity(), torso, head, dt * units::s));
                const auto pose = bodyPose(*model);
                const double yaw = pose.head_pose.yaw;
                EXPECT_GE(yaw + 1e-12, previous);
                EXPECT_LT(yaw - previous, 1.0);
                if (i == 24) {
                    EXPECT_NEAR(yaw, 10.0, 0.25);
                }
                EXPECT_NEAR(pose.torso_pose.orientation.yaw, yaw, 1e-10);
                previous = yaw;
            }
            EXPECT_EQ(model->getHeadOrientation(), head);
            EXPECT_EQ(model->getTorsoPose(), torso);
        }
    }
}

TEST_F(OrientationTimingTest, YawAndTorsoSequencesKeepDurationAtHigherSamplingRate) {
    for (bool yaw : {false, true}) {
        std::unique_ptr<ISequenceGait> gait;
        model->moveTorso(model->getStandingToePositions());
        model->setHeadOrientation(COrientation());
        const auto origin = bodyPose(*model);
        if (yaw)
            gait = std::make_unique<CYawSequenceGait>(node, model, CYawSequenceGait::Sweep::OneSide,
                                                      25.0 * units::deg, 20.0 * units::deg);
        else
            gait = std::make_unique<CTorsoRollGait>(node, model, params.torso_roll);
        gait->start(1.0 * units::s, MovementRequest::CLOCKWISE);
        for (int i = 0; i < 50; ++i) {
            const double dt = i % 2 ? 0.025 : 0.015;
            ASSERT_TRUE(gait->updateTimed(dt * units::s));
            if (i == 24) {
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
            for (int i = 0; i < 50; ++i) gait->updateTimed(0.02 * units::s);
            EXPECT_EQ(gait->state(), EGaitState::Stopped);
        }
    }
}
