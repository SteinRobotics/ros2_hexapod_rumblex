#include <gtest/gtest.h>

#include "gait/pose_model.hpp"
#include "rumblex_utils/body_pose.hpp"
#include "test_helpers.hpp"

using namespace brain;
class BodyPoseTest : public ::testing::Test {
   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        rclcpp::NodeOptions options;
        options.parameter_overrides(test_helpers::defaultRobotParameters());
        source_node = std::make_shared<rclcpp::Node>("source_pose", options);
        receiver_node = std::make_shared<rclcpp::Node>("receiver_pose", options);
        source = std::make_unique<CPoseModel>(source_node);
        receiver = std::make_unique<CPoseModel>(receiver_node);
    }
    void TearDown() override {
        rclcpp::shutdown();
    }
    std::shared_ptr<rclcpp::Node> source_node, receiver_node;
    std::unique_ptr<CPoseModel> source, receiver;
};
TEST_F(BodyPoseTest, FullPoseRoundTripUsesMetresDegreesAndStableLegOrder) {
    CPose torso(0.005, -0.003, 0.002, 3.0, -2.0, 5.0);
    source->moveTorso(source->getStandingToePositions(), torso);
    source->setHeadOrientation(COrientation(1.0, 12.0, -23.0));
    auto msg = bodyPose(*source);
    EXPECT_DOUBLE_EQ(msg.head_pose.yaw, -23.0);
    EXPECT_DOUBLE_EQ(msg.torso_pose.position.x, 0.005);
    EXPECT_DOUBLE_EQ(msg.toe_positions[0].x, 0.201);
    EXPECT_DOUBLE_EQ(msg.toe_positions[2].x, -0.201);
    EXPECT_DOUBLE_EQ(msg.toe_positions[3].y, 0.160);
    receiver->moveTorso(toeTargets(msg), CPose(msg.torso_pose));
    receiver->setHeadOrientation(COrientation(msg.head_pose));
    EXPECT_EQ(bodyPose(*receiver), msg);
    for (auto leg : bodyLegOrder) expectAnglesNear(source->getLegAngles(leg), receiver->getLegAngles(leg));
}
TEST_F(BodyPoseTest, JointTargetsRoundTripWithNonzeroTorso) {
    source->moveTorso(source->getStandingToePositions(), CPose(0.005, -0.003, 0.002, 3.0, -2.0, 5.0));
    for (auto leg : bodyLegOrder) {
        auto angles = source->getLegAngles(leg);
        angles.torso_coxa += 10.0 * units::deg;
        angles.coxa_femur += 15.0 * units::deg;
        angles.femur_tibia += 20.0 * units::deg;
        source->setLegAngles(leg, angles);
    }
    auto msg = bodyPose(*source);
    receiver->moveTorso(toeTargets(msg), CPose(msg.torso_pose));
    for (auto leg : bodyLegOrder) expectAnglesNear(source->getLegAngles(leg), receiver->getLegAngles(leg));
}
TEST_F(BodyPoseTest, PartialLegUpdateRetainsOtherTargets) {
    const auto before = bodyPose(*source);
    source->setToePosition(ELegIndex::LeftMid, CPosition(0.0, -0.19, -0.01));
    const auto after = bodyPose(*source);
    for (size_t i = 0; i < 6; ++i) {
        if (i == 4)
            EXPECT_DOUBLE_EQ(after.toe_positions[i].z, -0.01);
        else
            EXPECT_EQ(after.toe_positions[i], before.toe_positions[i]);
    }
}

#include "gait/gait_clap.hpp"
#include "gait/gait_high_five.hpp"
#include "gait/gait_test_legs.hpp"
#include "rcl/time.h"

TEST_F(BodyPoseTest, JointBasedGaitsPreserveEverySampleAcrossMessageBoundary) {
    auto params = test_helpers::makeDeclaredParameters(source_node);
    // Share ownership for the existing gait interface.
    auto planner = std::shared_ptr<CPoseModel>(std::move(source));
    ASSERT_EQ(rcl_enable_ros_time_override(source_node->get_clock()->get_clock_handle()), RCL_RET_OK);
    std::vector<std::unique_ptr<ISequenceGait>> gaits;
    gaits.push_back(std::make_unique<CHighFiveGait>(source_node, planner, params.high_five));
    gaits.push_back(std::make_unique<CClapGait>(source_node, planner, params.clap));
    gaits.push_back(std::make_unique<CTestLegsGait>(source_node, planner, params.test_legs));
    int64_t time = 1000000000;
    for (auto& gait : gaits) {
        planner->moveTorso(planner->getStandingToePositions(), CPose(0.003, -0.002, 0.001, 2.0, -1.0, 3.0));
        gait->start(1.0 * units::s, 0);
        int tick = 0;
        while (gait->state() != EGaitState::Stopped && tick++ < 1000) {
            time += 100000000;
            ASSERT_EQ(rcl_set_ros_time_override(source_node->get_clock()->get_clock_handle(), time),
                      RCL_RET_OK);
            gait->update();
            const auto msg = bodyPose(*planner);
            ASSERT_TRUE(validBodyPose(msg));
            receiver->moveTorso(toeTargets(msg), CPose(msg.torso_pose));
            for (auto leg : bodyLegOrder) {
                SCOPED_TRACE(std::to_string(tick) + " " + legIndexToName(leg));
                const auto expected = planner->getLegAngles(leg);
                const auto actual = receiver->getLegAngles(leg);
                EXPECT_NEAR(
                    std::remainder((expected.torso_coxa - actual.torso_coxa).numerical_value_in(units::deg),
                                   360.0),
                    0.0, 1e-7);
                EXPECT_NEAR(expected.coxa_femur.numerical_value_in(units::deg),
                            actual.coxa_femur.numerical_value_in(units::deg), 1e-7);
                EXPECT_NEAR(expected.femur_tibia.numerical_value_in(units::deg),
                            actual.femur_tibia.numerical_value_in(units::deg), 1e-7);
            }
        }
        EXPECT_LT(tick, 1000);
    }
}
