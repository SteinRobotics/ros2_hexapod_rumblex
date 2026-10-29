#include <gtest/gtest.h>

#include "gait/gait_controller.hpp"
#include "rumblex_utils/body_pose.hpp"
#include "test_helpers.hpp"

namespace {
using namespace brain;
using namespace mp_units;
using namespace mp_units::si::unit_symbols;

class SinglePoseTargetTest : public ::testing::Test {
   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        rclcpp::NodeOptions options;
        options.parameter_overrides(test_helpers::defaultRobotParameters());
        node = std::make_shared<rclcpp::Node>("single_pose_target", options);
        model = std::make_shared<CPoseModel>(node);
        model->moveTorso(model->getStandingToePositions(), CPose(0.003, -0.002, 0.001, 2, -3, 4));
        model->setHeadOrientation(COrientation(0, 6, 8));
        controller = std::make_unique<CGaitController>(node, model);
    }
    void TearDown() override {
        controller.reset();
        model.reset();
        node.reset();
        rclcpp::shutdown();
    }
    MovementRequest poseRequest() {
        MovementRequest request;
        request.type = MovementRequest::SINGLE_POSE;
        request.name = "SINGLE_POSE";
        request.duration_s = 1.0 * s;
        return request;
    }
    void advance(int samples) {
        for (int i = 0; i < samples; ++i)
            controller->updateSelectedGait(Velocity(), CPose(), COrientation(), 0.02 * s);
    }
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<CPoseModel> model;
    std::unique_ptr<CGaitController> controller;
};

TEST_F(SinglePoseTargetTest, HeadOnlyPreservesTorsoAndToesThroughoutTrajectory) {
    const auto origin = bodyPose(*model);
    auto request = poseRequest();
    request.body_pose.has_head_orientation = true;
    request.body_pose.head_orientation.yaw = -12;
    ASSERT_TRUE(controller->setGait(request));
    EXPECT_EQ(bodyPose(*model), origin);
    advance(25);
    const auto middle = bodyPose(*model);
    EXPECT_EQ(middle.torso_pose, origin.torso_pose);
    EXPECT_EQ(middle.toe_positions, origin.toe_positions);
    EXPECT_NEAR(middle.head_orientation.yaw, -2, 1e-9);
    advance(25);
    EXPECT_TRUE(controller->motionStopped());
    EXPECT_EQ(bodyPose(*model), resolveBodyPose(origin, request.body_pose));
}

TEST_F(SinglePoseTargetTest, ToesInterpolateWithTheSameTimingAsHeadAndTorso) {
    const auto origin = bodyPose(*model);
    auto request = poseRequest();
    request.body_pose = origin;
    request.body_pose.head_orientation.yaw += 10;
    request.body_pose.torso_pose.position.z += 0.004;
    for (auto& toe : request.body_pose.toe_positions) toe.z += 0.006;
    ASSERT_TRUE(controller->setGait(request));
    advance(25);
    const auto middle = bodyPose(*model);
    EXPECT_NEAR(middle.head_orientation.yaw, origin.head_orientation.yaw + 5, 1e-9);
    EXPECT_NEAR(middle.torso_pose.position.z, origin.torso_pose.position.z + 0.002, 1e-9);
    for (size_t i = 0; i < 6; ++i)
        EXPECT_NEAR(middle.toe_positions[i].z, origin.toe_positions[i].z + 0.003, 1e-9);
    advance(25);
    EXPECT_EQ(bodyPose(*model), request.body_pose);
    // An explicit repeated full pose still holds for the requested time.
    ASSERT_TRUE(controller->setGait(request));
    EXPECT_FALSE(controller->motionStopped());
    advance(49);
    EXPECT_FALSE(controller->motionStopped());
    advance(1);
    EXPECT_TRUE(controller->motionStopped());
}

TEST_F(SinglePoseTargetTest, PendingSameTypeResolvesOmissionsAtThePreviousEndpoint) {
    auto first = poseRequest();
    first.body_pose.has_torso_pose = true;
    first.body_pose.torso_pose.position.z = 0.007;
    ASSERT_TRUE(controller->setGait(first));
    advance(10);
    auto second = poseRequest();
    second.body_pose.has_head_orientation = true;
    second.body_pose.head_orientation.yaw = -15;
    ASSERT_TRUE(controller->setGait(second));
    EXPECT_TRUE(controller->hasPendingGait());
    advance(40);
    const auto endpoint = bodyPose(*model);
    EXPECT_EQ(endpoint.torso_pose, first.body_pose.torso_pose);
    advance(50);
    EXPECT_FALSE(controller->hasPendingGait());
    EXPECT_TRUE(controller->motionStopped());
    EXPECT_EQ(bodyPose(*model), resolveBodyPose(endpoint, second.body_pose));
}

TEST_F(SinglePoseTargetTest, CancellationFinishesCurrentTargetAndClearsPendingOnReplacement) {
    auto first = poseRequest();
    first.body_pose.has_head_orientation = true;
    first.body_pose.head_orientation.yaw = 15;
    const auto expected = resolveBodyPose(bodyPose(*model), first.body_pose);
    ASSERT_TRUE(controller->setGait(first));
    advance(10);
    auto pending = poseRequest();
    pending.body_pose.has_torso_pose = true;
    pending.body_pose.torso_pose.position.z = 0.02;
    ASSERT_TRUE(controller->setGait(pending));
    ASSERT_TRUE(controller->hasPendingGait());
    controller->requestStopSelectedGait();
    EXPECT_FALSE(controller->hasPendingGait());
    advance(90);
    EXPECT_TRUE(controller->motionStopped());
    EXPECT_EQ(bodyPose(*model), expected);
    first.body_pose.head_orientation.yaw = -15;
    ASSERT_TRUE(controller->setGait(first));
    advance(50);
    EXPECT_DOUBLE_EQ(bodyPose(*model).head_orientation.yaw, -15);
}
}  // namespace
