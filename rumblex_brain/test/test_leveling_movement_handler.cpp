#include "movement_handler_test_fixture.hpp"

namespace {
TEST_F(MovementHandlerTest, LevelingUsesOnlyImuAndPreservesFeetAndHead) {
    stand();
    auto pose = std::make_shared<RequestMovementType>();
    pose->movementRequest.type = MovementRequest::SINGLE_POSE;
    pose->movementRequest.body_pose.has_head_orientation = true;
    pose->movementRequest.body_pose.head_orientation.yaw = 15;
    pose->movementRequest.body_pose.head_orientation.pitch = -5;
    handler->run(pose);
    pump(1200ms);
    const auto initial = poses.back();
    feed_imu = true;
    imu_roll = 0.1;
    level(0.5);
    pump(1800ms);
    EXPECT_TRUE(handler->requestFinished());
    EXPECT_TRUE(handler->motionStopped());
    EXPECT_EQ(node->count_subscribers("test_range"), 0u);
    EXPECT_LT(poses.back().torso_pose.orientation.roll, initial.torso_pose.orientation.roll);
    EXPECT_EQ(poses.back().toe_positions, initial.toe_positions);
    EXPECT_EQ(poses.back().head_orientation, initial.head_orientation);
    for (const auto& velocity : velocities) EXPECT_EQ(velocity, geometry_msgs::msg::Twist());
}
TEST_F(MovementHandlerTest, LevelingSensorLossAbortsAndCancellationFreezesCorrection) {
    stand();
    sequence_ended = false;
    feed_imu = true;
    imu_roll = 0.1;
    level(10);
    pump(400ms);
    EXPECT_FALSE(sequence_ended);
    feed_imu = false;
    pump(1700ms);
    EXPECT_TRUE(sequence_ended);
    EXPECT_TRUE(handler->requestFinished());
    EXPECT_TRUE(handler->motionStopped());
    feed_imu = true;
    level(10);
    pump(400ms);
    handler->cancel();
    const auto final_torso = poses.back().torso_pose;
    pump(1500ms);
    EXPECT_TRUE(handler->requestFinished());
    EXPECT_TRUE(handler->motionStopped());
    EXPECT_EQ(poses.back().torso_pose, final_torso);
    EXPECT_EQ(velocities.back(), geometry_msgs::msg::Twist());
}
TEST_F(MovementHandlerTest, LevelingWaitsOnlyForImuAndRejectsLaydown) {
    feedback->publish(initialPose());
    pump(150ms);
    sequence_ended = false;
    level(1);
    pump(200ms);
    EXPECT_TRUE(sequence_ended);
    stand();
    sequence_ended = false;
    feed_range = true;
    level(1);
    pump(2300ms);
    EXPECT_TRUE(sequence_ended);
    EXPECT_TRUE(handler->requestFinished());
}
}  // namespace
