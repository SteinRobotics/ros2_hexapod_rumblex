#include "movement_handler_test_fixture.hpp"

namespace {
TEST_F(MovementHandlerTest, FeedbackDurationStartsAfterHeadPreparationAndCanRepeat) {
    stand();
    feed_range = true;
    hold(0.3);
    pump(800ms);
    EXPECT_FALSE(handler->requestFinished());
    pump(1000ms);
    EXPECT_TRUE(handler->requestFinished());
    EXPECT_TRUE(handler->motionStopped());
    ASSERT_FALSE(poses.empty());
    EXPECT_DOUBLE_EQ(poses.back().head_orientation.yaw, 0.0);
    hold(0.3);
    pump(1800ms);
    EXPECT_TRUE(handler->requestFinished());
    ASSERT_FALSE(velocities.empty());
    EXPECT_DOUBLE_EQ(velocities.back().linear.x, 0.0);
}
TEST_F(MovementHandlerTest, SensorLossAbortsAndStopsWalking) {
    stand();
    sequence_ended = false;
    feed_range = true;
    range_m = 1.0F;
    const auto initial_torso = poses.back().torso_pose;
    hold(10.0);
    pump(1800ms);
    EXPECT_EQ(poses.back().torso_pose, initial_torso);
    EXPECT_EQ(node->count_subscribers("test_imu"), 0u);
    EXPECT_FALSE(sequence_ended);
    ASSERT_FALSE(velocities.empty());
    EXPECT_GT(velocities.back().linear.x, 0.0);
    feed_range = false;
    pump(700ms);
    EXPECT_TRUE(sequence_ended);
    EXPECT_DOUBLE_EQ(velocities.back().linear.x, 0.0);
    pump(6500ms);
    EXPECT_TRUE(handler->requestFinished());
    EXPECT_TRUE(handler->motionStopped());
}
TEST_F(MovementHandlerTest, MissingSensorsTimeOutAfterHeadPreparation) {
    stand();
    sequence_ended = false;
    hold(1.0);
    pump(800ms);
    EXPECT_FALSE(sequence_ended);
    pump(2500ms);
    EXPECT_TRUE(sequence_ended);
    EXPECT_TRUE(handler->requestFinished());
    EXPECT_TRUE(handler->motionStopped());
    ASSERT_FALSE(velocities.empty());
    EXPECT_DOUBLE_EQ(velocities.back().linear.x, 0.0);
}
TEST_F(MovementHandlerTest, CancellationClearsFeedbackVelocity) {
    stand();
    feed_range = true;
    range_m = 1.0F;
    const auto initial_torso = poses.back().torso_pose;
    hold(10.0);
    pump(1800ms);
    EXPECT_EQ(poses.back().torso_pose, initial_torso);
    EXPECT_EQ(node->count_subscribers("test_imu"), 0u);
    handler->cancel();
    pump(6500ms);
    EXPECT_TRUE(handler->motionStopped());
    EXPECT_TRUE(handler->requestFinished());
    EXPECT_DOUBLE_EQ(velocities.back().linear.x, 0.0);
}
TEST_F(MovementHandlerTest, FeedbackUsesStandardInitialPoseDeferralAndRejectsLaydown) {
    sequence_ended = false;
    hold(1.0);
    pump(2200ms);
    EXPECT_FALSE(sequence_ended);
    EXPECT_FALSE(handler->requestFinished());
    feedback->publish(initialPose());
    pump(150ms);
    sequence_ended = false;
    hold(1.0);
    pump(150ms);
    EXPECT_TRUE(sequence_ended);
    EXPECT_TRUE(handler->requestFinished());
}
}  // namespace
