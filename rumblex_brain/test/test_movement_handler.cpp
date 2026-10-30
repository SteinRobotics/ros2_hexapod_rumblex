#include "movement_handler_test_fixture.hpp"
#include "parser/sequence_parser.hpp"

namespace {
TEST_F(MovementHandlerTest, WaitsForFeedbackAndDoesNotStartDurationEarly) {
    request(MovementRequest::SEQUENCE_STAND_UP, 0.2);
    pump(350ms);
    EXPECT_TRUE(poses.empty());
    EXPECT_FALSE(handler->requestFinished());
    feedback->publish(initialPose());
    pump(400ms);
    EXPECT_FALSE(handler->requestFinished());
    pump(800ms);
    EXPECT_TRUE(handler->requestFinished());
    ASSERT_FALSE(poses.empty());
    EXPECT_TRUE(validBodyPose(poses.back()));
}
TEST_F(MovementHandlerTest, AlreadyLayingDownDoesNotCommandMotion) {
    feedback->publish(initialPose());
    pump(250ms);
    EXPECT_TRUE(poses.empty());
    ASSERT_FALSE(velocities.empty());
    EXPECT_EQ(velocities.back(), geometry_msgs::msg::Twist());
    EXPECT_TRUE(handler->requestFinished());
}
TEST_F(MovementHandlerTest, StartupMovesToLaydownAfterValidFeedback) {
    pump(150ms);
    EXPECT_TRUE(poses.empty());
    auto invalid = initialPose();
    invalid.head_orientation.pitch = std::numeric_limits<double>::quiet_NaN();
    feedback->publish(invalid);
    pump(150ms);
    EXPECT_TRUE(poses.empty());

    const auto target = initialPose();
    rclcpp::NodeOptions options;
    options.parameter_overrides(test_helpers::defaultKinematicsParameters());
    auto model_node = std::make_shared<rclcpp::Node>("standing_seed", options);
    CPoseModel model(model_node);
    model.moveTorso(model.getStandingToePositions(), CPose());
    const auto seed = bodyPose(model);
    feedback->publish(seed);
    pump(250ms);
    ASSERT_FALSE(poses.empty());
    EXPECT_NE(poses.back(), seed);
    EXPECT_NE(poses.back(), target);
    EXPECT_FALSE(handler->requestFinished());

    pump(1000ms);
    EXPECT_EQ(poses.back(), target);
    EXPECT_TRUE(handler->requestFinished());
    const auto command_count = poses.size();
    feedback->publish(seed);
    pump(200ms);
    EXPECT_EQ(poses.size(), command_count);
}
TEST_F(MovementHandlerTest, PublishesHighFiveRestorationAndIgnoresLaterFeedback) {
    const auto seed = initialPose();
    feedback->publish(seed);
    pump(200ms);
    CSequenceParser parser(node);
    ASSERT_TRUE(parser.parseFile(std::string(RUMBLEX_SEQUENCES_DIRECTORY) + "/high_five.yaml"));
    const auto sequence = parser.getSequence("high_five");
    ASSERT_TRUE(sequence);
    auto movementAt = [&](size_t index) {
        for (const auto& action : sequence->get().actionGroups[index])
            if (auto movement = std::dynamic_pointer_cast<RequestMovementType>(action)) return movement;
        return std::shared_ptr<RequestMovementType>();
    };
    ASSERT_TRUE(movementAt(0));
    handler->run(movementAt(0));
    pump(350ms);
    auto unrelated = seed;
    unrelated.head_orientation.yaw = -80.0;
    feedback->publish(unrelated);
    pump(800ms);
    ASSERT_TRUE(handler->requestFinished());
    handler->run(movementAt(1));
    pump(2100ms);
    ASSERT_TRUE(handler->requestFinished());
    handler->run(movementAt(2));
    pump(1100ms);
    ASSERT_TRUE(handler->requestFinished());
    ASSERT_FALSE(poses.empty());
    EXPECT_NEAR(poses.back().head_orientation.yaw, seed.head_orientation.yaw, 1e-8);
    EXPECT_NEAR(poses.back().head_orientation.pitch, seed.head_orientation.pitch, 1e-8);
    for (size_t i = 0; i < 6; ++i) {
        EXPECT_NEAR(poses.back().toe_positions[i].x, seed.toe_positions[i].x, 1e-8);
        EXPECT_NEAR(poses.back().toe_positions[i].y, seed.toe_positions[i].y, 1e-8);
        EXPECT_NEAR(poses.back().toe_positions[i].z, seed.toe_positions[i].z, 1e-8);
    }
}
TEST_F(MovementHandlerTest, VelocityIsZeroDuringNonWalkingGait) {
    feedback->publish(initialPose());
    pump(150ms);
    auto velocity = std::make_shared<RequestVelocity>();
    velocity->velocity.linear.x = 0.01;
    handler->run(velocity);
    request(MovementRequest::CONTINUOUS_MOVE);
    pump(300ms);
    ASSERT_FALSE(velocities.empty());
    EXPECT_DOUBLE_EQ(velocities.back().linear.x, 0.01);
    bool activated = false;
    handler->on_gait_changed = [&](const MovementRequest& request) {
        if (request.type == MovementRequest::SEQUENCE_STAND_UP) activated = true;
    };
    request(MovementRequest::SEQUENCE_STAND_UP);
    // A graceful walking stop completes its swing and places each foot group.
    for (int i = 0; i < 160 && !activated; ++i) pump(50ms);
    ASSERT_TRUE(activated);
    pump(150ms);
    EXPECT_DOUBLE_EQ(velocities.back().linear.x, 0.0);
}

TEST_F(MovementHandlerTest, PendingRepeatingGaitGetsFullDurationAfterActivation) {
    feedback->publish(initialPose());
    pump(150ms);
    request(MovementRequest::SEQUENCE_STAND_UP, 1.0);
    pump(150ms);
    bool activated = false;
    handler->on_gait_changed = [&](const MovementRequest& request) {
        if (request.type == MovementRequest::CONTINUOUS_POSE) activated = true;
    };
    request(MovementRequest::CONTINUOUS_POSE, 0.5);
    pump(600ms);
    EXPECT_FALSE(activated);
    EXPECT_FALSE(handler->requestFinished());
    for (int i = 0; i < 20 && !activated; ++i) pump(50ms);
    ASSERT_TRUE(activated);
    EXPECT_FALSE(handler->requestFinished());
    pump(200ms);
    EXPECT_FALSE(handler->requestFinished());
    pump(400ms);
    EXPECT_TRUE(handler->requestFinished());
    EXPECT_FALSE(handler->motionStopped());
    EXPECT_FALSE(handler->stopRequested());
}

TEST_F(MovementHandlerTest, FiniteGaitWaitsForActualCompletion) {
    feedback->publish(initialPose());
    pump(150ms);
    request(MovementRequest::SEQUENCE_STAND_UP, 0.1);
    pump(300ms);
    EXPECT_FALSE(handler->requestFinished());
    pump(1000ms);
    EXPECT_TRUE(handler->requestFinished());
}

TEST_F(MovementHandlerTest, CancellationFinishesRequestBeforeFiniteMotionStops) {
    feedback->publish(initialPose());
    pump(150ms);
    request(MovementRequest::SEQUENCE_STAND_UP, 1.0);
    pump(150ms);
    ASSERT_FALSE(handler->requestFinished());
    ASSERT_FALSE(handler->motionStopped());
    handler->cancel();
    EXPECT_TRUE(handler->requestFinished());
    EXPECT_TRUE(handler->stopRequested());
    EXPECT_FALSE(handler->motionStopped());
    const auto poses_before_stop = poses.size();
    pump(1200ms);
    EXPECT_TRUE(handler->requestFinished());
    EXPECT_TRUE(handler->motionStopped());
    EXPECT_FALSE(handler->stopRequested());
    EXPECT_GT(poses.size(), poses_before_stop);
}

TEST_F(MovementHandlerTest, RepeatedTimedRequestRestartsCompletionTimer) {
    feedback->publish(initialPose());
    pump(150ms);
    request(MovementRequest::CONTINUOUS_POSE, 0.2);
    pump(300ms);
    ASSERT_TRUE(handler->requestFinished());
    request(MovementRequest::CONTINUOUS_POSE, 0.3);
    EXPECT_FALSE(handler->requestFinished());
    pump(100ms);
    EXPECT_FALSE(handler->requestFinished());
    pump(300ms);
    EXPECT_TRUE(handler->requestFinished());
}

TEST_F(MovementHandlerTest, CancelledPendingRequestDoesNotRestartCompletionTracking) {
    feedback->publish(initialPose());
    pump(150ms);
    request(MovementRequest::SEQUENCE_STAND_UP, 0.5);
    pump(100ms);
    request(MovementRequest::CONTINUOUS_POSE, 1.0);
    handler->cancel();
    pump(700ms);
    EXPECT_TRUE(handler->requestFinished());
}

TEST_F(MovementHandlerTest, PendingFiniteGaitWaitsForItsOwnCompletion) {
    feedback->publish(initialPose());
    pump(150ms);
    request(MovementRequest::SEQUENCE_STAND_UP, 0.8);
    pump(100ms);
    request(MovementRequest::SEQUENCE_LAYDOWN, 0.2);
    pump(350ms);
    EXPECT_FALSE(handler->requestFinished());
    pump(1800ms);
    EXPECT_TRUE(handler->requestFinished());
}

TEST_F(MovementHandlerTest, SameTypeFiniteRequestCompletesAndInvalidRequestDoesNotBlock) {
    feedback->publish(initialPose());
    pump(150ms);
    // Laydown is already selected when the controller is constructed.
    request(MovementRequest::SEQUENCE_LAYDOWN, 0.2);
    pump(1200ms);
    EXPECT_TRUE(handler->requestFinished());
    request(MovementRequest::NO_REQUEST);
    EXPECT_TRUE(handler->requestFinished());
    request(MovementRequest::SEQUENCE_DANCE);
    EXPECT_TRUE(handler->requestFinished());
}

TEST_F(MovementHandlerTest, OrientationPublishesFrequentSamplesWithoutCompressingDuration) {
    const auto seed = initialPose();
    feedback->publish(seed);
    pump(200ms);
    auto request = std::make_shared<RequestMovementType>();
    request->movementRequest.type = MovementRequest::CONTINUOUS_POSE;
    request->movementRequest.body_pose.has_head_orientation = true;
    request->movementRequest.body_pose.head_orientation = seed.head_orientation;
    request->movementRequest.body_pose.head_orientation.yaw = 23.0;
    handler->run(request);
    poses.clear();
    pump(250ms);
    EXPECT_GE(poses.size(), 10u);
    ASSERT_FALSE(poses.empty());
    EXPECT_GT(poses.back().head_orientation.yaw, seed.head_orientation.yaw);
    EXPECT_LT(poses.back().head_orientation.yaw, 20.0);
    pump(1100ms);
    EXPECT_DOUBLE_EQ(poses.back().head_orientation.yaw, 23.0);
}

TEST_F(MovementHandlerTest, PartialFeedbackCannotInitializeAndPoseRequestRetainsComponents) {
    auto seed = initialPose();
    auto partial = seed;
    partial.has_toe_positions = false;
    feedback->publish(partial);
    auto pose = std::make_shared<RequestMovementType>();
    pose->movementRequest.type = MovementRequest::SINGLE_POSE;
    pose->movementRequest.body_pose.has_head_orientation = true;
    pose->movementRequest.body_pose.head_orientation.yaw = 12;
    handler->run(pose);
    pump(200ms);
    EXPECT_TRUE(poses.empty());
    EXPECT_FALSE(handler->requestFinished());
    feedback->publish(seed);
    pump(1200ms);
    ASSERT_FALSE(poses.empty());
    EXPECT_TRUE(handler->requestFinished());
    EXPECT_EQ(poses.back(), resolveBodyPose(seed, pose->movementRequest.body_pose));
}

}  // namespace
