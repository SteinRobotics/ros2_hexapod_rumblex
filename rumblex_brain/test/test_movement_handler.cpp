#include <gtest/gtest.h>

#include <limits>
#include <thread>

#include "handler/movement.hpp"
#include "test_helpers.hpp"

using namespace brain;
using namespace std::chrono_literals;
class MovementHandlerTest : public ::testing::Test {
   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        rclcpp::NodeOptions options;
        options.parameter_overrides(test_helpers::defaultRobotParameters());
        node = std::make_shared<rclcpp::Node>("movement_handler_test", "/movement_handler_test", options);
        executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(node);
        handler = std::make_unique<CMovement>(node);
        feedback = node->create_publisher<rumblex_interfaces::msg::BodyPose>(
            "body_pose_actual", rclcpp::QoS(1).transient_local());
        commands = node->create_subscription<rumblex_interfaces::msg::BodyPose>(
            "cmd_movement", 10,
            [this](const rumblex_interfaces::msg::BodyPose& msg) { poses.push_back(msg); });
        telemetry = node->create_subscription<geometry_msgs::msg::Twist>(
            "movement_velocity", 10,
            [this](const geometry_msgs::msg::Twist& msg) { velocities.push_back(msg); });
    }
    void TearDown() override {
        executor.reset();
        handler.reset();
        rclcpp::shutdown();
    }
    void pump(std::chrono::milliseconds duration) {
        const auto until = std::chrono::steady_clock::now() + duration;
        do {
            executor->spin_some();
            handler->update();
            std::this_thread::sleep_for(10ms);
        } while (std::chrono::steady_clock::now() < until);
        executor->spin_some();
    }
    rumblex_interfaces::msg::BodyPose initialPose() {
        rclcpp::NodeOptions options;
        options.parameter_overrides(test_helpers::defaultKinematicsParameters());
        auto model_node = std::make_shared<rclcpp::Node>("seed", options);
        CPoseModel model(model_node);
        model.moveTorso(model.getLaydownToePositions(), CPose());
        model.setHeadOrientation(
            COrientation(0.0 * units::deg,
                         -node->get_parameter("gait.generic.head_max_pitch_deg").as_double() * units::deg,
                         0.0 * units::deg));
        return bodyPose(model);
    }
    void request(uint8_t type, double duration = 1.0) {
        auto r = std::make_shared<RequestMovementType>();
        r->movementRequest.type = type;
        r->movementRequest.duration_s = duration;
        r->movementRequest.name = "test";
        handler->run(r);
    }
    std::shared_ptr<rclcpp::Node> node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    std::unique_ptr<CMovement> handler;
    rclcpp::Publisher<rumblex_interfaces::msg::BodyPose>::SharedPtr feedback;
    rclcpp::Subscription<rumblex_interfaces::msg::BodyPose>::SharedPtr commands;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr telemetry;
    std::vector<rumblex_interfaces::msg::BodyPose> poses;
    std::vector<geometry_msgs::msg::Twist> velocities;
};
TEST_F(MovementHandlerTest, WaitsForFeedbackAndDoesNotStartDurationEarly) {
    request(MovementRequest::SEQUENCE_STAND_UP, 0.2);
    pump(350ms);
    EXPECT_TRUE(poses.empty());
    EXPECT_FALSE(handler->done());
    feedback->publish(initialPose());
    pump(400ms);
    EXPECT_FALSE(handler->done());
    pump(800ms);
    EXPECT_TRUE(handler->done());
    ASSERT_FALSE(poses.empty());
    EXPECT_TRUE(validBodyPose(poses.back()));
}
TEST_F(MovementHandlerTest, AlreadyLayingDownDoesNotCommandMotion) {
    feedback->publish(initialPose());
    pump(250ms);
    EXPECT_TRUE(poses.empty());
    ASSERT_FALSE(velocities.empty());
    EXPECT_EQ(velocities.back(), geometry_msgs::msg::Twist());
    EXPECT_TRUE(handler->done());
}
TEST_F(MovementHandlerTest, StartupMovesToLaydownAfterValidFeedback) {
    pump(150ms);
    EXPECT_TRUE(poses.empty());
    auto invalid = initialPose();
    invalid.head_pose.pitch = std::numeric_limits<double>::quiet_NaN();
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
    EXPECT_FALSE(handler->done());

    pump(1000ms);
    EXPECT_EQ(poses.back(), target);
    EXPECT_TRUE(handler->done());
    const auto command_count = poses.size();
    feedback->publish(seed);
    pump(200ms);
    EXPECT_EQ(poses.size(), command_count);
}
TEST_F(MovementHandlerTest, PublishesHighFiveRestorationAndIgnoresLaterFeedback) {
    const auto seed = initialPose();
    feedback->publish(seed);
    pump(200ms);
    request(MovementRequest::SEQUENCE_HIGH_FIVE);
    pump(350ms);
    auto unrelated = seed;
    unrelated.head_pose.yaw = -80.0;
    feedback->publish(unrelated);
    pump(4500ms);
    ASSERT_FALSE(poses.empty());
    EXPECT_NEAR(poses.back().head_pose.yaw, seed.head_pose.yaw, 1e-8);
    EXPECT_NEAR(poses.back().head_pose.pitch, seed.head_pose.pitch, 1e-8);
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
    EXPECT_FALSE(handler->done());
    for (int i = 0; i < 20 && !activated; ++i) pump(50ms);
    ASSERT_TRUE(activated);
    EXPECT_FALSE(handler->done());
    pump(200ms);
    EXPECT_FALSE(handler->done());
    pump(400ms);
    EXPECT_TRUE(handler->done());
}

TEST_F(MovementHandlerTest, FiniteGaitWaitsForActualCompletion) {
    feedback->publish(initialPose());
    pump(150ms);
    request(MovementRequest::SEQUENCE_HIGH_FIVE, 0.1);
    pump(300ms);
    EXPECT_FALSE(handler->done());
    pump(4200ms);
    EXPECT_TRUE(handler->done());
}

TEST_F(MovementHandlerTest, RepeatedTimedRequestRestartsCompletionTimer) {
    feedback->publish(initialPose());
    pump(150ms);
    request(MovementRequest::CONTINUOUS_POSE, 0.2);
    pump(300ms);
    ASSERT_TRUE(handler->done());
    request(MovementRequest::CONTINUOUS_POSE, 0.3);
    EXPECT_FALSE(handler->done());
    pump(100ms);
    EXPECT_FALSE(handler->done());
    pump(300ms);
    EXPECT_TRUE(handler->done());
}

TEST_F(MovementHandlerTest, CancelledPendingRequestDoesNotRestartCompletionTracking) {
    feedback->publish(initialPose());
    pump(150ms);
    request(MovementRequest::SEQUENCE_STAND_UP, 0.5);
    pump(100ms);
    request(MovementRequest::CONTINUOUS_POSE, 1.0);
    handler->cancel();
    pump(700ms);
    EXPECT_TRUE(handler->done());
}

TEST_F(MovementHandlerTest, PendingFiniteGaitWaitsForItsOwnCompletion) {
    feedback->publish(initialPose());
    pump(150ms);
    request(MovementRequest::SEQUENCE_STAND_UP, 0.8);
    pump(100ms);
    request(MovementRequest::SEQUENCE_LAYDOWN, 0.2);
    pump(350ms);
    EXPECT_FALSE(handler->done());
    pump(1800ms);
    EXPECT_TRUE(handler->done());
}

TEST_F(MovementHandlerTest, SameTypeFiniteRequestCompletesAndInvalidRequestDoesNotBlock) {
    feedback->publish(initialPose());
    pump(150ms);
    // Laydown is already selected when the controller is constructed.
    request(MovementRequest::SEQUENCE_LAYDOWN, 0.2);
    pump(1200ms);
    EXPECT_TRUE(handler->done());
    request(MovementRequest::NO_REQUEST);
    EXPECT_TRUE(handler->done());
    request(MovementRequest::SEQUENCE_DANCE);
    EXPECT_TRUE(handler->done());
}

TEST_F(MovementHandlerTest, OrientationPublishesFrequentSamplesWithoutCompressingDuration) {
    const auto seed = initialPose();
    feedback->publish(seed);
    pump(200ms);
    request(MovementRequest::CONTINUOUS_POSE);
    auto head = std::make_shared<RequestHeadOrientation>();
    head->orientation = seed.head_pose;
    head->orientation.yaw = 23.0;
    handler->run(head);
    poses.clear();
    pump(250ms);
    EXPECT_GE(poses.size(), 10u);
    ASSERT_FALSE(poses.empty());
    EXPECT_GT(poses.back().head_pose.yaw, seed.head_pose.yaw);
    EXPECT_LT(poses.back().head_pose.yaw, 20.0);
    pump(1100ms);
    EXPECT_DOUBLE_EQ(poses.back().head_pose.yaw, 23.0);
}
