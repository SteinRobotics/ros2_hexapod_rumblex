#include <gtest/gtest.h>

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
        node = std::make_shared<rclcpp::Node>("movement_handler_test", options);
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
        handler.reset();
        rclcpp::shutdown();
    }
    void pump(std::chrono::milliseconds duration) {
        const auto until = std::chrono::steady_clock::now() + duration;
        do {
            rclcpp::spin_some(node);
            handler->update();
            std::this_thread::sleep_for(10ms);
        } while (std::chrono::steady_clock::now() < until);
        rclcpp::spin_some(node);
    }
    rumblex_interfaces::msg::BodyPose initialPose() {
        rclcpp::NodeOptions options;
        options.parameter_overrides(test_helpers::defaultKinematicsParameters());
        auto model_node = std::make_shared<rclcpp::Node>("seed", options);
        CPoseModel model(model_node);
        model.setHeadOrientation(COrientation(0.0, 7.0, 13.0));
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
    EXPECT_TRUE(handler->done());
    ASSERT_FALSE(poses.empty());
    EXPECT_TRUE(validBodyPose(poses.back()));
}
TEST_F(MovementHandlerTest, FeedbackAloneDoesNotCommandMotion) {
    feedback->publish(initialPose());
    pump(250ms);
    EXPECT_TRUE(poses.empty());
    ASSERT_FALSE(velocities.empty());
    EXPECT_EQ(velocities.back(), geometry_msgs::msg::Twist());
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
    request(MovementRequest::SEQUENCE_STAND_UP);
    pump(1500ms);
    EXPECT_DOUBLE_EQ(velocities.back().linear.x, 0.0);
}
