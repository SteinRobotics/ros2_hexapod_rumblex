#pragma once

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <thread>

#include "handler/movement.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "test_helpers.hpp"

namespace {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using mp_units::angular::unit_symbols::deg;
using namespace brain;
using namespace std::chrono_literals;
class MovementHandlerTest : public ::testing::Test {
   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        rclcpp::NodeOptions options;
        auto parameters = test_helpers::defaultRobotParameters();
        const auto extra = test_helpers::holdDistanceParameters();
        parameters.insert(parameters.end(), extra.begin(), extra.end());
        const auto leveling = test_helpers::levelTorsoParameters();
        parameters.insert(parameters.end(), leveling.begin(), leveling.end());
        options.parameter_overrides(parameters);
        node = std::make_shared<rclcpp::Node>("movement_handler_test", "/movement_handler_test", options);
        executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(node);
        handler = std::make_unique<CMovement>(node);
        imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("test_imu", rclcpp::SensorDataQoS());
        range_pub = node->create_publisher<sensor_msgs::msg::Range>("test_range", rclcpp::SensorDataQoS());
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
            if (feed_imu) {
                sensor_msgs::msg::Imu imu;
                imu.header.stamp = node->now();
                imu.header.frame_id = "base_link";
                const double half_roll = imu_roll / 2.0;
                imu.orientation.x = std::sin(half_roll);
                imu.orientation.w = std::cos(half_roll);
                imu_pub->publish(imu);
            }
            if (feed_range) {
                sensor_msgs::msg::Range range;
                range.header.stamp = node->now();
                range.header.frame_id = "base_link";
                range.range = range_m;
                range.min_range = 0.05;
                range.max_range = 10.0;
                range_pub->publish(range);
            }
            executor->spin_some();
            handler->update();
            if (handler->takeSequenceAbort()) sequence_ended = true;
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
        model.setHeadOrientation(COrientation(
            0.0 * deg, -node->get_parameter("gait.generic.head_max_pitch_deg").as_double() * deg, 0.0 * deg));
        return bodyPose(model);
    }
    void request(uint8_t type, double duration = 1.0) {
        auto r = std::make_shared<RequestMovementType>();
        r->movementRequest.type = type;
        r->movementRequest.duration_s = duration * s;
        r->movementRequest.name = "test";
        handler->run(r);
    }
    void stand() {
        feedback->publish(initialPose());
        pump(150ms);
        request(MovementRequest::SEQUENCE_STAND_UP);
        pump(1300ms);
        ASSERT_TRUE(handler->requestFinished());
    }
    void hold(double duration) {
        auto action = std::make_shared<RequestMovementType>();
        action->movementRequest.type = MovementRequest::HOLD_DISTANCE;
        action->movementRequest.name = "HOLD_DISTANCE";
        action->movementRequest.distance_m = 0.6 * m;
        action->movementRequest.duration_s = duration * s;
        handler->run(action);
    }
    void level(double duration) {
        auto action = std::make_shared<RequestMovementType>();
        action->movementRequest.type = MovementRequest::LEVEL_TORSO;
        action->movementRequest.name = "LEVEL_TORSO";
        action->movementRequest.duration_s = duration * s;
        handler->run(action);
    }
    bool sequence_ended = false;
    bool feed_range = false;
    bool feed_imu = false;
    double imu_roll = 0.0;
    float range_m = 0.6F;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub;
    std::shared_ptr<rclcpp::Node> node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    std::unique_ptr<CMovement> handler;
    rclcpp::Publisher<rumblex_interfaces::msg::BodyPose>::SharedPtr feedback;
    rclcpp::Subscription<rumblex_interfaces::msg::BodyPose>::SharedPtr commands;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr telemetry;
    std::vector<rumblex_interfaces::msg::BodyPose> poses;
    std::vector<geometry_msgs::msg::Twist> velocities;
};
}  // namespace
