#include <gtest/gtest.h>

#include <algorithm>
#include <numbers>
#include <thread>

#include "handler/leg_servo_conversion.hpp"
#include "mock/mock_servo_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/requester.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;
using namespace rumblex_movement;

class RequesterTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        rclcpp::NodeOptions options;
        options.parameter_overrides(test_helpers::defaultKinematicsParameters());
        node_ = std::make_shared<rclcpp::Node>("test_requester_node", options);
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        servo_controller_mock_ = std::make_shared<CServoControllerMock>();
        servo_controller_mock_->initial_head_orientation = rumblex_geometry::COrientation(0.0, 7.0, -13.0);
        servo_controller_mock_->initial_leg_angles[rumblex_geometry::ELegIndex::RightFront] =
            rumblex_geometry::CLegAngles(5.0, 25.0, -40.0);
        requester_ = std::make_unique<CRequester>(node_, servo_controller_mock_);
    }

    void TearDown() override {
        executor_.reset();
        requester_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::unique_ptr<CRequester> requester_;
    std::shared_ptr<CServoControllerMock> servo_controller_mock_;
};

TEST_F(RequesterTest, ConstructAndUpdate) {
    // basic smoke test: update should run without throwing
    EXPECT_NO_THROW(requester_->update(std::chrono::milliseconds(0)));
    EXPECT_FALSE(node_->has_parameter("servo.serial_port"));
}

TEST_F(RequesterTest, PublishesInitialJointStatesWithoutMovementRequest) {
    sensor_msgs::msg::JointState::SharedPtr received;
    auto subscriber = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        [&received](sensor_msgs::msg::JointState::SharedPtr message) { received = message; });

    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!received && std::chrono::steady_clock::now() < deadline) {
        requester_->update(100ms);
        executor_->spin_some();
        std::this_thread::sleep_for(10ms);
    }

    ASSERT_NE(received, nullptr);
    EXPECT_EQ(received->name.size(), 20u);
    EXPECT_EQ(received->position.size(), received->name.size());
    for (const auto* joint : {"right_front_coxa_joint", "right_front_femur_joint", "right_front_tibia_joint",
                              "head_yaw_joint", "head_pitch_joint"}) {
        EXPECT_NE(std::find(received->name.begin(), received->name.end(), joint), received->name.end());
    }
    const std::map<std::string, double> initial_angles = {{"right_front_coxa_joint", 5.0},
                                                          {"right_front_femur_joint", 25.0},
                                                          {"right_front_tibia_joint", -40.0},
                                                          {"head_pitch_joint", 7.0},
                                                          {"head_yaw_joint", -13.0}};
    for (const auto& [joint, angle] : initial_angles) {
        const auto found = std::find(received->name.begin(), received->name.end(), joint);
        ASSERT_NE(found, received->name.end());
        const auto index = std::distance(received->name.begin(), found);
        EXPECT_NEAR(received->position[index], angle * std::numbers::pi / 180.0, 1e-4) << joint;
    }
    EXPECT_TRUE(servo_controller_mock_->requests.empty());
}

TEST_F(RequesterTest, IdleDoesNotCommandServos) {
    requester_->update(100ms);
    EXPECT_TRUE(servo_controller_mock_->requests.empty());
}

TEST_F(RequesterTest, BodyPoseForwardsAllTargetsAndOnlySendsLatestCommand) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(test_helpers::defaultKinematicsParameters());
    auto reference_node = std::make_shared<rclcpp::Node>("reference", options);
    rumblex_geometry::CBodyModel reference(reference_node);
    reference.moveTorso(reference.getStandingToePositions(),
                        rumblex_geometry::CPose(0.005, 0.0, 0.0, 1.0, 2.0, 3.0));
    reference.setHeadOrientation(rumblex_geometry::COrientation(0.0, 12.0, -23.0));
    auto msg = rumblex_geometry::bodyPose(reference);
    requester_->onBodyPose(msg);
    reference.setHeadOrientation(rumblex_geometry::COrientation(0.0, 15.0, -20.0));
    msg = rumblex_geometry::bodyPose(reference);
    requester_->onBodyPose(msg);
    requester_->update(100ms);
    const auto& requests = servo_controller_mock_->requests;
    ASSERT_EQ(requests.size(), 1u);
    EXPECT_DOUBLE_EQ(requests.back().duration_s, 0.1);
    const auto expected =
        leg_servo_conversion::buildServoTargets(reference.getHeadOrientation(), reference.getLegAngles());
    ASSERT_EQ(requests.back().target_angles.size(), 20u);
    for (const auto& [index, angle] : expected) {
        EXPECT_NEAR(requests.back().target_angles.at(index), angle, 1e-3) << index;
    }
    requester_->update(100ms);
    EXPECT_EQ(requests.size(), 1u);
}

TEST_F(RequesterTest, RejectsInvalidPoseWithoutPartialApplication) {
    rumblex_interfaces::msg::BodyPose msg;
    msg.head_pose.yaw = std::numeric_limits<double>::quiet_NaN();
    requester_->onBodyPose(msg);
    requester_->update(100ms);
    EXPECT_TRUE(servo_controller_mock_->requests.empty());
}
