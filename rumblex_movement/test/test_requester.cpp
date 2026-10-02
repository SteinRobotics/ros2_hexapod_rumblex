#include <gtest/gtest.h>

#include <algorithm>
#include <numbers>
#include <thread>

#include "handler/servo_handler.hpp"
#include "mock/mock_servo_handler.hpp"
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
        auto overrides = test_helpers::defaultRobotParameters();
        overrides.emplace_back("servo.offline", true);
        options.parameter_overrides(overrides);
        node_ = std::make_shared<rclcpp::Node>("test_requester_node", options);
        servo_handler_mock_ = std::make_shared<CServoHandlerMock>(node_);
        requester_ = std::make_unique<CRequester>(node_, servo_handler_mock_);
    }

    void TearDown() override {
        requester_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<CRequester> requester_;
    std::shared_ptr<CServoHandlerMock> servo_handler_mock_;
};

TEST_F(RequesterTest, ConstructAndUpdate) {
    // basic smoke test: update should run without throwing
    EXPECT_NO_THROW(requester_->update(std::chrono::milliseconds(0)));
}

TEST_F(RequesterTest, PublishesInitialJointStatesWithoutMovementRequest) {
    sensor_msgs::msg::JointState::SharedPtr received;
    auto subscriber = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        [&received](sensor_msgs::msg::JointState::SharedPtr message) { received = message; });

    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!received && std::chrono::steady_clock::now() < deadline) {
        requester_->update(100ms);
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(10ms);
    }

    ASSERT_NE(received, nullptr);
    EXPECT_EQ(received->name.size(), 20u);
    EXPECT_EQ(received->position.size(), received->name.size());
    for (const auto* joint : {"right_front_coxa_joint", "right_front_femur_joint", "right_front_tibia_joint",
                              "head_yaw_joint", "head_pitch_joint"}) {
        EXPECT_NE(std::find(received->name.begin(), received->name.end(), joint), received->name.end());
    }
    const auto femur = std::find(received->name.begin(), received->name.end(), "right_front_femur_joint");
    ASSERT_NE(femur, received->name.end());
    const auto index = std::distance(received->name.begin(), femur);
    EXPECT_NEAR(received->position[index], 70.723 * std::numbers::pi / 180.0, 1e-4);
}

TEST_F(RequesterTest, IdleDoesNotCommandServos) {
    requester_->update(100ms);
    EXPECT_TRUE(servo_handler_mock_->getRequests().empty());
}

TEST_F(RequesterTest, BodyPoseForwardsAllTargetsAndOnlySendsLatestCommand) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(test_helpers::defaultKinematicsParameters());
    auto reference_node = std::make_shared<rclcpp::Node>("reference", options);
    CKinematics reference(reference_node);
    reference.moveTorso(reference.getStandingToePositions(), CPose(0.005, 0.0, 0.0, 1.0, 2.0, 3.0));
    reference.setHeadOrientation(COrientation(0.0, 12.0, -23.0));
    auto msg = bodyPose(reference);
    requester_->onBodyPose(msg);
    reference.setHeadOrientation(COrientation(0.0, 15.0, -20.0));
    msg = bodyPose(reference);
    requester_->onBodyPose(msg);
    requester_->update(100ms);
    auto requests = servo_handler_mock_->getRequests();
    ASSERT_EQ(requests.size(), 1u);
    EXPECT_DOUBLE_EQ(requests.back()->duration(), 0.1);
    expectHeadNear(reference.getHeadOrientation(), requests.back()->getHeadOrientation());
    for (auto leg : bodyLegOrder)
        expectAnglesNear(reference.getLegAngles(leg), requests.back()->getLegAngles().at(leg));
    requester_->update(100ms);
    EXPECT_EQ(servo_handler_mock_->getRequests().size(), 1u);
}

TEST_F(RequesterTest, RejectsInvalidPoseWithoutPartialApplication) {
    rumblex_interfaces::msg::BodyPose msg;
    msg.head_pose.yaw = std::numeric_limits<double>::quiet_NaN();
    requester_->onBodyPose(msg);
    requester_->update(100ms);
    EXPECT_TRUE(servo_handler_mock_->getRequests().empty());
}
