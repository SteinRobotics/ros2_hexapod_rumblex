#include <gtest/gtest.h>

#include "handler/servohandler.hpp"
#include "mock/mock_servohandler.hpp"
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
        overrides.emplace_back("SERVO_CONTROLLER_OFFLINE", true);
        overrides.emplace_back("STANDING_FOOT_POS_X",
                               std::vector<double>{0.092, 0.0, -0.092, 0.092, 0.0, -0.092});
        overrides.emplace_back("STANDING_FOOT_POS_Y",
                               std::vector<double>{0.092, 0.130, 0.092, -0.092, -0.130, -0.092});
        overrides.emplace_back("STANDING_FOOT_POS_Z",
                               std::vector<double>{-0.050, -0.050, -0.050, -0.050, -0.050, -0.050});
        overrides.emplace_back("LAYDOWN_FOOT_POS_X",
                               std::vector<double>{0.071, 0.0, -0.071, 0.071, 0.0, -0.071});
        overrides.emplace_back("LAYDOWN_FOOT_POS_Y",
                               std::vector<double>{0.071, 0.100, 0.071, -0.071, -0.100, -0.071});
        overrides.emplace_back("LAYDOWN_FOOT_POS_Z",
                               std::vector<double>{0.010, 0.010, 0.010, 0.010, 0.010, 0.010});
        options.parameter_overrides(overrides);
        node_ = std::make_shared<rclcpp::Node>("test_requester_node", options);
        servoHandlerMock_ = std::make_shared<CServoHandlerMock>(node_);
        requester_ = std::make_unique<CRequester>(node_, servoHandlerMock_);
    }

    void TearDown() override {
        requester_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<CRequester> requester_;
    std::shared_ptr<CServoHandlerMock> servoHandlerMock_;
};

TEST_F(RequesterTest, ConstructAndUpdate) {
    // basic smoke test: update should run without throwing
    EXPECT_NO_THROW(requester_->update(std::chrono::milliseconds(0)));
}

TEST_F(RequesterTest, HandleNoRequestMessage) {
    // Create an empty MovementRequest message with NO_REQUEST and pass it to the handler
    rumblex_interfaces::msg::MovementRequest msg;
    msg.name = "";  // empty name indicates NO_REQUEST in the code path
    // MovementRequest uses the field 'type' to indicate the request kind
    msg.type = rumblex_interfaces::msg::MovementRequest::NO_REQUEST;

    EXPECT_NO_THROW(requester_->onMovementRequest(msg));
}
