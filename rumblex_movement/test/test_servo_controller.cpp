#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "handler/offline_servo_protocol.hpp"
#include "handler/servo_controller.hpp"

using namespace rumblex_interfaces::msg;
using namespace rumblex_movement;

class ServoControllerTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) rclcpp::init(0, nullptr);
        rclcpp::NodeOptions options;
        // Only force offline mode; load real servo description from package config
        options.parameter_overrides({rclcpp::Parameter("SERVO_CONTROLLER_OFFLINE", true)});
        node_ = std::make_shared<rclcpp::Node>("test_servo_controller_node", rclcpp::NodeOptions(options));
        controller_ = std::make_unique<CServoController>(node_);
    }

    void TearDown() override {
        controller_.reset();
        rclcpp::shutdown();
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<CServoController> controller_;
};

TEST_F(ServoControllerTest, OfflineUsesMockAndWritesServoId) {
    // Prepare a direct request to write servo id for servo "HEAD_YAW" (serial id 10)
    ServoDirectRequest msg;
    msg.name = "HEAD_YAW";
    msg.cmd = ServoDirectRequest::SERVO_ID_WRITE;
    msg.data1 = 5;  // new id

    controller_->onServoDirectRequestReceived(msg);

    // The mock should have recorded the new mapping for serial 10 -> 5
    auto mock = COfflineServoProtocol::getLastInstance();
    ASSERT_NE(mock, nullptr);
    EXPECT_EQ(mock->getMappedId(10), static_cast<uint8_t>(5));
}

TEST_F(ServoControllerTest, SendServoRequestUpdatesMockPosition) {
    // Prepare a request that moves the first servo (index 0 -> serial id 10)
    ServoRequest req;
    req.time_to_reach_target_angles_ms = 1000;
    req.target_angles.resize(1);
    req.target_angles[0].angle_deg = 30.0;  // move to +30 degrees

    controller_->sendServoRequest(req);

    // Expected ticks for 30 degrees: ((30 + 120) / 6) * 25 = 625
    int16_t pos = 0;
    auto mock = COfflineServoProtocol::getLastInstance();
    ASSERT_NE(mock, nullptr);
    bool ok = mock->getPosition(static_cast<uint8_t>(10), pos);
    ASSERT_TRUE(ok);
    EXPECT_EQ(pos, static_cast<int16_t>(625));
}
