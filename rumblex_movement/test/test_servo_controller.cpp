#include <gtest/gtest.h>

#include <limits>
#include <tuple>

#include "handler/servo_controller.hpp"

namespace rumblex_movement {
class ServoControllerTestAccess {
   public:
    static void protocol(CServoController& controller, std::shared_ptr<CServoProtocol> protocol) {
        controller.protocol_ = std::move(protocol);
    }
};

class RecordingProtocol : public CServoProtocol {
   public:
    explicit RecordingProtocol(std::shared_ptr<rclcpp::Node> node) : CServoProtocol(node) {
    }
    bool setRegPos(uint8_t id, uint16_t ticks, uint16_t duration_ms) override {
        positions.emplace_back(id, ticks, duration_ms);
        return !fail;
    }
    bool actionStart(uint8_t) override {
        ++actions;
        return true;
    }
    std::vector<std::tuple<uint8_t, uint16_t, uint16_t>> positions;
    int actions = 0;
    bool fail = false;
};

class ServoControllerTest : public ::testing::Test {
   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        rclcpp::NodeOptions options;
        options.parameter_overrides(
            {rclcpp::Parameter("servo.offline", true),
             rclcpp::Parameter("servo.names", std::vector<std::string>{"HEAD_YAW"}),
             rclcpp::Parameter("servo.adaptation_deg", std::vector<double>{0.0}),
             rclcpp::Parameter("servo.offset_deg", std::vector<double>{0.0}),
             rclcpp::Parameter("servo.orientation_clockwise", std::vector<bool>{true}),
             rclcpp::Parameter("servo.serial_ids", std::vector<int64_t>{1})});
        node = std::make_shared<rclcpp::Node>("servo_resolution_test", options);
        controller = std::make_unique<CServoController>(node);
        protocol = std::make_shared<RecordingProtocol>(node);
        ServoControllerTestAccess::protocol(*controller, protocol);
    }
    void TearDown() override {
        controller.reset();
        rclcpp::shutdown();
    }
    std::shared_ptr<rclcpp::Node> node;
    std::unique_ptr<CServoController> controller;
    std::shared_ptr<RecordingProtocol> protocol;
};

TEST_F(ServoControllerTest, SmallMovesUseOneTickRatherThanHalfDegreeSteps) {
    controller->requestAngles({{0, 0.1}}, 0.02);
    EXPECT_TRUE(protocol->positions.empty());
    EXPECT_EQ(protocol->actions, 0);
    controller->requestAngles({{0, 0.13}}, 0.02);
    ASSERT_EQ(protocol->positions.size(), 1u);
    EXPECT_EQ(std::get<1>(protocol->positions.back()), 501);
    EXPECT_EQ(std::get<2>(protocol->positions.back()), 20);
    controller->requestAngles({{0, 0.26}}, 0.02);
    EXPECT_EQ(protocol->positions.size(), 1u);
    controller->requestAngles({{0, 0.37}}, 0.02);
    ASSERT_EQ(protocol->positions.size(), 2u);
    EXPECT_EQ(std::get<1>(protocol->positions.back()), 502);
    controller->requestAngles({{0, -0.13}}, 0.02);
    ASSERT_EQ(protocol->positions.size(), 3u);
    EXPECT_EQ(std::get<1>(protocol->positions.back()), 499);
    EXPECT_EQ(protocol->actions, 3);
}

TEST_F(ServoControllerTest, FailedWritesCanRetryAndInvalidCommandsDoNotStartActions) {
    protocol->fail = true;
    controller->requestAngles({{0, 0.24}}, 0.02);
    EXPECT_EQ(protocol->actions, 0);
    protocol->fail = false;
    controller->requestAngles({{0, 0.24}}, 0.02);
    EXPECT_EQ(protocol->positions.size(), 2u);
    EXPECT_EQ(protocol->actions, 1);
    controller->requestAngles({{0, std::numeric_limits<double>::quiet_NaN()}}, 0.02);
    controller->requestAngles({{0, 10.0}}, -0.02);
    EXPECT_EQ(protocol->positions.size(), 2u);
    EXPECT_EQ(protocol->actions, 1);
}

TEST_F(ServoControllerTest, SmallFrameDurationsKeepServoSpeedLimit) {
    controller->requestAngles({{0, 10.08}}, 0.02);
    ASSERT_EQ(protocol->positions.size(), 1u);
    EXPECT_EQ(std::get<2>(protocol->positions.back()), 31);
}
}  // namespace rumblex_movement
