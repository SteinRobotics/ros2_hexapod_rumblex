#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <limits>
#include <tuple>

#include "handler/leg_servo_conversion.hpp"
#include "handler/servo_controller.hpp"
#include "rumblex_utils/body_model.hpp"

namespace rumblex_movement {
class ServoControllerTestAccess {
   public:
    static void protocol(CServoController& controller, std::shared_ptr<CServoProtocol> protocol) {
        controller.protocol_ = std::move(protocol);
    }
    static int ticks(CServoController& controller, double angle, int index) {
        return controller.angle_to_ticks(angle, index);
    }
    static double angle(CServoController& controller, int ticks, int index) {
        return controller.ticks_to_angle(ticks, index);
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

class NoxServoControllerTest : public ::testing::Test {
   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        rclcpp::NodeOptions options;
        options.arguments(
            {"--ros-args", "--params-file",
             (std::filesystem::path(RUMBLEX_CONFIG_DIR) / "nox/servo_description.yaml").string(),
             "--params-file", (std::filesystem::path(RUMBLEX_ANATOMY_DIR) / "nox/anatomy.yaml").string()});
        options.parameter_overrides({rclcpp::Parameter("servo.offline", true)});
        node = std::make_shared<rclcpp::Node>("node_movement", options);
        controller = std::make_unique<CServoController>(node);
        protocol = std::make_shared<RecordingProtocol>(node);
        ServoControllerTestAccess::protocol(*controller, protocol);
    }
    void TearDown() override {
        controller.reset();
        protocol.reset();
        node.reset();
        rclcpp::shutdown();
    }
    std::shared_ptr<rclcpp::Node> node;
    std::unique_ptr<CServoController> controller;
    std::shared_ptr<RecordingProtocol> protocol;
};

TEST_F(NoxServoControllerTest, YawCommandsPreservePhysicalZeroAndUseMigratedDirections) {
    using rumblex_interfaces::msg::ServoIndex;
    struct ExpectedServo {
        uint32_t index;
        uint8_t serial_id;
        int zero_ticks;
        int ticks_per_twelve_degrees;
    };
    // Physical zero ticks from Nox's original calibration. A 12-degree move is 50 ticks.
    const std::array<ExpectedServo, 7> servos = {{
        {ServoIndex::HEAD_YAW, 10, 479, 50},
        {ServoIndex::LEG_RIGHT_FRONT_COXA, 11, 500, -50},
        {ServoIndex::LEG_RIGHT_MID_COXA, 12, 500, -50},
        {ServoIndex::LEG_RIGHT_BACK_COXA, 13, 542, -50},
        {ServoIndex::LEG_LEFT_FRONT_COXA, 16, 500, -50},
        {ServoIndex::LEG_LEFT_MID_COXA, 15, 500, -50},
        {ServoIndex::LEG_LEFT_BACK_COXA, 14, 500, -50},
    }};
    for (const auto& servo : servos) {
        SCOPED_TRACE(servo.serial_id);
        EXPECT_EQ(ServoControllerTestAccess::ticks(*controller, 0.0, servo.index), servo.zero_ticks);
        for (const int direction : {-1, 1}) {
            const double target = direction * 12.0;
            const int expected_ticks = servo.zero_ticks + direction * servo.ticks_per_twelve_degrees;
            protocol->positions.clear();
            controller->requestAngles({{servo.index, target}}, 0.1);
            ASSERT_EQ(protocol->positions.size(), 1u);
            EXPECT_EQ(std::get<0>(protocol->positions.front()), servo.serial_id);
            EXPECT_EQ(std::get<1>(protocol->positions.front()), expected_ticks);
            // Startup feedback must use the same model convention, within half a tick.
            EXPECT_NEAR(ServoControllerTestAccess::angle(*controller, expected_ticks, servo.index), target,
                        0.12);
        }
    }
}

TEST_F(NoxServoControllerTest, ForwardToeTargetsDriveBothSidesInTheirPhysicalForwardDirection) {
    using namespace rumblex_geometry;
    CBodyModel model(node);
    const auto standing = model.getStandingToePositions();
    model.moveTorso(standing);
    for (const auto& mapping : leg_servo_conversion::LEG_SERVO_MAP) {
        if (mapping.axis != leg_servo_conversion::EJointAxis::TorsoCoxa) continue;
        SCOPED_TRACE(mapping.servo_index);
        const auto initial_angle = model.getLegAngles(mapping.leg).torso_coxa;
        auto forward = standing.at(mapping.leg);
        forward.x += 10.0 * units::mm;
        model.setToePosition(mapping.leg, forward);
        const auto target_angle = model.getLegAngles(mapping.leg).torso_coxa;
        const int initial_ticks = ServoControllerTestAccess::ticks(
            *controller, initial_angle.numerical_value_in(units::deg), mapping.servo_index);
        const int target_ticks = ServoControllerTestAccess::ticks(
            *controller, target_angle.numerical_value_in(units::deg), mapping.servo_index);
        // Nox's right coxae move forward with increasing ticks; left coxae with decreasing ticks.
        if (forward.y < 0.0 * units::m) {
            EXPECT_GT(target_ticks, initial_ticks);
        } else {
            EXPECT_LT(target_ticks, initial_ticks);
        }
    }
}
}  // namespace rumblex_movement
