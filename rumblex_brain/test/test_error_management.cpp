#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "requester/error_management.hpp"

using brain::CErrorManagement;
using brain::EError;

class ErrorManagementTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        rclcpp::NodeOptions options;
        options.parameter_overrides(getParameterOverrides());
        node_ = std::make_shared<rclcpp::Node>("test_error_management_node", options);
        manager_ = std::make_unique<CErrorManagement>(node_);
    }

    void TearDown() override {
        manager_.reset();
        node_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    static std::vector<rclcpp::Parameter> getParameterOverrides() {
        return {
            rclcpp::Parameter("supply_voltage", 15.0),
            rclcpp::Parameter("supply_voltage_low", 14.9),
            rclcpp::Parameter("supply_voltage_critical_low", 14.0),
            rclcpp::Parameter("servo_voltage", 12.0),
            rclcpp::Parameter("servo_voltage_low", 11.0),
            rclcpp::Parameter("servo_voltage_critical_low", 10.0),
            rclcpp::Parameter("servo_temperature_high", 60.0),
            rclcpp::Parameter("servo_temperature_critical_high", 80.0),
        };
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<CErrorManagement> manager_;
};

TEST_F(ErrorManagementTest, DetectsHighServoTemperature) {
    rumblex_interfaces::msg::ServoStatus msg;
    msg.servo_max_temperature = "servo_1";
    msg.max_temperature = 65.0f;  // above 60 high threshold, below 80 critical

    auto error = manager_->getErrorServo(msg);
    EXPECT_EQ(error, EError::TemperatureHigh);
}

TEST_F(ErrorManagementTest, FiltersSupplyVoltageToLowState) {
    auto error = manager_->filterSupplyVoltage(12.0f);  // pulls filtered value below low threshold
    EXPECT_EQ(error, EError::VoltageLow);
    EXPECT_LT(manager_->getFilteredSupplyVoltage(), 14.9f);
    EXPECT_GT(manager_->getFilteredSupplyVoltage(), 14.0f);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
