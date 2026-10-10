#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "requester/error_management.hpp"

using brain::CErrorManagement;
using brain::EError;
namespace units = rumblex_geometry::units;

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
    auto error = manager_->filterSupplyVoltage(12.0 * units::V);  // pulls filtered value below low threshold
    EXPECT_EQ(error, EError::VoltageLow);
    EXPECT_LT(manager_->getFilteredSupplyVoltage().numerical_value_in(units::V), 14.9f);
    EXPECT_GT(manager_->getFilteredSupplyVoltage().numerical_value_in(units::V), 14.0f);
}

TEST_F(ErrorManagementTest, TemperatureFilteringKeepsCelsiusOriginAndRawThresholds) {
    EXPECT_DOUBLE_EQ(units::inCelsius(manager_->getFilteredServoTemperature()), 0.0);
    rumblex_interfaces::msg::ServoStatus msg;
    msg.max_voltage = 12.0;
    msg.max_temperature = 60.0;
    EXPECT_EQ(manager_->getErrorServo(msg), EError::None);
    EXPECT_DOUBLE_EQ(units::inCelsius(manager_->getFilteredServoTemperature()), 12.0);
    const auto previous_voltage = manager_->getFilteredServoVoltage();
    msg.max_temperature = 80.0;
    EXPECT_EQ(manager_->getErrorServo(msg), EError::TemperatureHigh);
    EXPECT_NEAR(units::inCelsius(manager_->getFilteredServoTemperature()), 25.6, 1e-12);
    msg.max_temperature = 81.0;
    msg.max_voltage = 0.0;
    EXPECT_EQ(manager_->getErrorServo(msg), EError::TemperatureCriticalHigh);
    // Temperature errors take priority and skip the voltage filter.
    EXPECT_EQ(manager_->getFilteredServoVoltage(), previous_voltage);
}

TEST_F(ErrorManagementTest, VoltageFilteringPreservesInitializationAndCriticalThreshold) {
    EXPECT_EQ(manager_->getFilteredSupplyVoltage(), 15.0 * units::V);
    EXPECT_EQ(manager_->getFilteredServoVoltage(), 12.0 * units::V);
    EXPECT_EQ(manager_->filterSupplyVoltage(12.0 * units::V), EError::VoltageLow);
    EXPECT_NEAR(manager_->getFilteredSupplyVoltage().numerical_value_in(units::V), 14.4, 1e-12);
    EXPECT_EQ(manager_->filterSupplyVoltage(12.0 * units::V), EError::VoltageCriticalLow);
    EXPECT_NEAR(manager_->getFilteredSupplyVoltage().numerical_value_in(units::V), 13.92, 1e-12);
    rumblex_interfaces::msg::ServoStatus msg;
    msg.max_voltage = 0.0;
    EXPECT_EQ(manager_->getErrorServo(msg), EError::VoltageCriticalLow);
    EXPECT_NEAR(manager_->getFilteredServoVoltage().numerical_value_in(units::V), 9.6, 1e-12);
}

TEST_F(ErrorManagementTest, VoltageThresholdEqualityIsNotCritical) {
    for (bool at_critical : {false, true}) {
        auto parameters = getParameterOverrides();
        for (auto& parameter : parameters) {
            if (parameter.get_name() == "supply_voltage_low")
                parameter = rclcpp::Parameter("supply_voltage_low", at_critical ? 16.0 : 15.0);
            if (parameter.get_name() == "supply_voltage_critical_low")
                parameter = rclcpp::Parameter("supply_voltage_critical_low", at_critical ? 15.0 : 14.0);
        }
        rclcpp::NodeOptions options;
        options.parameter_overrides(parameters);
        auto node = std::make_shared<rclcpp::Node>("voltage_equality", options);
        CErrorManagement manager(node);
        EXPECT_EQ(manager.filterSupplyVoltage(15.0 * units::V),
                  at_critical ? EError::VoltageLow : EError::None);
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
