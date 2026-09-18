/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "requester/error_management.hpp"

using namespace rumblex_interfaces::msg;

namespace brain {

CErrorManagement::Parameters CErrorManagement::Parameters::declare(std::shared_ptr<rclcpp::Node> node) {
    Parameters params;
    params.supply.nominal = node->declare_parameter<double>("supply_voltage");
    params.supply.low = node->declare_parameter<double>("supply_voltage_low");
    params.supply.critical_low = node->declare_parameter<double>("supply_voltage_critical_low");
    params.servo.nominal = node->declare_parameter<double>("servo_voltage");
    params.servo.low = node->declare_parameter<double>("servo_voltage_low");
    params.servo.critical_low = node->declare_parameter<double>("servo_voltage_critical_low");

    params.servo_temperature.high = node->declare_parameter<double>("servo_temperature_high");
    params.servo_temperature.critical_high =
        node->declare_parameter<double>("servo_temperature_critical_high");

    return params;
}

CErrorManagement::CErrorManagement(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    parameters_ = Parameters::declare(node_);
    supply_voltage_filtered_ = parameters_.supply.nominal;
    servo_voltage_filtered_ = parameters_.servo.nominal;
}

EError CErrorManagement::getErrorServo(const ServoStatus& msg) {
    EError error = getStatusServoTemperature(msg);
    if (error != EError::None) {
        return error;
    }
    error = filterServoVoltage(msg);
    if (error != EError::None) {
        return error;
    }
    return EError::None;
}

// private methods:
EError CErrorManagement::getStatusServoTemperature(const ServoStatus& msg) {
    servo_temperature_filtered_ =
        utils::lowPassFilter(servo_temperature_filtered_, static_cast<double>(msg.max_temperature), 0.2);
    if (msg.max_temperature > parameters_.servo_temperature.critical_high) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Servo temperature of "
                                                     << msg.servo_max_temperature << " is critical: "
                                                     << to_string_with_precision(msg.max_temperature, 0)
                                                     << "°C");

        return EError::TemperatureCriticalHigh;
    }
    if (msg.max_temperature > parameters_.servo_temperature.high) {
        RCLCPP_WARN_STREAM(node_->get_logger(),
                           "Servo temperature of "
                               << msg.servo_max_temperature
                               << " is high: " << to_string_with_precision(msg.max_temperature, 0) << "°C");
        return EError::TemperatureHigh;
    }
    return EError::None;
}

double CErrorManagement::getFilteredSupplyVoltage() {
    return supply_voltage_filtered_;
}

double CErrorManagement::getFilteredServoVoltage() {
    return servo_voltage_filtered_;
}

double CErrorManagement::getFilteredServoTemperature() {
    return servo_temperature_filtered_;
}

EError CErrorManagement::getStatusVoltage(double voltage, const Parameters::VoltageGroup& thresholds) {
    if (voltage < thresholds.critical_low) {
        return EError::VoltageCriticalLow;
    }
    if (voltage < thresholds.low) {
        return EError::VoltageLow;
    }
    return EError::None;
}

EError CErrorManagement::filterSupplyVoltage(double voltage) {
    supply_voltage_filtered_ = utils::lowPassFilter(supply_voltage_filtered_, voltage, 0.2);
    return getStatusVoltage(supply_voltage_filtered_, parameters_.supply);
}

EError CErrorManagement::filterServoVoltage(const ServoStatus& msg) {
    servo_voltage_filtered_ =
        utils::lowPassFilter(servo_voltage_filtered_, static_cast<double>(msg.max_voltage), 0.2);
    return getStatusVoltage(servo_voltage_filtered_, parameters_.servo);
}

}  // namespace brain
