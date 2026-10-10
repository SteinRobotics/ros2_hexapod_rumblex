/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "requester/error_management.hpp"

using namespace rumblex_interfaces::msg;

namespace brain {

CErrorManagement::Parameters CErrorManagement::Parameters::declare(std::shared_ptr<rclcpp::Node> node) {
    Parameters params;
    params.supply.nominal = node->declare_parameter<double>("supply_voltage") * units::V;
    params.supply.low = node->declare_parameter<double>("supply_voltage_low") * units::V;
    params.supply.critical_low = node->declare_parameter<double>("supply_voltage_critical_low") * units::V;
    params.servo.nominal = node->declare_parameter<double>("servo_voltage") * units::V;
    params.servo.low = node->declare_parameter<double>("servo_voltage_low") * units::V;
    params.servo.critical_low = node->declare_parameter<double>("servo_voltage_critical_low") * units::V;

    params.servo_temperature.high = units::celsius(node->declare_parameter<double>("servo_temperature_high"));
    params.servo_temperature.critical_high =
        units::celsius(node->declare_parameter<double>("servo_temperature_critical_high"));

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
    const auto temperature = units::celsius(msg.max_temperature);
    servo_temperature_filtered_ += 0.2 * (temperature - servo_temperature_filtered_);
    if (temperature > parameters_.servo_temperature.critical_high) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Servo temperature of "
                                                     << msg.servo_max_temperature << " is critical: "
                                                     << to_string_with_precision(msg.max_temperature, 0)
                                                     << "°C");

        return EError::TemperatureCriticalHigh;
    }
    if (temperature > parameters_.servo_temperature.high) {
        RCLCPP_WARN_STREAM(node_->get_logger(),
                           "Servo temperature of "
                               << msg.servo_max_temperature
                               << " is high: " << to_string_with_precision(msg.max_temperature, 0) << "°C");
        return EError::TemperatureHigh;
    }
    return EError::None;
}

units::Voltage CErrorManagement::getFilteredSupplyVoltage() {
    return supply_voltage_filtered_;
}

units::Voltage CErrorManagement::getFilteredServoVoltage() {
    return servo_voltage_filtered_;
}

units::Temperature CErrorManagement::getFilteredServoTemperature() {
    return servo_temperature_filtered_;
}

EError CErrorManagement::getStatusVoltage(units::Voltage voltage,
                                          const Parameters::VoltageGroup& thresholds) {
    if (voltage < thresholds.critical_low) {
        return EError::VoltageCriticalLow;
    }
    if (voltage < thresholds.low) {
        return EError::VoltageLow;
    }
    return EError::None;
}

EError CErrorManagement::filterSupplyVoltage(units::Voltage voltage) {
    supply_voltage_filtered_ = utils::lowPassFilter(supply_voltage_filtered_, voltage, 0.2);
    return getStatusVoltage(supply_voltage_filtered_, parameters_.supply);
}

EError CErrorManagement::filterServoVoltage(const ServoStatus& msg) {
    servo_voltage_filtered_ =
        utils::lowPassFilter(servo_voltage_filtered_, static_cast<double>(msg.max_voltage) * units::V, 0.2);
    return getStatusVoltage(servo_voltage_filtered_, parameters_.servo);
}

}  // namespace brain
