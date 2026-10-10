/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#pragma once

#include <magic_enum/magic_enum.hpp>

#include "rclcpp/rclcpp.hpp"
#include "requester/utility.hpp"
#include "rumblex_interfaces/msg/servo_status.hpp"
#include "rumblex_utils/filters.hpp"
#include "rumblex_utils/units.hpp"

namespace brain {
namespace units = rumblex_geometry::units;

enum class EError {
    None = 0,
    VoltageLow = 1,
    VoltageCriticalLow = 2,
    VoltageHigh = 3,
    TemperatureHigh = 4,
    TemperatureCriticalHigh = 5,
};

class CErrorManagement {
   public:
    CErrorManagement(std::shared_ptr<rclcpp::Node> node);
    virtual ~CErrorManagement() = default;

    EError getErrorServo(const rumblex_interfaces::msg::ServoStatus& msg);
    EError filterSupplyVoltage(units::Voltage voltage);
    std::string getErrorName(EError error) {
        return std::string(magic_enum::enum_name(error));
    }
    units::Voltage getFilteredSupplyVoltage();
    units::Voltage getFilteredServoVoltage();
    units::Temperature getFilteredServoTemperature();

   private:
    struct Parameters {
        struct VoltageGroup {
            units::Voltage nominal = 0.0 * units::V;
            units::Voltage low = 0.0 * units::V;
            units::Voltage critical_low = 0.0 * units::V;
        };

        struct TemperatureGroup {
            units::Temperature high = units::celsius(0.0);
            units::Temperature critical_high = units::celsius(0.0);
        };

        VoltageGroup supply;
        VoltageGroup servo;
        TemperatureGroup servo_temperature;

        static Parameters declare(std::shared_ptr<rclcpp::Node> node);
    };

    EError filterServoVoltage(const rumblex_interfaces::msg::ServoStatus& msg);
    EError getStatusServoTemperature(const rumblex_interfaces::msg::ServoStatus& msg);
    EError getStatusVoltage(units::Voltage voltage, const Parameters::VoltageGroup& thresholds);

    std::shared_ptr<rclcpp::Node> node_;
    Parameters parameters_;
    units::Voltage supply_voltage_filtered_ = 0.0 * units::V;
    units::Voltage servo_voltage_filtered_ = 0.0 * units::V;
    units::Temperature servo_temperature_filtered_ = units::celsius(0.0);
};

}  // namespace brain
