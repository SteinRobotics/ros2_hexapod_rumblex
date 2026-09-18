/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#pragma once

#include "magic_enum.hpp"
#include "rumblex_interfaces/msg/servo_status.hpp"
#include "rumblex_utils/filters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/utility.hpp"

namespace brain {

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
    EError filterSupplyVoltage(double voltage);
    std::string getErrorName(EError error) {
        return std::string(magic_enum::enum_name(error));
    }
    double getFilteredSupplyVoltage();
    double getFilteredServoVoltage();
    double getFilteredServoTemperature();

   private:
    struct Parameters {
        struct VoltageGroup {
            double nominal{0.0};
            double low{0.0};
            double critical_low{0.0};
        };

        struct TemperatureGroup {
            double high{0.0};
            double critical_high{0.0};
        };

        VoltageGroup supply;
        VoltageGroup servo;
        TemperatureGroup servo_temperature;

        static Parameters declare(std::shared_ptr<rclcpp::Node> node);
    };

    EError filterServoVoltage(const rumblex_interfaces::msg::ServoStatus& msg);
    EError getStatusServoTemperature(const rumblex_interfaces::msg::ServoStatus& msg);
    EError getStatusVoltage(double voltage, const Parameters::VoltageGroup& thresholds);

    std::shared_ptr<rclcpp::Node> node_;
    Parameters parameters_;
    double supply_voltage_filtered_ = 0.0;
    double servo_voltage_filtered_ = 0.0;
    double servo_temperature_filtered_ = 0.0;
};

}  // namespace brain
