/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace rumblex_movement {

constexpr uint8_t SERVO_Broadcast_ID = 0xFE;

class CServoProtocol {
   public:
    explicit CServoProtocol(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    }
    virtual ~CServoProtocol() = default;

    // Connection
    virtual bool triggerConnection() {
        return false;
    }
    virtual void closeConnection() {
    }
    virtual bool isConnected() {
        return false;
    }

    // Generic Commands
    virtual bool getServoID(uint8_t /*ID*/, uint8_t& /*answer*/) {
        return false;
    }
    virtual bool setServoID(uint8_t /*ID*/, uint8_t /*newID*/) {  // set new ID for servo, 0..254
        return false;
    }
    virtual bool getMode(uint8_t /*ID*/, uint8_t& /*mode*/) {  // get servo mode, 0 = servo, 1 = motor
        return false;
    }
    virtual bool setServoMode(uint8_t /*ID*/) {  // set servo mode
        return false;
    }
    virtual bool setMotorMode(uint8_t /*ID*/, int16_t /*speed*/ = 0) {
        return false;
    }
    virtual bool setTorque(uint8_t /*ID*/, bool /*active*/) {  // set motor torque active
        return false;
    }

    // Position
    virtual bool setPosition(uint8_t /*ID*/, uint16_t /*pos*/, uint16_t /*timeMs*/) {
        return false;
    }
    virtual bool setRegPos(uint8_t /*ID*/, uint16_t /*pos*/, uint16_t /*timeMs*/) {
        return false;
    }
    virtual bool actionStart(uint8_t /*ID*/ = SERVO_Broadcast_ID) {
        return false;
    }
    virtual bool moveStop(uint8_t /*ID*/ = SERVO_Broadcast_ID) {
        return false;
    }
    virtual bool getPosition(uint8_t /*ID*/, int16_t& /*pos*/) {
        return false;
    }
    virtual bool getPositionLimits(uint8_t /*ID*/, uint16_t& /*min_position*/, uint16_t& /*max_position*/) {
        return false;
    }
    virtual bool setPositionLimits(uint8_t /*ID*/, uint16_t /*min_position*/, uint16_t /*max_position*/) {
        return false;
    }
    virtual bool getPositionOffset(uint8_t /*ID*/, int8_t& /*deviation*/) {
        return false;
    }
    virtual bool setPositionOffset(uint8_t /*ID*/, int8_t /*deviation*/) {
        return false;
    }
    virtual bool savePositionOffset(uint8_t /*ID*/) {
        return false;
    }

    // Motor Control
    virtual bool getMotorSpeed(uint8_t /*ID*/, int16_t& /*speed*/) {
        return false;
    }

    // Voltage
    virtual bool getVoltage(uint8_t /*ID*/, uint16_t& /*mV*/) {
        return false;
    }
    virtual bool getVoltageLimits(uint8_t /*ID*/, uint16_t& /*mVmin*/, uint16_t& /*mVmax*/) {
        return false;
    }
    virtual bool setVoltageLimits(uint8_t /*ID*/, uint16_t /*mVmin*/, uint16_t /*mVmax*/) {
        return false;
    }

    // Temperature
    virtual bool getTemperature(uint8_t /*ID*/, uint8_t& /*degTemp*/) {
        return false;
    }
    virtual bool getMaxTemperatureLimit(uint8_t /*ID*/, uint8_t& /*degLimit*/) {
        return false;
    }
    virtual bool setMaxTemperatureLimit(uint8_t /*ID*/, uint8_t /*degLimit*/) {
        return false;
    }

    // LED
    virtual bool isLedOn(uint8_t /*ID*/) {
        return false;
    }
    virtual bool setLed(uint8_t /*ID*/, bool /*on*/) {
        return false;
    }

    // Error handling
    // get LED error code, b0=temp, b1:voltage, b2:stalled -> also clears errors?
    virtual bool getLedErrcode(uint8_t /*ID*/, uint8_t& /*lederrcode*/) {
        return false;
    }
    virtual bool flashLedErrCode(uint8_t /*ID*/, uint8_t /*code*/) {
        return false;
    }

   protected:
    std::shared_ptr<rclcpp::Node> node_;
};

}  // namespace rumblex_movement
