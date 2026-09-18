/*******************************************************************************
 * Copyright (c) 2026 Christian Stein
 ******************************************************************************/

#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "handler/feetech/HLSCL.h"
#include "handler/servo_protocol.hpp"

namespace rumblex_movement {

class CFeetechProtocol : public CServoProtocol {
   public:
    CFeetechProtocol(std::shared_ptr<rclcpp::Node> node, const std::string& deviceName);
    ~CFeetechProtocol() override;

    bool triggerConnection() override;
    void closeConnection() override;
    bool isConnected() override;

    bool getServoID(uint8_t ID, uint8_t& answer) override;
    bool setServoID(uint8_t ID, uint8_t newID) override;
    bool getMode(uint8_t ID, uint8_t& mode) override;
    bool setServoMode(uint8_t ID) override;
    bool setMotorMode(uint8_t ID, int16_t speed = 0) override;
    bool setTorque(uint8_t ID, bool active) override;

    bool setPosition(uint8_t ID, uint16_t pos, uint16_t timeMs) override;
    bool setRegPos(uint8_t ID, uint16_t pos, uint16_t timeMs) override;
    bool actionStart(uint8_t ID = SERVO_Broadcast_ID) override;
    bool moveStop(uint8_t ID = SERVO_Broadcast_ID) override;
    bool getPosition(uint8_t ID, int16_t& pos) override;
    bool getPositionLimits(uint8_t ID, uint16_t& min_position, uint16_t& max_position) override;
    bool setPositionLimits(uint8_t ID, uint16_t min_position, uint16_t max_position) override;
    bool getPositionOffset(uint8_t ID, int8_t& deviation) override;
    bool setPositionOffset(uint8_t ID, int8_t deviation) override;
    bool savePositionOffset(uint8_t ID) override;

    bool getMotorSpeed(uint8_t ID, int16_t& speed) override;

    bool getVoltage(uint8_t ID, uint16_t& mV) override;
    bool getVoltageLimits(uint8_t ID, uint16_t& mVmin, uint16_t& mVmax) override;
    bool setVoltageLimits(uint8_t ID, uint16_t mVmin, uint16_t mVmax) override;

    bool getTemperature(uint8_t ID, uint8_t& degTemp) override;
    bool getMaxTemperatureLimit(uint8_t ID, uint8_t& degLimit) override;
    bool setMaxTemperatureLimit(uint8_t ID, uint8_t degLimit) override;

    bool isLedOn(uint8_t ID) override;
    bool setLed(uint8_t ID, bool on) override;

    bool getLedErrcode(uint8_t ID, uint8_t& lederrcode) override;
    bool flashLedErrCode(uint8_t ID, uint8_t code) override;

   private:
    uint16_t convertTicksToFeetech(uint16_t ticks) const;
    uint16_t convertFeetechToTicks(uint16_t feetech_ticks) const;

    std::string deviceName_;
    bool isConnected_ = false;
    HLSCL hlscl_;
};

}  // namespace rumblex_movement
