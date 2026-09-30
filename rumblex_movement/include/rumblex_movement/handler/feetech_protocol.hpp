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
    CFeetechProtocol(std::shared_ptr<rclcpp::Node> node, const std::string& device_name);
    ~CFeetechProtocol() override;

    bool triggerConnection() override;
    void closeConnection() override;
    bool isConnected() override;

    bool getServoID(uint8_t id, uint8_t& answer) override;
    bool setServoID(uint8_t id, uint8_t new_id) override;
    bool getMode(uint8_t id, uint8_t& mode) override;
    bool setServoMode(uint8_t id) override;
    bool setMotorMode(uint8_t id, int16_t speed = 0) override;
    bool setTorque(uint8_t id, bool active) override;

    bool setPosition(uint8_t id, uint16_t pos, uint16_t time_ms) override;
    bool setRegPos(uint8_t id, uint16_t pos, uint16_t time_ms) override;
    bool actionStart(uint8_t id = SERVO_Broadcast_ID) override;
    bool moveStop(uint8_t id = SERVO_Broadcast_ID) override;
    bool getPosition(uint8_t id, int16_t& pos) override;
    bool getPositionLimits(uint8_t id, uint16_t& min_position, uint16_t& max_position) override;
    bool setPositionLimits(uint8_t id, uint16_t min_position, uint16_t max_position) override;
    bool getPositionOffset(uint8_t id, int8_t& deviation) override;
    bool setPositionOffset(uint8_t id, int8_t deviation) override;
    bool savePositionOffset(uint8_t id) override;

    bool getMotorSpeed(uint8_t id, int16_t& speed) override;

    bool getVoltage(uint8_t id, uint16_t& m_v) override;
    bool getVoltageLimits(uint8_t id, uint16_t& m_vmin, uint16_t& m_vmax) override;
    bool setVoltageLimits(uint8_t id, uint16_t m_vmin, uint16_t m_vmax) override;

    bool getTemperature(uint8_t id, uint8_t& deg_temp) override;
    bool getMaxTemperatureLimit(uint8_t id, uint8_t& deg_limit) override;
    bool setMaxTemperatureLimit(uint8_t id, uint8_t deg_limit) override;

    bool isLedOn(uint8_t id) override;
    bool setLed(uint8_t id, bool on) override;

    bool getLedErrcode(uint8_t id, uint8_t& lederrcode) override;
    bool flashLedErrCode(uint8_t id, uint8_t code) override;

   private:
    uint16_t convertTicksToFeetech(uint16_t ticks) const;
    uint16_t convertFeetechToTicks(uint16_t feetech_ticks) const;

    std::string device_name_;
    bool is_connected_ = false;
    HLSCL hlscl_;
};

}  // namespace rumblex_movement
