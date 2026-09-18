/*******************************************************************************
 * Copyright (c) 2026 Christian Stein
 ******************************************************************************/

#include "handler/feetech_protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace rumblex_movement {

CFeetechProtocol::CFeetechProtocol(std::shared_ptr<rclcpp::Node> node, const std::string& deviceName)
    : CServoProtocol(node), deviceName_(deviceName) {
}

CFeetechProtocol::~CFeetechProtocol() {
    closeConnection();
}

bool CFeetechProtocol::triggerConnection() {
    isConnected_ = false;
    RCLCPP_INFO(node_->get_logger(),
                "CFeetechProtocol: Connecting to serial port %s with 115200 baud Rate...",
                deviceName_.c_str());
    if (hlscl_.begin(115200, deviceName_.c_str())) {
        isConnected_ = true;
        RCLCPP_INFO(node_->get_logger(), "CFeetechProtocol: Connection successful");
        return true;
    }
    RCLCPP_ERROR(node_->get_logger(), "CFeetechProtocol: Connection failed on port %s", deviceName_.c_str());
    return false;
}

void CFeetechProtocol::closeConnection() {
    if (isConnected_) {
        hlscl_.end();
        isConnected_ = false;
    }
}

bool CFeetechProtocol::isConnected() {
    return isConnected_;
}

bool CFeetechProtocol::getServoID(uint8_t ID, uint8_t& answer) {
    if (!isConnected_) return false;
    int res = hlscl_.Ping(ID);
    if (res != -1) {
        answer = static_cast<uint8_t>(res);
        return true;
    }
    return false;
}

bool CFeetechProtocol::setServoID(uint8_t ID, uint8_t newID) {
    if (!isConnected_) return false;
    hlscl_.unLockEprom(ID);
    int res = hlscl_.writeByte(ID, HLSCL_ID, newID);
    hlscl_.LockEprom(newID);
    return res != -1;
}

bool CFeetechProtocol::getMode(uint8_t ID, uint8_t& mode) {
    if (!isConnected_) return false;
    int res = hlscl_.readByte(ID, HLSCL_MODE);
    if (res != -1) {
        mode = static_cast<uint8_t>(res);
        return true;
    }
    return false;
}

bool CFeetechProtocol::setServoMode(uint8_t ID) {
    if (!isConnected_) return false;
    return hlscl_.ServoMode(ID) != -1;
}

bool CFeetechProtocol::setMotorMode(uint8_t ID, int16_t speed) {
    if (!isConnected_) return false;
    if (hlscl_.WheelMode(ID) == -1) return false;
    return hlscl_.WriteSpe(ID, speed, 50, 500) != -1;
}

bool CFeetechProtocol::setTorque(uint8_t ID, bool active) {
    if (!isConnected_) return false;
    return hlscl_.EnableTorque(ID, active ? 1 : 0) != -1;
}

bool CFeetechProtocol::setPosition(uint8_t ID, uint16_t pos, uint16_t timeMs) {
    if (!isConnected_) return false;
    int current_pos = hlscl_.ReadPos(ID);
    if (current_pos == -1) {
        current_pos = 2048;  // neutral
    }
    uint16_t target_pos = convertTicksToFeetech(pos);
    uint16_t delta = std::abs(current_pos - static_cast<int>(target_pos));

    uint16_t speed = 0;
    if (timeMs > 0 && delta > 0) {
        speed = static_cast<uint16_t>(std::round((delta * 20.0) / timeMs));
        if (speed < 1) speed = 1;
        if (speed > 250) speed = 250;
    } else {
        speed = 50;  // default safe speed
    }

    return hlscl_.WritePosEx(ID, static_cast<s16>(target_pos), speed, 50, 500) != -1;
}

bool CFeetechProtocol::setRegPos(uint8_t ID, uint16_t pos, uint16_t timeMs) {
    if (!isConnected_) return false;
    int current_pos = hlscl_.ReadPos(ID);
    if (current_pos == -1) {
        current_pos = 2048;  // neutral
    }
    uint16_t target_pos = convertTicksToFeetech(pos);
    uint16_t delta = std::abs(current_pos - static_cast<int>(target_pos));

    uint16_t speed = 0;
    if (timeMs > 0 && delta > 0) {
        speed = static_cast<uint16_t>(std::round((delta * 20.0) / timeMs));
        if (speed < 1) speed = 1;
        if (speed > 250) speed = 250;
    } else {
        speed = 50;  // default safe speed
    }

    return hlscl_.RegWritePosEx(ID, static_cast<s16>(target_pos), speed, 50, 500) != -1;
}

bool CFeetechProtocol::actionStart(uint8_t ID) {
    if (!isConnected_) return false;
    return hlscl_.RegWriteAction(ID) != -1;
}

bool CFeetechProtocol::moveStop(uint8_t ID) {
    if (!isConnected_) return false;
    return hlscl_.WriteSpe(ID, 0, 50, 500) != -1;
}

bool CFeetechProtocol::getPosition(uint8_t ID, int16_t& pos) {
    if (!isConnected_) return false;
    int res = hlscl_.ReadPos(ID);
    if (res != -1) {
        pos = static_cast<int16_t>(convertFeetechToTicks(static_cast<uint16_t>(res)));
        return true;
    }
    return false;
}

bool CFeetechProtocol::getPositionLimits(uint8_t ID, uint16_t& min_position, uint16_t& max_position) {
    if (!isConnected_) return false;
    int min_pos = hlscl_.readWord(ID, HLSCL_MIN_ANGLE_LIMIT_L);
    int max_pos = hlscl_.readWord(ID, HLSCL_MAX_ANGLE_LIMIT_L);
    if (min_pos != -1 && max_pos != -1) {
        min_position = convertFeetechToTicks(static_cast<uint16_t>(min_pos));
        max_position = convertFeetechToTicks(static_cast<uint16_t>(max_pos));
        return true;
    }
    return false;
}

bool CFeetechProtocol::setPositionLimits(uint8_t ID, uint16_t min_position, uint16_t max_position) {
    if (!isConnected_) return false;
    uint16_t min_counts = convertTicksToFeetech(min_position);
    uint16_t max_counts = convertTicksToFeetech(max_position);
    hlscl_.unLockEprom(ID);
    int r1 = hlscl_.writeWord(ID, HLSCL_MIN_ANGLE_LIMIT_L, min_counts);
    int r2 = hlscl_.writeWord(ID, HLSCL_MAX_ANGLE_LIMIT_L, max_counts);
    hlscl_.LockEprom(ID);
    return r1 != -1 && r2 != -1;
}

bool CFeetechProtocol::getPositionOffset(uint8_t ID, int8_t& deviation) {
    if (!isConnected_) return false;
    int ofs = hlscl_.readWord(ID, HLSCL_OFS_L);
    if (ofs != -1) {
        if (ofs & (1 << 15)) {
            ofs = -(ofs & ~(1 << 15));
        }
        double dev_deg = static_cast<double>(ofs) * (360.0 / 4096.0);
        deviation = static_cast<int8_t>(std::round(dev_deg / 0.24));
        return true;
    }
    return false;
}

bool CFeetechProtocol::setPositionOffset(uint8_t ID, int8_t deviation) {
    if (!isConnected_) return false;
    double dev_deg = static_cast<double>(deviation) * 0.24;
    int16_t ofs_counts = static_cast<int16_t>(std::round(dev_deg * (4096.0 / 360.0)));
    uint16_t ofs_u16 = static_cast<uint16_t>(ofs_counts);
    if (ofs_counts < 0) {
        ofs_u16 = static_cast<uint16_t>(-ofs_counts) | (1 << 15);
    }
    hlscl_.unLockEprom(ID);
    int res = hlscl_.writeWord(ID, HLSCL_OFS_L, ofs_u16);
    hlscl_.LockEprom(ID);
    return res != -1;
}

bool CFeetechProtocol::savePositionOffset(uint8_t /*ID*/) {
    return true;
}

bool CFeetechProtocol::getMotorSpeed(uint8_t ID, int16_t& speed) {
    if (!isConnected_) return false;
    int res = hlscl_.ReadSpeed(ID);
    if (res != -1) {
        speed = static_cast<int16_t>(res);
        return true;
    }
    return false;
}

bool CFeetechProtocol::getVoltage(uint8_t ID, uint16_t& mV) {
    if (!isConnected_) return false;
    int res = hlscl_.ReadVoltage(ID);
    if (res != -1) {
        mV = static_cast<uint16_t>(res * 100);
        return true;
    }
    return false;
}

bool CFeetechProtocol::getVoltageLimits(uint8_t /*ID*/, uint16_t& /*mVmin*/, uint16_t& /*mVmax*/) {
    return false;
}

bool CFeetechProtocol::setVoltageLimits(uint8_t /*ID*/, uint16_t /*mVmin*/, uint16_t /*mVmax*/) {
    return false;
}

bool CFeetechProtocol::getTemperature(uint8_t ID, uint8_t& degTemp) {
    if (!isConnected_) return false;
    int res = hlscl_.ReadTemper(ID);
    if (res != -1) {
        degTemp = static_cast<uint8_t>(res);
        return true;
    }
    return false;
}

bool CFeetechProtocol::getMaxTemperatureLimit(uint8_t /*ID*/, uint8_t& /*degLimit*/) {
    return false;
}

bool CFeetechProtocol::setMaxTemperatureLimit(uint8_t /*ID*/, uint8_t /*degLimit*/) {
    return false;
}

bool CFeetechProtocol::isLedOn(uint8_t /*ID*/) {
    return false;
}

bool CFeetechProtocol::setLed(uint8_t /*ID*/, bool /*on*/) {
    return false;
}

bool CFeetechProtocol::getLedErrcode(uint8_t ID, uint8_t& lederrcode) {
    if (!isConnected_) return false;
    int res = hlscl_.Ping(ID);
    if (res != -1) {
        lederrcode = hlscl_.getState();
        return true;
    }
    return false;
}

bool CFeetechProtocol::flashLedErrCode(uint8_t /*ID*/, uint8_t /*code*/) {
    return false;
}

uint16_t CFeetechProtocol::convertTicksToFeetech(uint16_t ticks) const {
    double angle_deg = (static_cast<double>(ticks) - 500.0) * 0.24;
    double feetech_ticks = angle_deg * (4096.0 / 360.0) + 2048.0;
    if (feetech_ticks < 0.0) feetech_ticks = 0.0;
    if (feetech_ticks > 4095.0) feetech_ticks = 4095.0;
    return static_cast<uint16_t>(std::round(feetech_ticks));
}

uint16_t CFeetechProtocol::convertFeetechToTicks(uint16_t feetech_ticks) const {
    double angle_deg = (static_cast<double>(feetech_ticks) - 2048.0) * (360.0 / 4096.0);
    double ticks = angle_deg * (25.0 / 6.0) + 500.0;
    if (ticks < 0.0) ticks = 0.0;
    if (ticks > 1000.0) ticks = 1000.0;
    return static_cast<uint16_t>(std::round(ticks));
}

}  // namespace rumblex_movement
