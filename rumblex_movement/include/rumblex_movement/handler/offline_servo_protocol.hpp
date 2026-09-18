/*
 * Simple test mock for BusServoProtocol used in unit tests.
 * Provides basic, deterministic behavior for servo queries and commands.
 */
#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

#include "handler/servo_protocol.hpp"

namespace rumblex_movement {

class COfflineServoProtocol : public CServoProtocol {
   public:
    COfflineServoProtocol(std::shared_ptr<rclcpp::Node> node, const std::string& /*port*/)
        : CServoProtocol(node) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "COfflineServoProtocol initialized (mock)");
        // initialize some sensible defaults for common servo ids
        for (uint8_t id = 1; id <= 40; ++id) {
            positions_[id] = 500;    // neutral tick
            temperatures_[id] = 30;  // 30°C
            voltages_[id] = 12100;   // 12.1 V in mV
            ledErrcode_[id] = 0;
            ledOn_[id] = false;
            idMap_[id] = id;
        }
    }

    // Simulate successful connection
    bool triggerConnection() override {
        return true;
    }

    bool getLedErrcode(uint8_t id, uint8_t& out) override {
        out = ledErrcode_.count(id) ? ledErrcode_[id] : 0;
        return true;
    }

    bool getTemperature(uint8_t id, uint8_t& out) override {
        out = temperatures_.count(id) ? temperatures_[id] : 30;
        return true;
    }

    bool getVoltage(uint8_t id, uint16_t& out) override {
        out = voltages_.count(id) ? static_cast<uint16_t>(voltages_[id]) : static_cast<uint16_t>(12000);
        return true;
    }

    bool getPosition(uint8_t id, int16_t& out) override {
        out = positions_.count(id) ? static_cast<int16_t>(positions_[id]) : static_cast<int16_t>(500);
        return true;
    }

    // Action commands - record requested values
    bool actionStart(uint8_t /*ID*/ = SERVO_Broadcast_ID) override {
        return true;
    }

    bool setRegPos(uint8_t id, uint16_t pos, uint16_t /*duration_ms*/) override {
        positions_[id] = pos;
        return true;
    }

    bool setPosition(uint8_t id, uint16_t pos, uint16_t /*duration_ms*/) override {
        positions_[id] = pos;
        return true;
    }

    bool getServoID(uint8_t serial, uint8_t& out) override {
        out = idMap_.count(serial) ? idMap_[serial] : serial;
        return true;
    }

    bool setServoID(uint8_t serial, uint8_t newId) override {
        idMap_[serial] = newId;
        return true;
    }

    // Test helper: read mapped id
    uint8_t getMappedId(uint8_t serial) const {
        return idMap_.count(serial) ? idMap_.at(serial) : serial;
    }

    bool isLedOn(uint8_t id) override {
        return ledOn_.count(id) ? ledOn_[id] : false;
    }

    bool setLed(uint8_t id, bool on) override {
        ledOn_[id] = on;
        return true;
    }

   private:
    std::unordered_map<uint8_t, int> positions_;
    std::unordered_map<uint8_t, uint8_t> temperatures_;
    std::unordered_map<uint8_t, int> voltages_;
    std::unordered_map<uint8_t, uint8_t> ledErrcode_;
    std::unordered_map<uint8_t, bool> ledOn_;
    std::unordered_map<uint8_t, uint8_t> idMap_;
};

}  // namespace rumblex_movement
