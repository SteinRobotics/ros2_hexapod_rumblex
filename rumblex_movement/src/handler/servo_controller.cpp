/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

// Servo Layout
//
//            020
//            010
//             |
// 036-026-016-|-011-021-031
//             |
// 035-025-015-|-012-022-032
//             |
// 034-024-014-|-013-023-033
//

#include "handler/servo_controller.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "handler/feetech_protocol.hpp"
#include "handler/hiwonder_protocol.hpp"
#include "handler/leg_servo_conversion.hpp"
#include "handler/offline_servo_protocol.hpp"
using namespace std::chrono_literals;
using rumblex_interfaces::msg::ServoAngle;
using rumblex_interfaces::msg::ServoAngles;
using rumblex_interfaces::msg::ServoDirectRequest;
using rumblex_interfaces::msg::ServoStatus;
using std::placeholders::_1;

namespace rumblex_movement {

namespace {

constexpr auto kServoControllerTypeParam = "SERVO_CONTROLLER_TYPE";
constexpr auto kHiwonderServoControllerType = "hiwonder";
constexpr auto kFeetechServoControllerType = "feetech";

std::string normalizeServoControllerType(std::string controllerType) {
    std::transform(controllerType.begin(), controllerType.end(), controllerType.begin(),
                   [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    return controllerType;
}

std::shared_ptr<CServoProtocol> createServoProtocol(const std::shared_ptr<rclcpp::Node>& node,
                                                    const std::string& serialPort,
                                                    const bool isServoControllerOffline,
                                                    const std::string& servoControllerType) {
    if (isServoControllerOffline) {
        RCLCPP_INFO_STREAM(node->get_logger(),
                           "CServoController: using offline protocol for servo controller type '"
                               << servoControllerType << "'");
        return std::make_shared<COfflineServoProtocol>(node, serialPort);
    }

    if (servoControllerType == kHiwonderServoControllerType) {
        return std::make_shared<CHiwonderProtocol>(node, serialPort);
    }

    if (servoControllerType == kFeetechServoControllerType) {
        return std::make_shared<CFeetechProtocol>(node, serialPort);
    }

    RCLCPP_ERROR_STREAM(node->get_logger(), "CServoController: unsupported SERVO_CONTROLLER_TYPE='"
                                                << servoControllerType << "'. Supported values are '"
                                                << kHiwonderServoControllerType << "' and '"
                                                << kFeetechServoControllerType << "'");
    return nullptr;
}

}  // namespace

CServoController::CServoController(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    cycleCounter_ = 0;

    RCLCPP_INFO_STREAM(node_->get_logger(), "CServoController: initializing connection...");

    std::string serialPort = node_->declare_parameter<std::string>("SERIAL_PORT", "/dev/ttyUSB0");

    std::string servoControllerType = normalizeServoControllerType(
        node_->declare_parameter<std::string>(kServoControllerTypeParam, kHiwonderServoControllerType));

    bool isServoControllerOffline = node_->declare_parameter<bool>("SERVO_CONTROLLER_OFFLINE", false);

    std::vector<std::string> names =
        node_->declare_parameter<std::vector<std::string>>("SERVO_NAME", std::vector<std::string>());

    std::vector<double> adaptations =
        node_->declare_parameter<std::vector<double>>("SERVO_ADAPTATION_DEG", std::vector<double>());

    std::vector<double> offsets =
        node_->declare_parameter<std::vector<double>>("SERVO_OFFSET_DEG", std::vector<double>());

    std::vector<bool> clockwise =
        node_->declare_parameter<std::vector<bool>>("SERVO_ORIENTATION_CLOCKWISE", std::vector<bool>());

    std::vector<int64_t> ids =
        node_->declare_parameter<std::vector<int64_t>>("SERVO_SERIAL_ID", std::vector<int64_t>());

    size_t numServos = names.size();
    if (adaptations.size() < numServos || offsets.size() < numServos || clockwise.size() < numServos ||
        ids.size() < numServos) {
        RCLCPP_ERROR(node_->get_logger(),
                     "CServoController: Parameter size mismatch! SERVO_NAME: %zu, "
                     "SERVO_ADAPTATION_DEG: %zu, SERVO_OFFSET_DEG: %zu, "
                     "SERVO_ORIENTATION_CLOCKWISE: %zu, SERVO_SERIAL_ID: %zu",
                     names.size(), adaptations.size(), offsets.size(), clockwise.size(), ids.size());
        numServos =
            std::min({names.size(), adaptations.size(), offsets.size(), clockwise.size(), ids.size()});
    }

    for (size_t i = 0; i < numServos; ++i) {
        servos_[i] = CServo{names[i], static_cast<uint8_t>(ids[i]), clockwise[i], offsets[i], adaptations[i]};
        nameToIdx_[names.at(i)] = i;
    }

    protocol_ = createServoProtocol(node_, serialPort, isServoControllerOffline, servoControllerType);
    if (!protocol_) {
        return;
    }

    if (!protocol_->triggerConnection()) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "CServoController: connection FAILED");
        return;
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "CServoController: connection successful");

    // wait 500ms for servos to be ready
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    subSingleServoRequest_ = node_->create_subscription<ServoAngle>(
        "single_servo_request", 10, std::bind(&CServoController::onSingleServoRequestReceived, this, _1));

    subServoDirectRequest_ = node_->create_subscription<ServoDirectRequest>(
        "servo_direct_request", 10, std::bind(&CServoController::onServoDirectRequestReceived, this, _1));

    pubAngles_ = node_->create_publisher<ServoAngles>("servo_angles", 10);
    pubStatus_ = node_->create_publisher<ServoStatus>("servo_status", 10);
}

void CServoController::initServos() {
    RCLCPP_INFO_STREAM(node_->get_logger(), "CServoController: initializing servos... " << servos_.size());

    for (auto& [idx, servo] : servos_) {
        auto start = node_->get_clock()->now();

        RCLCPP_INFO_STREAM(node_->get_logger(), "CServoController: initializing servo "
                                                    << servo.getName() << " (ID "
                                                    << uint32_t(servo.getSerialID()) << ")");

        uint8_t ledErrorCode = 0;
        if (!protocol_->getLedErrcode(servo.getSerialID(), ledErrorCode)) {
            RCLCPP_ERROR_STREAM(node_->get_logger(),
                                "CServoController: TIMEOUT getLedErrcode " << servo.getName());
        } else {
            servo.setErrorCode(ledErrorCode);
        }

        uint8_t temp = 0;
        if (!protocol_->getTemperature(servo.getSerialID(), temp)) {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "CServoController: TIMEOUT getTemp " << servo.getName());
        } else {
            servo.setTemperature(temp);
        }

        uint16_t vin = 0;
        if (!protocol_->getVoltage(servo.getSerialID(), vin)) {
            RCLCPP_ERROR_STREAM(node_->get_logger(),
                                "CServoController: TIMEOUT getVoltage " << servo.getName());
        } else {
            double vin_volt = static_cast<double>(vin) / 1000.0;  // convert mV to V
            servo.setVoltage(vin_volt);
        }

        int16_t pos = 0;
        if (!protocol_->getPosition(servo.getSerialID(), pos)) {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "CServoController: TIMEOUT getPos " << servo.getName());
        } else {
            servo.setAngle(ticks_to_angle(pos, idx));
        }

        auto duration = node_->get_clock()->now() - start;
        RCLCPP_INFO(node_->get_logger(), "CServoController: %s initialization took: %.3fs",
                    servo.getName().c_str(), duration.seconds());
        RCLCPP_INFO_STREAM(node_->get_logger(), servo.getVoltage()
                                                    << "V" << " | " << servo.getTemperature() << "°C"
                                                    << " | angle: " << servo.getAngle()
                                                    << "° | error code: " << uint32_t(servo.getErrorCode()));
    }

    auto msg_angles = ServoAngles();
    for (auto& [idx, servo] : servos_) {
        msg_angles.current_angles[idx].angle_deg = servo.getAngle();
        msg_angles.current_angles[idx].name = servo.getName();
    }
    msg_angles.header.stamp = node_->get_clock()->now();

    if (initialAnglesCallback_) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "initial servo angles received, invoking callback.");
        auto initial_leg_angles_deg = leg_servo_conversion::servoAnglesMsgToLegAngles(msg_angles);
        initialAnglesCallback_(initial_leg_angles_deg);
    }

    if (pubAngles_) {
        pubAngles_->publish(msg_angles);
    }

    timer_ = node_->create_wall_timer(100ms, std::bind(&CServoController::onTimerStatus, this));
}

double CServoController::ticks_to_angle(int ticks, int idx) {
    double angle = (static_cast<double>(ticks) - 500.0) * 6.0 / 25.0;
    if (!servos_.at(idx).isOrientationClockwise()) {
        angle = -angle;
    }
    angle = angle - servos_.at(idx).getAdaptation() + servos_.at(idx).getOffsetDegree();
    return angle;
}

int CServoController::angle_to_ticks(double angle, int idx) {
    angle = angle - servos_.at(idx).getOffsetDegree() + servos_.at(idx).getAdaptation();
    if (!servos_.at(idx).isOrientationClockwise()) {
        angle = -angle;
    }
    return static_cast<int>(angle * 25.0 / 6.0 + 500.0);
}

void CServoController::onTimerStatus() {
    if (servos_.empty()) return;
    // RCLCPP_INFO_STREAM(node_->get_logger(), "onTimerStatus: " << uint32_t(cycleCounter_));
    CServo& servo = servos_.at(cycleCounter_);

    uint8_t error_code = 0;
    if (!protocol_->getLedErrcode(servo.getSerialID(), error_code)) {
        RCLCPP_ERROR_STREAM_ONCE(node_->get_logger(), "TIMEOUT getLedErrcode " << servo.getName());
    }
    servo.setErrorCode(error_code);

    uint8_t temp = 0;
    if (!protocol_->getTemperature(servo.getSerialID(), temp)) {
        RCLCPP_ERROR_STREAM_ONCE(node_->get_logger(), "TIMEOUT getTemp " << servo.getName());
    }
    servo.setTemperature(temp);

    uint16_t vin = 0;
    if (!protocol_->getVoltage(servo.getSerialID(), vin)) {
        RCLCPP_ERROR_STREAM_ONCE(node_->get_logger(), "TIMEOUT getVoltage " << servo.getName());
    }
    double vin_volt = static_cast<double>(vin) / 1000.0;  // convert mV to V
    servo.setVoltage(vin_volt);

    auto msg_status = ServoStatus();
    msg_status.header.stamp = node_->get_clock()->now();

    double max_temp = 0, max_voltage = 0, min_voltage = 99;
    std::string name_temp, name_max_volt, name_min_volt;

    for (auto& [idx, servo] : servos_) {
        if (servo.getTemperature() > max_temp) {
            max_temp = servo.getTemperature();
            name_temp = servo.getName();
        }
        if (servo.getVoltage() > max_voltage) {
            max_voltage = servo.getVoltage();
            name_max_volt = servo.getName();
        }
        if (servo.getVoltage() < min_voltage) {
            min_voltage = servo.getVoltage();
            name_min_volt = servo.getName();
        }
    }

    msg_status.max_temperature = max_temp;
    msg_status.servo_max_temperature = name_temp;
    msg_status.max_voltage = max_voltage;
    msg_status.servo_max_voltage = name_max_volt;
    msg_status.min_voltage = min_voltage;
    msg_status.servo_min_voltage = name_min_volt;

    pubStatus_->publish(msg_status);

    cycleCounter_ = (cycleCounter_ + 1) % servos_.size();
}

void CServoController::requestAngles(const std::map<uint32_t, double>& targetAngles,
                                     const double duration_s) {
    for (const auto& [idx, targetAngle] : targetAngles) {
        auto it = servos_.find(idx);
        if (it == servos_.end()) {
            RCLCPP_WARN_ONCE(node_->get_logger(), "requestAngles: unknown servo index %u", idx);
            continue;
        }
        double diff = std::abs(it->second.getAngle() - targetAngle);
        if (diff < 0.49) continue;

        // max speed is 3ms for 1° (0.18s for 60°)
        int duration = std::max(static_cast<int>(diff * 3), static_cast<int>(duration_s * 1000.0));
        it->second.setAngle(targetAngle);

        int ticks = angle_to_ticks(targetAngle, idx);
        protocol_->setRegPos(it->second.getSerialID(), ticks, duration);
    }
    protocol_->actionStart();
}

// void CServoController::sendServoRequest(const ServoRequest& msg) {
//     for (size_t idx = 0; idx < msg.target_angles.size(); ++idx) {
//         // guard: skip indexes we don't know about
//         if (servos_.find(static_cast<int>(idx)) == servos_.end()) {
//             RCLCPP_WARN(node_->get_logger(), "sendServoRequest: unknown servo index %zu", idx);
//             continue;
//         }
//         double req_angle = msg.target_angles[idx].angle_deg;
//         double diff = std::abs(servos_.at(idx).getAngle() - req_angle);
//         if (diff < 0.49) continue;

//         int duration =
//             std::max(static_cast<int>(diff * 3), static_cast<int>(msg.time_to_reach_target_angles_ms));
//         servos_.at(idx).setAngle(req_angle);

//         int ticks = angle_to_ticks(req_angle, idx);
//         protocol_->setRegPos(servos_.at(idx).getSerialID(), ticks, duration);
//     }
//     protocol_->actionStart();
// }

void CServoController::onSingleServoRequestReceived(const ServoAngle& msg) {
    RCLCPP_INFO(node_->get_logger(), "single_servo_request: %s: %.1f", msg.name.c_str(), msg.angle_deg);
    auto it = nameToIdx_.find(msg.name);
    if (it == nameToIdx_.end()) {
        RCLCPP_WARN(node_->get_logger(), "single_servo_request: unknown servo name '%s'", msg.name.c_str());
        return;
    }
    int idx = it->second;
    double req_angle = msg.angle_deg;
    double diff = std::abs(servos_.at(idx).getAngle() - req_angle);

    int duration = std::max(static_cast<int>(diff * 3), 2000);
    servos_.at(idx).setAngle(req_angle);

    int ticks = angle_to_ticks(req_angle, idx);
    protocol_->setPosition(servos_.at(idx).getSerialID(), ticks, duration);
}

void CServoController::onServoDirectRequestReceived(const ServoDirectRequest& msg) {
    RCLCPP_INFO(node_->get_logger(), "servo_direct_request: %s: %d", msg.name.c_str(), msg.cmd);

    auto it = nameToIdx_.find(msg.name);
    if (it == nameToIdx_.end()) {
        RCLCPP_WARN(node_->get_logger(), "servo_direct_request: unknown servo name '%s'", msg.name.c_str());
        return;
    }
    int idx = it->second;
    if (servos_.find(idx) == servos_.end()) {
        RCLCPP_WARN(node_->get_logger(), "servo_direct_request: servo index %d not present", idx);
        return;
    }
    uint8_t serialID = servos_.at(idx).getSerialID();

    switch (msg.cmd) {
        case ServoDirectRequest::SERVO_ID_READ: {
            uint8_t id = 0;
            protocol_->getServoID(serialID, id);
            RCLCPP_INFO(node_->get_logger(), "read servo ID: %d", id);
            break;
        }
        case ServoDirectRequest::SERVO_ID_WRITE: {
            RCLCPP_INFO(node_->get_logger(), "set servo ID: %d -> %d", serialID, msg.data1);
            protocol_->setServoID(serialID, msg.data1);
            servos_.at(idx).setSerialID(msg.data1);
            break;
        }
        case ServoDirectRequest::SERVO_LED_CTRL_READ: {
            bool ledOn = protocol_->isLedOn(serialID);
            RCLCPP_INFO(node_->get_logger(), "read LED: %s", ledOn ? "ON" : "OFF");
            break;
        }
        case ServoDirectRequest::SERVO_LED_CTRL_WRITE: {
            RCLCPP_INFO(node_->get_logger(), "set LED: %s", msg.data1 ? "ON" : "OFF");
            protocol_->setLed(serialID, msg.data1);
            break;
        }

        default:
            break;
    }
}

void CServoController::setInitialAnglesCallback(InitialAnglesCallback callback) {
    RCLCPP_INFO(node_->get_logger(), "setInitialAnglesCallback");
    initialAnglesCallback_ = std::move(callback);
    initServos();
}

}  // namespace rumblex_movement
