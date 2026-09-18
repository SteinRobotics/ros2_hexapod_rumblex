/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#include "requester/recognition.hpp"

using std::placeholders::_1;

namespace brain {

CRecognition::CRecognition(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CCoordinator> coordinator)
    : node_(node), coordinator_(coordinator) {
    subSpeechRecognition_ = node_->create_subscription<std_msgs::msg::String>(
        "speech_recognition_online", 10, std::bind(&CRecognition::onSpeechRecognition, this, _1));

    subSupplyVoltage_ = node_->create_subscription<std_msgs::msg::Float32>(
        "supply_voltage", 10, std::bind(&CRecognition::onSupplyVoltage, this, _1));

    subJoystick_ = node_->create_subscription<rumblex_interfaces::msg::JoystickRequest>(
        "joystick_request", 10, std::bind(&CRecognition::onJoystickRequest, this, _1));

    subCmdVel_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&CRecognition::onCmdVel, this, _1));

    subServoStatus_ = node_->create_subscription<rumblex_interfaces::msg::ServoStatus>(
        "servo_status", 10, std::bind(&CRecognition::onServoStatus, this, _1));

    subMovementTypeActual_ = node_->create_subscription<rumblex_interfaces::msg::MovementRequest>(
        "movement_type_actual", 10, std::bind(&CRecognition::onMovementTypeActual, this, _1));
}

void CRecognition::onSupplyVoltage(const std_msgs::msg::Float32& msg) const {
    coordinator_->supplyVoltageReceived(msg.data);
}

void CRecognition::onSpeechRecognition(const std_msgs::msg::String& msg) const {
    coordinator_->speechRecognized(msg.data);
}

void CRecognition::onJoystickRequest(const rumblex_interfaces::msg::JoystickRequest& msg) const {
    coordinator_->joystickRequestReceived(msg);
}

void CRecognition::onCmdVel(const geometry_msgs::msg::Twist& msg) const {
    coordinator_->cmdVelReceived(msg);
}

void CRecognition::onServoStatus(const rumblex_interfaces::msg::ServoStatus& msg) const {
    coordinator_->servoStatusReceived(msg);
}

void CRecognition::onMovementTypeActual(const rumblex_interfaces::msg::MovementRequest& msg) const {
    coordinator_->movementTypeActualReceived(msg);
}

}  // namespace brain
