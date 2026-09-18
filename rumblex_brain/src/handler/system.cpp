/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#include "handler/system.hpp"

namespace brain {

CSystem::CSystem(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    pubServoRelay_ = node_->create_publisher<std_msgs::msg::Bool>("request_servo_relay", 10);
    pubSystemShutdown_ = node_->create_publisher<std_msgs::msg::Bool>("request_system_shutdown", 10);
}

void CSystem::run(std::shared_ptr<RequestSystem> request) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "CSystem::run(RequestSystem request)");
    if (request->systemShutdown) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "systemShutdown");
    }
    if (request->turnOffServoRelay) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "turnOffServoRelay");
    }
    // System shutdown
    std_msgs::msg::Bool msgSystemShutdown;
    msgSystemShutdown.data = request->systemShutdown;
    pubSystemShutdown_->publish(msgSystemShutdown);
    setDone(false);

    // Servo relay (invert: turnOffServoRelay=true means relay should be OFF)
    std_msgs::msg::Bool msgServoRelay;
    msgServoRelay.data = !request->turnOffServoRelay;
    pubServoRelay_->publish(msgServoRelay);

    setDone(false);
    // shall we wait for expectedDurationMs and then set done() to true??
    // m_expectedDurationMs = system.expectedDurationMs;
}

void CSystem::update() {
    setDone(true);
}

void CSystem::cancel() {
}

}  // namespace brain
