/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "handler/servohandler.hpp"

#include "handler/leg_servo_conversion.hpp"

using namespace std::chrono_literals;
using namespace rumblex_interfaces::msg;

namespace rumblex_movement {

CServoHandler::CServoHandler(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    servoController_ = std::make_shared<CServoController>(node_);
}

void CServoHandler::run(CRequest request) {
    // RCLCPP_INFO_STREAM(node_->get_logger(), "CServoHandler::run | CRequest");
    auto targetAngles = leg_servo_conversion::buildServoTargets(request.head(), request.legAngles());
    servoController_->requestAngles(targetAngles, request.duration());
}

}  // namespace rumblex_movement
