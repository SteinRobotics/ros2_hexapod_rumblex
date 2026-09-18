/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <functional>
#include <memory>
//
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
//
#include "geometry_msgs/msg/twist.hpp"
#include "rumblex_interfaces/msg/joystick_request.hpp"
#include "rumblex_interfaces/msg/movement_request.hpp"
#include "rumblex_interfaces/msg/servo_status.hpp"
#include "requester/coordinator.hpp"
#include "requester/error_management.hpp"
#include "requester/irequester.hpp"

namespace brain {

class CRecognition {
   public:
    CRecognition(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CCoordinator> coordinator);
    virtual ~CRecognition() = default;

    void onSupplyVoltage(const std_msgs::msg::Float32& msg) const;
    void onSpeechRecognition(const std_msgs::msg::String& msg) const;
    void onJoystickRequest(const rumblex_interfaces::msg::JoystickRequest& msg) const;
    void onCmdVel(const geometry_msgs::msg::Twist& msg) const;
    void onServoStatus(const rumblex_interfaces::msg::ServoStatus& msg) const;
    void onMovementTypeActual(const rumblex_interfaces::msg::MovementRequest& msg) const;

   private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subSpeechRecognition_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subSupplyVoltage_;
    rclcpp::Subscription<rumblex_interfaces::msg::JoystickRequest>::SharedPtr subJoystick_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subCmdVel_;
    rclcpp::Subscription<rumblex_interfaces::msg::ServoStatus>::SharedPtr subServoStatus_;
    rclcpp::Subscription<rumblex_interfaces::msg::MovementRequest>::SharedPtr subMovementTypeActual_;

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CCoordinator> coordinator_;
};

}  // namespace brain
