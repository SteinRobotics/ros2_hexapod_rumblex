/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
//
#include "rumblex_interfaces/msg/continuous_movement_update.hpp"
#include "rumblex_interfaces/msg/movement_request.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
//
#include "gaitcontroller.hpp"
#include "handler/servohandler.hpp"
#include "igaits.hpp"
#include "kinematics.hpp"

namespace rumblex_movement {

class CRequester {
   public:
    CRequester(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CServoHandler> servoHandler = nullptr);
    virtual ~CRequester() = default;

    void update(std::chrono::milliseconds timeslice);

    void onMovementRequest(const rumblex_interfaces::msg::MovementRequest& msg);
    void onContinuousMovementUpdate(const rumblex_interfaces::msg::ContinuousMovementUpdate& msg);

   private:
    rclcpp::Subscription<rumblex_interfaces::msg::MovementRequest>::SharedPtr subMovementRequest_;
    rclcpp::Subscription<rumblex_interfaces::msg::ContinuousMovementUpdate>::SharedPtr
        subContinuousMovementUpdate_;

    void sendServoRequest(const double duration_s);

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    std::shared_ptr<CGaitController> gait_controller_;
    std::shared_ptr<CServoHandler> servo_handler_;

    geometry_msgs::msg::Twist velocity_;
    rumblex_interfaces::msg::Pose pose_body_;
    rumblex_interfaces::msg::Orientation orientation_head_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pubJointStates_;
    void publishJointStates(const std::map<ELegIndex, CLegAngles>& legs, const COrientation& head);
};
}  // namespace rumblex_movement
