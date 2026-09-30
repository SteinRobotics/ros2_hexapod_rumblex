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
#include "gait_controller.hpp"
#include "gait_interfaces.hpp"
#include "handler/servo_handler.hpp"
#include "kinematics.hpp"

namespace rumblex_movement {

class CRequester {
   public:
    CRequester(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CServoHandler> servo_handler = nullptr);
    virtual ~CRequester() = default;

    void update(std::chrono::milliseconds timeslice);

    void onMovementRequest(const rumblex_interfaces::msg::MovementRequest& msg);
    void onContinuousMovementUpdate(const rumblex_interfaces::msg::ContinuousMovementUpdate& msg);

   private:
    rclcpp::Subscription<rumblex_interfaces::msg::MovementRequest>::SharedPtr sub_movement_request_;
    rclcpp::Subscription<rumblex_interfaces::msg::ContinuousMovementUpdate>::SharedPtr
        sub_continuous_movement_update_;

    void sendServoRequest(const double duration_s);

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    std::shared_ptr<CGaitController> gait_controller_;
    std::shared_ptr<CServoHandler> servo_handler_;

    geometry_msgs::msg::Twist velocity_;
    rumblex_interfaces::msg::Pose torso_pose_;
    rumblex_interfaces::msg::Orientation head_orientation_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
    void publishJointStates(const std::map<ELegIndex, CLegAngles>& legs, const COrientation& head);
};
}  // namespace rumblex_movement
