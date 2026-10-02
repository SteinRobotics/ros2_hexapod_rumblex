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
#include "rumblex_utils/body_pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
//

#include "handler/servo_handler.hpp"
#include "kinematics.hpp"

namespace rumblex_movement {

class CRequester {
   public:
    CRequester(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CServoHandler> servo_handler = nullptr);
    virtual ~CRequester() = default;

    void update(std::chrono::milliseconds timeslice);

    void onBodyPose(const rumblex_interfaces::msg::BodyPose& msg);

   private:
    rclcpp::Subscription<rumblex_interfaces::msg::BodyPose>::SharedPtr sub_body_pose_;
    rclcpp::Publisher<rumblex_interfaces::msg::BodyPose>::SharedPtr pub_body_pose_;
    bool pending_pose_ = false;

    void sendServoRequest(const double duration_s);

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    std::shared_ptr<CServoHandler> servo_handler_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
    void publishJointStates(const std::map<ELegIndex, CLegAngles>& legs, const COrientation& head);
};
}  // namespace rumblex_movement
