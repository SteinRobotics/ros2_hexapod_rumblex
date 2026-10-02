/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "requester/requester.hpp"

#include <cmath>

#include "units.hpp"

using namespace rumblex_interfaces::msg;
using std::placeholders::_1;

namespace rumblex_movement {

CRequester::CRequester(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CServoHandler> servo_handler)
    : node_(node) {
    kinematics_ = std::make_shared<CKinematics>(node);
    if (servo_handler) {
        servo_handler_ = servo_handler;
    } else {
        servo_handler_ = std::make_shared<CServoHandler>(node);
    }

    if (auto servo_controller = servo_handler_->getServoController()) {
        servo_controller->setInitialAnglesCallback(
            [this](const std::map<ELegIndex, CLegAngles>& initial_angles, const COrientation& head) {
                RCLCPP_INFO_STREAM(node_->get_logger(),
                                   "on Callback: initial servo angles received, setting kinematics.");
                kinematics_->setHeadOrientation(head);
                for (const auto& [leg_index, leg_angles] : initial_angles) {
                    kinematics_->setLegAngles(leg_index, leg_angles);
                }
            });
    }

    sub_body_pose_ = node_->create_subscription<BodyPose>("cmd_movement", rclcpp::QoS(1),
                                                          std::bind(&CRequester::onBodyPose, this, _1));
    pub_body_pose_ = node_->create_publisher<BodyPose>("body_pose_actual", rclcpp::QoS(1).transient_local());
    pub_body_pose_->publish(bodyPose(*kinematics_));

    pub_joint_states_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
}

void CRequester::sendServoRequest(const double duration_s) {
    auto head = kinematics_->getHeadOrientation();
    auto legs = kinematics_->getLegAngles();
    servo_handler_->run(CRequest(head, legs, duration_s));
}

void CRequester::publishJointStates(const std::map<ELegIndex, CLegAngles>& legs, const COrientation& head) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node_->get_clock()->now();

    // Joint names must match the URDF joint names in rumblex_description
    static const std::vector<std::pair<ELegIndex, std::string>> leg_order = {
        {ELegIndex::RightFront, "right_front"}, {ELegIndex::RightMid, "right_mid"},
        {ELegIndex::RightBack, "right_back"},   {ELegIndex::LeftFront, "left_front"},
        {ELegIndex::LeftMid, "left_mid"},       {ELegIndex::LeftBack, "left_back"},
    };

    for (const auto& [leg_idx, prefix] : leg_order) {
        auto it = legs.find(leg_idx);
        if (it == legs.end()) continue;
        const auto& a = it->second;
        msg.name.push_back(prefix + "_coxa_joint");
        msg.position.push_back(a.torso_coxa.numerical_value_in(units::rad));
        msg.name.push_back(prefix + "_femur_joint");
        msg.position.push_back(a.coxa_femur.numerical_value_in(units::rad));
        msg.name.push_back(prefix + "_tibia_joint");
        msg.position.push_back(a.femur_tibia.numerical_value_in(units::rad));
    }

    msg.name.push_back("head_yaw_joint");
    msg.position.push_back(head.yaw.numerical_value_in(units::rad));
    msg.name.push_back("head_pitch_joint");
    msg.position.push_back(head.pitch.numerical_value_in(units::rad));

    pub_joint_states_->publish(msg);
}

void CRequester::onBodyPose(const BodyPose& msg) {
    if (!validBodyPose(msg)) {
        RCLCPP_WARN(node_->get_logger(), "Ignoring non-finite body pose");
        return;
    }
    kinematics_->moveTorso(toeTargets(msg), CPose(msg.torso_pose));
    kinematics_->setHeadOrientation(COrientation(msg.head_pose));
    pending_pose_ = true;
}

void CRequester::update(std::chrono::milliseconds timeslice) {
    if (pending_pose_) {
        pending_pose_ = false;
        double duration_s = double(timeslice.count() / 1000.0);
        sendServoRequest(duration_s);
    }
    pub_body_pose_->publish(bodyPose(*kinematics_));
    publishJointStates(kinematics_->getLegAngles(), kinematics_->getHeadOrientation());
}

}  // namespace rumblex_movement
