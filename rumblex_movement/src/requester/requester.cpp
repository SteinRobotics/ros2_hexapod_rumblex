/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "requester/requester.hpp"

#include <cmath>

#include "rumblex_utils/geometry.hpp"

using namespace rumblex_interfaces::msg;
using std::placeholders::_1;

namespace rumblex_movement {

CRequester::CRequester(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CServoHandler> servoHandler)
    : node_(node) {
    kinematics_ = std::make_shared<CKinematics>(node);
    gait_controller_ = std::make_shared<CGaitController>(node, kinematics_);
    if (servoHandler) {
        servo_handler_ = servoHandler;
    } else {
        servo_handler_ = std::make_shared<CServoHandler>(node);
    }

    if (auto servo_controller = servo_handler_->getServoController()) {
        servo_controller->setInitialAnglesCallback(
            [this](const std::map<ELegIndex, CLegAngles>& initial_angles) {
                RCLCPP_INFO_STREAM(node_->get_logger(),
                                   "on Callback: initial servo angles received, setting kinematics.");
                for (const auto& [leg_index, leg_angles] : initial_angles) {
                    kinematics_->setLegAngles(leg_index, leg_angles);
                }
            });
    }

    subMovementRequest_ = node_->create_subscription<MovementRequest>(
        "cmd_movement", 10, std::bind(&CRequester::onMovementRequest, this, _1));

    subContinuousMovementUpdate_ = node_->create_subscription<ContinuousMovementUpdate>(
        "cmd_movement_update", 10, std::bind(&CRequester::onContinuousMovementUpdate, this, _1));

    pubJointStates_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
}

void CRequester::sendServoRequest(const double duration_s) {
    auto head = kinematics_->getHead();
    auto legs = kinematics_->getLegsAngles();
    servo_handler_->run(CRequest(head, legs, duration_s));
    publishJointStates(legs, head);
}

void CRequester::publishJointStates(const std::map<ELegIndex, CLegAngles>& legs, const COrientation& head) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node_->get_clock()->now();

    // Joint names must match the URDF joint names in rumblex_description
    static const std::vector<std::pair<ELegIndex, std::string>> legOrder = {
        {ELegIndex::RightFront, "right_front"}, {ELegIndex::RightMid, "right_mid"},
        {ELegIndex::RightBack, "right_back"},   {ELegIndex::LeftFront, "left_front"},
        {ELegIndex::LeftMid, "left_mid"},       {ELegIndex::LeftBack, "left_back"},
    };

    for (const auto& [legIdx, prefix] : legOrder) {
        auto it = legs.find(legIdx);
        if (it == legs.end()) continue;
        const auto& a = it->second;
        msg.name.push_back(prefix + "_coxa_joint");
        msg.position.push_back(utils::deg2rad(a.coxa_deg));
        msg.name.push_back(prefix + "_femur_joint");
        msg.position.push_back(utils::deg2rad(a.femur_deg));
        msg.name.push_back(prefix + "_tibia_joint");
        msg.position.push_back(utils::deg2rad(a.tibia_deg));
    }

    msg.name.push_back("head_yaw_joint");
    msg.position.push_back(utils::deg2rad(head.yaw_deg));
    msg.name.push_back("head_pitch_joint");
    msg.position.push_back(utils::deg2rad(head.pitch_deg));

    pubJointStates_->publish(msg);
}

void CRequester::onMovementRequest(const MovementRequest& msg) {
    if (msg.type != gait_controller_->currentGait()) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "CRequester::onMovementRequest: " << msg.name);
    }
    gait_controller_->setGait(msg);
}

void CRequester::onContinuousMovementUpdate(const ContinuousMovementUpdate& msg) {
    velocity_ = msg.velocity;
    pose_body_ = msg.body_pose;
    orientation_head_ = msg.head_orientation;
}

void CRequester::update(std::chrono::milliseconds timeslice) {
    if (gait_controller_->updateSelectedGait(velocity_, pose_body_, orientation_head_)) {
        double duration_s = double(timeslice.count() / 1000.0);
        sendServoRequest(duration_s);
    }
}

}  // namespace rumblex_movement
