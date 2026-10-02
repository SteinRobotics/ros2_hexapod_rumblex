#pragma once

#include "rumblex_utils/body_pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace rumblex_movement {

// Display-only IK: the shared hardware model is deliberately left unchanged.
inline sensor_msgs::msg::JointState visualizationJoints(rumblex_geometry::CBodyModel& model,
                                                        const rumblex_interfaces::msg::BodyPose& pose) {
    using namespace rumblex_geometry;
    const CPose torso(pose.torso_pose);
    auto targets = toeTargets(pose);
    // R^-1 = Rx(-roll) Ry(-pitch) Rz(-yaw), applied right to left.
    for (auto& [index, target] : targets) {
        (void)index;
        target = target - torso.position;
        const double yaw = torso.orientation.yaw.numerical_value_in(units::rad);
        const double pitch = torso.orientation.pitch.numerical_value_in(units::rad);
        const double roll = torso.orientation.roll.numerical_value_in(units::rad);
        const auto x = std::cos(yaw) * target.x + std::sin(yaw) * target.y;
        const auto y = -std::sin(yaw) * target.x + std::cos(yaw) * target.y;
        const auto z = std::sin(pitch) * x + std::cos(pitch) * target.z;
        target = {std::cos(pitch) * x - std::sin(pitch) * target.z, std::cos(roll) * y + std::sin(roll) * z,
                  -std::sin(roll) * y + std::cos(roll) * z};
    }
    model.moveTorso(targets, CPose());
    sensor_msgs::msg::JointState joints;
    const std::array<std::string, 6> names = {"right_front", "right_mid", "right_back",
                                              "left_front",  "left_mid",  "left_back"};
    for (size_t i = 0; i < names.size(); ++i) {
        const auto& angles = model.getLegAngles(bodyLegOrder[i]);
        joints.name.insert(joints.name.end(),
                           {names[i] + "_coxa_joint", names[i] + "_femur_joint", names[i] + "_tibia_joint"});
        joints.position.insert(joints.position.end(), {angles.torso_coxa.numerical_value_in(units::rad),
                                                       angles.coxa_femur.numerical_value_in(units::rad),
                                                       angles.femur_tibia.numerical_value_in(units::rad)});
    }
    joints.name.insert(joints.name.end(), {"head_yaw_joint", "head_pitch_joint"});
    joints.position.insert(joints.position.end(),
                           {utils::deg2rad(pose.head_pose.yaw), utils::deg2rad(pose.head_pose.pitch)});
    return joints;
}
}  // namespace rumblex_movement
