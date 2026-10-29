#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include "rumblex_interfaces/msg/body_pose.hpp"
#include "rumblex_utils/body_model.hpp"
namespace rumblex_geometry {
inline constexpr std::array<ELegIndex, 6> bodyLegOrder = {ELegIndex::RightFront, ELegIndex::RightMid,
                                                          ELegIndex::RightBack,  ELegIndex::LeftFront,
                                                          ELegIndex::LeftMid,    ELegIndex::LeftBack};
inline bool validBodyPose(const rumblex_interfaces::msg::BodyPose& msg) {
    auto vector_valid = [](const auto& p) {
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    };
    auto angle_valid = [](const auto& p) {
        return std::isfinite(p.roll) && std::isfinite(p.pitch) && std::isfinite(p.yaw);
    };
    return (!msg.has_torso_pose ||
            (vector_valid(msg.torso_pose.position) && angle_valid(msg.torso_pose.orientation))) &&
           (!msg.has_head_orientation || angle_valid(msg.head_orientation)) &&
           (!msg.has_toe_positions ||
            std::all_of(msg.toe_positions.begin(), msg.toe_positions.end(), vector_valid));
}
inline bool hasBodyPose(const rumblex_interfaces::msg::BodyPose& pose) {
    return pose.has_head_orientation || pose.has_torso_pose || pose.has_toe_positions;
}
inline bool completeBodyPose(const rumblex_interfaces::msg::BodyPose& pose) {
    return pose.has_head_orientation && pose.has_torso_pose && pose.has_toe_positions;
}
// Resolve at execution/activation, never while parsing or queueing a command.
inline rumblex_interfaces::msg::BodyPose resolveBodyPose(const rumblex_interfaces::msg::BodyPose& current,
                                                         const rumblex_interfaces::msg::BodyPose& target) {
    if (!completeBodyPose(current) || !validBodyPose(current) || !validBodyPose(target)) {
        throw std::invalid_argument(
            "Body pose resolution needs a complete valid current pose and valid target");
    }
    auto result = current;
    if (target.has_head_orientation) result.head_orientation = target.head_orientation;
    if (target.has_torso_pose) result.torso_pose = target.torso_pose;
    if (target.has_toe_positions) result.toe_positions = target.toe_positions;
    return result;
}

inline std::map<ELegIndex, CPosition> toeTargets(const rumblex_interfaces::msg::BodyPose& msg) {
    std::map<ELegIndex, CPosition> toes;
    for (size_t i = 0; i < bodyLegOrder.size(); ++i) {
        const auto& p = msg.toe_positions[i];
        toes.emplace(bodyLegOrder[i], CPosition(p.x, p.y, p.z));
    }
    return toes;
}
inline rumblex_interfaces::msg::BodyPose bodyPose(const std::map<ELegIndex, CPosition>& toes,
                                                  const CPose& torso, const COrientation& head) {
    rumblex_interfaces::msg::BodyPose msg;
    msg.has_head_orientation = msg.has_torso_pose = msg.has_toe_positions = true;
    auto angle = [](const COrientation& p) {
        rumblex_interfaces::msg::Orientation out;
        out.roll = p.roll.numerical_value_in(units::deg);
        out.pitch = p.pitch.numerical_value_in(units::deg);
        out.yaw = p.yaw.numerical_value_in(units::deg);
        return out;
    };
    auto vector = [](const CPosition& p) {
        geometry_msgs::msg::Vector3 out;
        out.x = p.x.numerical_value_in(units::m);
        out.y = p.y.numerical_value_in(units::m);
        out.z = p.z.numerical_value_in(units::m);
        return out;
    };
    msg.head_orientation = angle(head);
    msg.torso_pose.orientation = angle(torso.orientation);
    msg.torso_pose.position = vector(torso.position);
    for (size_t i = 0; i < bodyLegOrder.size(); ++i) msg.toe_positions[i] = vector(toes.at(bodyLegOrder[i]));
    return msg;
}
inline rumblex_interfaces::msg::BodyPose bodyPose(CBodyModel& model) {
    return bodyPose(model.getToePositions(), model.getTorsoPose(), model.getHeadOrientation());
}
}  // namespace rumblex_geometry
