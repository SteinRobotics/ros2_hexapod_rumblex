#pragma once
#include <array>
#include <cmath>

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
    return vector_valid(msg.torso_pose.position) && angle_valid(msg.torso_pose.orientation) &&
           angle_valid(msg.head_pose) &&
           std::all_of(msg.toe_positions.begin(), msg.toe_positions.end(), vector_valid);
}
inline std::map<ELegIndex, CPosition> toeTargets(const rumblex_interfaces::msg::BodyPose& msg) {
    std::map<ELegIndex, CPosition> toes;
    for (size_t i = 0; i < bodyLegOrder.size(); ++i) {
        const auto& p = msg.toe_positions[i];
        toes.emplace(bodyLegOrder[i], CPosition(p.x, p.y, p.z));
    }
    return toes;
}
inline rumblex_interfaces::msg::BodyPose bodyPose(CBodyModel& model) {
    rumblex_interfaces::msg::BodyPose msg;
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
    msg.head_pose = angle(model.getHeadOrientation());
    msg.torso_pose.orientation = angle(model.getTorsoPose().orientation);
    msg.torso_pose.position = vector(model.getTorsoPose().position);
    const auto toes = model.getToePositions();
    for (size_t i = 0; i < bodyLegOrder.size(); ++i) msg.toe_positions[i] = vector(toes.at(bodyLegOrder[i]));
    return msg;
}
}  // namespace rumblex_geometry
