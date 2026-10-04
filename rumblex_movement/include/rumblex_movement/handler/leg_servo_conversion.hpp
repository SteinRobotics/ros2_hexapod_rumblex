/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <map>
#include <optional>
#include <string>
#include <string_view>

#include "rumblex_interfaces/msg/servo_angles.hpp"
#include "rumblex_interfaces/msg/servo_index.hpp"
#include "rumblex_utils/body_types.hpp"

namespace rumblex_movement {

namespace leg_servo_conversion {

using rumblex_interfaces::msg::ServoAngles;
using rumblex_interfaces::msg::ServoIndex;

enum class EJointAxis { TorsoCoxa, CoxaFemur, FemurTibia };

struct ServoMapping {
    rumblex_geometry::ELegIndex leg;
    EJointAxis axis;
    uint32_t servo_index;
};

inline constexpr std::array<ServoMapping, 18> LEG_SERVO_MAP = {
    ServoMapping{rumblex_geometry::ELegIndex::RightFront, EJointAxis::TorsoCoxa,
                 ServoIndex::LEG_RIGHT_FRONT_COXA},
    ServoMapping{rumblex_geometry::ELegIndex::RightFront, EJointAxis::CoxaFemur,
                 ServoIndex::LEG_RIGHT_FRONT_FEMUR},
    ServoMapping{rumblex_geometry::ELegIndex::RightFront, EJointAxis::FemurTibia,
                 ServoIndex::LEG_RIGHT_FRONT_TIBIA},
    ServoMapping{rumblex_geometry::ELegIndex::RightMid, EJointAxis::TorsoCoxa,
                 ServoIndex::LEG_RIGHT_MID_COXA},
    ServoMapping{rumblex_geometry::ELegIndex::RightMid, EJointAxis::CoxaFemur,
                 ServoIndex::LEG_RIGHT_MID_FEMUR},
    ServoMapping{rumblex_geometry::ELegIndex::RightMid, EJointAxis::FemurTibia,
                 ServoIndex::LEG_RIGHT_MID_TIBIA},
    ServoMapping{rumblex_geometry::ELegIndex::RightBack, EJointAxis::TorsoCoxa,
                 ServoIndex::LEG_RIGHT_BACK_COXA},
    ServoMapping{rumblex_geometry::ELegIndex::RightBack, EJointAxis::CoxaFemur,
                 ServoIndex::LEG_RIGHT_BACK_FEMUR},
    ServoMapping{rumblex_geometry::ELegIndex::RightBack, EJointAxis::FemurTibia,
                 ServoIndex::LEG_RIGHT_BACK_TIBIA},
    ServoMapping{rumblex_geometry::ELegIndex::LeftFront, EJointAxis::TorsoCoxa,
                 ServoIndex::LEG_LEFT_FRONT_COXA},
    ServoMapping{rumblex_geometry::ELegIndex::LeftFront, EJointAxis::CoxaFemur,
                 ServoIndex::LEG_LEFT_FRONT_FEMUR},
    ServoMapping{rumblex_geometry::ELegIndex::LeftFront, EJointAxis::FemurTibia,
                 ServoIndex::LEG_LEFT_FRONT_TIBIA},
    ServoMapping{rumblex_geometry::ELegIndex::LeftMid, EJointAxis::TorsoCoxa, ServoIndex::LEG_LEFT_MID_COXA},
    ServoMapping{rumblex_geometry::ELegIndex::LeftMid, EJointAxis::CoxaFemur, ServoIndex::LEG_LEFT_MID_FEMUR},
    ServoMapping{rumblex_geometry::ELegIndex::LeftMid, EJointAxis::FemurTibia,
                 ServoIndex::LEG_LEFT_MID_TIBIA},
    ServoMapping{rumblex_geometry::ELegIndex::LeftBack, EJointAxis::TorsoCoxa,
                 ServoIndex::LEG_LEFT_BACK_COXA},
    ServoMapping{rumblex_geometry::ELegIndex::LeftBack, EJointAxis::CoxaFemur,
                 ServoIndex::LEG_LEFT_BACK_FEMUR},
    ServoMapping{rumblex_geometry::ELegIndex::LeftBack, EJointAxis::FemurTibia,
                 ServoIndex::LEG_LEFT_BACK_TIBIA}};

inline double getAxisAngle(const rumblex_geometry::CLegAngles& leg_angles, EJointAxis axis) {
    switch (axis) {
        case EJointAxis::TorsoCoxa:
            return leg_angles.torso_coxa.numerical_value_in(rumblex_geometry::units::deg);
        case EJointAxis::CoxaFemur:
            return leg_angles.coxa_femur.numerical_value_in(rumblex_geometry::units::deg);
        case EJointAxis::FemurTibia:
            return leg_angles.femur_tibia.numerical_value_in(rumblex_geometry::units::deg);
    }
    return 0.0;
}

inline void appendLegServoTargets(
    const std::map<rumblex_geometry::ELegIndex, rumblex_geometry::CLegAngles>& leg_angles,
    std::map<uint32_t, double>& target_angles) {
    for (const auto& entry : LEG_SERVO_MAP) {
        auto it = leg_angles.find(entry.leg);
        if (it == leg_angles.end()) continue;
        target_angles[entry.servo_index] = getAxisAngle(it->second, entry.axis);
    }
}

inline void appendHeadServoTargets(const rumblex_geometry::COrientation& head,
                                   std::map<uint32_t, double>& target_angles) {
    target_angles[ServoIndex::HEAD_YAW] = head.yaw.numerical_value_in(rumblex_geometry::units::deg);
    target_angles[ServoIndex::HEAD_PITCH] = head.pitch.numerical_value_in(rumblex_geometry::units::deg);
}

inline std::map<uint32_t, double> buildServoTargets(
    const rumblex_geometry::COrientation& head,
    const std::map<rumblex_geometry::ELegIndex, rumblex_geometry::CLegAngles>& leg_angles) {
    std::map<uint32_t, double> targets;
    appendHeadServoTargets(head, targets);
    appendLegServoTargets(leg_angles, targets);
    return targets;
}

inline std::string toUpperCopy(std::string_view text) {
    std::string result(text.begin(), text.end());
    std::transform(result.begin(), result.end(), result.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return result;
}

inline std::optional<rumblex_geometry::ELegIndex> parseLegIndexFromUpperName(std::string_view upper_name) {
    static constexpr std::array<std::pair<std::string_view, rumblex_geometry::ELegIndex>, 10> LEG_KEYWORDS = {
        std::pair{"RIGHT_FRONT", rumblex_geometry::ELegIndex::RightFront},
        std::pair{"RIGHT_MID", rumblex_geometry::ELegIndex::RightMid},
        std::pair{"RIGHT_MIDDLE", rumblex_geometry::ELegIndex::RightMid},
        std::pair{"RIGHT_BACK", rumblex_geometry::ELegIndex::RightBack},
        std::pair{"RIGHT_REAR", rumblex_geometry::ELegIndex::RightBack},
        std::pair{"LEFT_FRONT", rumblex_geometry::ELegIndex::LeftFront},
        std::pair{"LEFT_MID", rumblex_geometry::ELegIndex::LeftMid},
        std::pair{"LEFT_MIDDLE", rumblex_geometry::ELegIndex::LeftMid},
        std::pair{"LEFT_BACK", rumblex_geometry::ELegIndex::LeftBack},
        std::pair{"LEFT_REAR", rumblex_geometry::ELegIndex::LeftBack}};

    for (const auto& [keyword, leg] : LEG_KEYWORDS) {
        if (upper_name.find(keyword) != std::string::npos) return leg;
    }
    return std::nullopt;
}

inline std::optional<EJointAxis> parseJointAxisFromUpperName(std::string_view upper_name) {
    if (upper_name.find("COXA") != std::string::npos) return EJointAxis::TorsoCoxa;
    if (upper_name.find("FEMUR") != std::string::npos) return EJointAxis::CoxaFemur;
    if (upper_name.find("TIBIA") != std::string::npos) return EJointAxis::FemurTibia;
    return std::nullopt;
}

inline std::map<rumblex_geometry::ELegIndex, rumblex_geometry::CLegAngles> servoAnglesMsgToLegAngles(
    const ServoAngles& msg) {
    std::map<rumblex_geometry::ELegIndex, rumblex_geometry::CLegAngles> leg_angles;
    for (const auto& servo : msg.current_angles) {
        if (servo.name.empty()) continue;
        std::string upper = toUpperCopy(servo.name);
        auto leg = parseLegIndexFromUpperName(upper);
        auto axis = parseJointAxisFromUpperName(upper);
        if (!leg || !axis) continue;
        rumblex_geometry::CLegAngles& entry = leg_angles[*leg];
        switch (*axis) {
            case EJointAxis::TorsoCoxa:
                entry.torso_coxa = servo.angle_deg * rumblex_geometry::units::deg;
                break;
            case EJointAxis::CoxaFemur:
                entry.coxa_femur = servo.angle_deg * rumblex_geometry::units::deg;
                break;
            case EJointAxis::FemurTibia:
                entry.femur_tibia = servo.angle_deg * rumblex_geometry::units::deg;
                break;
        }
    }
    return leg_angles;
}

}  // namespace leg_servo_conversion

}  // namespace rumblex_movement