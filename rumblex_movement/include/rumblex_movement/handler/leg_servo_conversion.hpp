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

#include "requester/types.hpp"
#include "rumblex_interfaces/msg/servo_angles.hpp"
#include "rumblex_interfaces/msg/servo_index.hpp"

namespace rumblex_movement {

namespace leg_servo_conversion {

using rumblex_interfaces::msg::ServoAngles;
using rumblex_interfaces::msg::ServoIndex;

enum class EJointAxis { TorsoCoxa, CoxaFemur, FemurTibia };

struct ServoMapping {
    ELegIndex leg;
    EJointAxis axis;
    uint32_t servoIndex;
};

inline constexpr std::array<ServoMapping, 18> LEG_SERVO_MAP = {
    ServoMapping{ELegIndex::RightFront, EJointAxis::TorsoCoxa, ServoIndex::LEG_RIGHT_FRONT_COXA},
    ServoMapping{ELegIndex::RightFront, EJointAxis::CoxaFemur, ServoIndex::LEG_RIGHT_FRONT_FEMUR},
    ServoMapping{ELegIndex::RightFront, EJointAxis::FemurTibia, ServoIndex::LEG_RIGHT_FRONT_TIBIA},
    ServoMapping{ELegIndex::RightMid, EJointAxis::TorsoCoxa, ServoIndex::LEG_RIGHT_MID_COXA},
    ServoMapping{ELegIndex::RightMid, EJointAxis::CoxaFemur, ServoIndex::LEG_RIGHT_MID_FEMUR},
    ServoMapping{ELegIndex::RightMid, EJointAxis::FemurTibia, ServoIndex::LEG_RIGHT_MID_TIBIA},
    ServoMapping{ELegIndex::RightBack, EJointAxis::TorsoCoxa, ServoIndex::LEG_RIGHT_BACK_COXA},
    ServoMapping{ELegIndex::RightBack, EJointAxis::CoxaFemur, ServoIndex::LEG_RIGHT_BACK_FEMUR},
    ServoMapping{ELegIndex::RightBack, EJointAxis::FemurTibia, ServoIndex::LEG_RIGHT_BACK_TIBIA},
    ServoMapping{ELegIndex::LeftFront, EJointAxis::TorsoCoxa, ServoIndex::LEG_LEFT_FRONT_COXA},
    ServoMapping{ELegIndex::LeftFront, EJointAxis::CoxaFemur, ServoIndex::LEG_LEFT_FRONT_FEMUR},
    ServoMapping{ELegIndex::LeftFront, EJointAxis::FemurTibia, ServoIndex::LEG_LEFT_FRONT_TIBIA},
    ServoMapping{ELegIndex::LeftMid, EJointAxis::TorsoCoxa, ServoIndex::LEG_LEFT_MID_COXA},
    ServoMapping{ELegIndex::LeftMid, EJointAxis::CoxaFemur, ServoIndex::LEG_LEFT_MID_FEMUR},
    ServoMapping{ELegIndex::LeftMid, EJointAxis::FemurTibia, ServoIndex::LEG_LEFT_MID_TIBIA},
    ServoMapping{ELegIndex::LeftBack, EJointAxis::TorsoCoxa, ServoIndex::LEG_LEFT_BACK_COXA},
    ServoMapping{ELegIndex::LeftBack, EJointAxis::CoxaFemur, ServoIndex::LEG_LEFT_BACK_FEMUR},
    ServoMapping{ELegIndex::LeftBack, EJointAxis::FemurTibia, ServoIndex::LEG_LEFT_BACK_TIBIA}};

inline double getAxisAngle(const CLegAngles& legAngles, EJointAxis axis) {
    switch (axis) {
        case EJointAxis::TorsoCoxa:
            return legAngles.torso_coxa.numerical_value_in(units::deg);
        case EJointAxis::CoxaFemur:
            return legAngles.coxa_femur.numerical_value_in(units::deg);
        case EJointAxis::FemurTibia:
            return legAngles.femur_tibia.numerical_value_in(units::deg);
    }
    return 0.0;
}

inline void appendLegServoTargets(const std::map<ELegIndex, CLegAngles>& legAngles,
                                  std::map<uint32_t, double>& targetAngles) {
    for (const auto& entry : LEG_SERVO_MAP) {
        auto it = legAngles.find(entry.leg);
        if (it == legAngles.end()) continue;
        targetAngles[entry.servoIndex] = getAxisAngle(it->second, entry.axis);
    }
}

inline void appendHeadServoTargets(const COrientation& head, std::map<uint32_t, double>& targetAngles) {
    targetAngles[ServoIndex::HEAD_YAW] = head.yaw.numerical_value_in(units::deg);
    targetAngles[ServoIndex::HEAD_PITCH] = head.pitch.numerical_value_in(units::deg);
}

inline std::map<uint32_t, double> buildServoTargets(const COrientation& head,
                                                    const std::map<ELegIndex, CLegAngles>& legAngles) {
    std::map<uint32_t, double> targets;
    appendHeadServoTargets(head, targets);
    appendLegServoTargets(legAngles, targets);
    return targets;
}

inline std::string toUpperCopy(std::string_view text) {
    std::string result(text.begin(), text.end());
    std::transform(result.begin(), result.end(), result.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return result;
}

inline std::optional<ELegIndex> parseLegIndexFromUpperName(std::string_view upperName) {
    static constexpr std::array<std::pair<std::string_view, ELegIndex>, 10> LEG_KEYWORDS = {
        std::pair{"RIGHT_FRONT", ELegIndex::RightFront}, std::pair{"RIGHT_MID", ELegIndex::RightMid},
        std::pair{"RIGHT_MIDDLE", ELegIndex::RightMid},  std::pair{"RIGHT_BACK", ELegIndex::RightBack},
        std::pair{"RIGHT_REAR", ELegIndex::RightBack},   std::pair{"LEFT_FRONT", ELegIndex::LeftFront},
        std::pair{"LEFT_MID", ELegIndex::LeftMid},       std::pair{"LEFT_MIDDLE", ELegIndex::LeftMid},
        std::pair{"LEFT_BACK", ELegIndex::LeftBack},     std::pair{"LEFT_REAR", ELegIndex::LeftBack}};

    for (const auto& [keyword, leg] : LEG_KEYWORDS) {
        if (upperName.find(keyword) != std::string::npos) return leg;
    }
    return std::nullopt;
}

inline std::optional<EJointAxis> parseJointAxisFromUpperName(std::string_view upperName) {
    if (upperName.find("COXA") != std::string::npos) return EJointAxis::TorsoCoxa;
    if (upperName.find("FEMUR") != std::string::npos) return EJointAxis::CoxaFemur;
    if (upperName.find("TIBIA") != std::string::npos) return EJointAxis::FemurTibia;
    return std::nullopt;
}

inline std::map<ELegIndex, CLegAngles> servoAnglesMsgToLegAngles(const ServoAngles& msg) {
    std::map<ELegIndex, CLegAngles> legAngles;
    for (const auto& servo : msg.current_angles) {
        if (servo.name.empty()) continue;
        std::string upper = toUpperCopy(servo.name);
        auto leg = parseLegIndexFromUpperName(upper);
        auto axis = parseJointAxisFromUpperName(upper);
        if (!leg || !axis) continue;
        CLegAngles& entry = legAngles[*leg];
        switch (*axis) {
            case EJointAxis::TorsoCoxa:
                entry.torso_coxa = servo.angle_deg * units::deg;
                break;
            case EJointAxis::CoxaFemur:
                entry.coxa_femur = servo.angle_deg * units::deg;
                break;
            case EJointAxis::FemurTibia:
                entry.femur_tibia = servo.angle_deg * units::deg;
                break;
        }
    }
    return legAngles;
}

}  // namespace leg_servo_conversion

}  // namespace rumblex_movement