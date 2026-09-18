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
#include "requester/types.hpp"

namespace rumblex_movement {

namespace leg_servo_conversion {

using rumblex_interfaces::msg::ServoAngles;
using rumblex_interfaces::msg::ServoIndex;

enum class EJointAxis { Coxa, Femur, Tibia };

struct ServoMapping {
    ELegIndex leg;
    EJointAxis axis;
    uint32_t servoIndex;
};

inline constexpr std::array<ServoMapping, 18> LEG_SERVO_MAP = {
    ServoMapping{ELegIndex::RightFront, EJointAxis::Coxa, ServoIndex::LEG_RIGHT_FRONT_COXA},
    ServoMapping{ELegIndex::RightFront, EJointAxis::Femur, ServoIndex::LEG_RIGHT_FRONT_FEMUR},
    ServoMapping{ELegIndex::RightFront, EJointAxis::Tibia, ServoIndex::LEG_RIGHT_FRONT_TIBIA},
    ServoMapping{ELegIndex::RightMid, EJointAxis::Coxa, ServoIndex::LEG_RIGHT_MID_COXA},
    ServoMapping{ELegIndex::RightMid, EJointAxis::Femur, ServoIndex::LEG_RIGHT_MID_FEMUR},
    ServoMapping{ELegIndex::RightMid, EJointAxis::Tibia, ServoIndex::LEG_RIGHT_MID_TIBIA},
    ServoMapping{ELegIndex::RightBack, EJointAxis::Coxa, ServoIndex::LEG_RIGHT_BACK_COXA},
    ServoMapping{ELegIndex::RightBack, EJointAxis::Femur, ServoIndex::LEG_RIGHT_BACK_FEMUR},
    ServoMapping{ELegIndex::RightBack, EJointAxis::Tibia, ServoIndex::LEG_RIGHT_BACK_TIBIA},
    ServoMapping{ELegIndex::LeftFront, EJointAxis::Coxa, ServoIndex::LEG_LEFT_FRONT_COXA},
    ServoMapping{ELegIndex::LeftFront, EJointAxis::Femur, ServoIndex::LEG_LEFT_FRONT_FEMUR},
    ServoMapping{ELegIndex::LeftFront, EJointAxis::Tibia, ServoIndex::LEG_LEFT_FRONT_TIBIA},
    ServoMapping{ELegIndex::LeftMid, EJointAxis::Coxa, ServoIndex::LEG_LEFT_MID_COXA},
    ServoMapping{ELegIndex::LeftMid, EJointAxis::Femur, ServoIndex::LEG_LEFT_MID_FEMUR},
    ServoMapping{ELegIndex::LeftMid, EJointAxis::Tibia, ServoIndex::LEG_LEFT_MID_TIBIA},
    ServoMapping{ELegIndex::LeftBack, EJointAxis::Coxa, ServoIndex::LEG_LEFT_BACK_COXA},
    ServoMapping{ELegIndex::LeftBack, EJointAxis::Femur, ServoIndex::LEG_LEFT_BACK_FEMUR},
    ServoMapping{ELegIndex::LeftBack, EJointAxis::Tibia, ServoIndex::LEG_LEFT_BACK_TIBIA}};

inline double getAxisAngle(const CLegAngles& legAngles, EJointAxis axis) {
    switch (axis) {
        case EJointAxis::Coxa:
            return legAngles.coxa_deg;
        case EJointAxis::Femur:
            return legAngles.femur_deg;
        case EJointAxis::Tibia:
            return legAngles.tibia_deg;
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
    targetAngles[ServoIndex::HEAD_YAW] = head.yaw_deg;
    targetAngles[ServoIndex::HEAD_PITCH] = head.pitch_deg;
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
    if (upperName.find("COXA") != std::string::npos) return EJointAxis::Coxa;
    if (upperName.find("FEMUR") != std::string::npos) return EJointAxis::Femur;
    if (upperName.find("TIBIA") != std::string::npos) return EJointAxis::Tibia;
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
            case EJointAxis::Coxa:
                entry.coxa_deg = servo.angle_deg;
                break;
            case EJointAxis::Femur:
                entry.femur_deg = servo.angle_deg;
                break;
            case EJointAxis::Tibia:
                entry.tibia_deg = servo.angle_deg;
                break;
        }
    }
    return legAngles;
}

}  // namespace leg_servo_conversion

}  // namespace rumblex_movement