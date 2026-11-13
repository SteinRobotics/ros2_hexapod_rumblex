#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

#include "interpolation.hpp"
#include "rumblex_interfaces/msg/body_pose.hpp"
#include "rumblex_utils/units.hpp"
namespace brain {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
namespace units = rumblex_geometry::units;
inline constexpr units::Duration kDefaultPoseDuration = 1.0 * s;
inline constexpr units::Duration kMinimumAbsolutePoseDuration = 0.1 * s;

// Deltas are resolved from the current pose when SINGLE_POSE starts.
struct PoseDelta {
    geometry_msgs::msg::Vector3 torso_translation;
    rumblex_interfaces::msg::Orientation head_rotation;
    rumblex_interfaces::msg::Orientation torso_rotation;
    std::array<geometry_msgs::msg::Vector3, 6> toe_translation;
    std::array<double, 6> coxa_rotation_deg{};

    bool valid() const {
        const auto finite = [](const auto& v) {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        };
        const auto finiteRotation = [](const auto& v) {
            return std::isfinite(v.roll) && std::isfinite(v.pitch) && std::isfinite(v.yaw);
        };
        if (!finite(torso_translation) || !finiteRotation(head_rotation) || !finiteRotation(torso_rotation))
            return false;
        for (size_t i = 0; i < 6; ++i)
            if (!finite(toe_translation[i]) || !std::isfinite(coxa_rotation_deg[i])) return false;
        return true;
    }
};
struct MovementRequest {
    enum Type : uint8_t {
        NO_REQUEST = 0,
        SINGLE_POSE = 1,
        SEQUENCE_LAYDOWN = 10,
        SEQUENCE_STAND_UP = 11,
        SEQUENCE_WAITING = 12,
        SEQUENCE_WATCH = 13,
        // 14 was the retired look gait; use the named left/right look sequences.
        SEQUENCE_DANCE = 15,
        // 16 was the retired high-five gait; use the high_five sequence.
        SEQUENCE_LEGS_WAVE = 17,
        SEQUENCE_BODY_ROLL = 18,
        SEQUENCE_BITE = 19,
        SEQUENCE_STOMP = 20,
        // 21 was the retired clap gait; clap is now a named sequence.
        SEQUENCE_TESTLEGS = 22,
        CONTINUOUS_POSE = 30,
        MOVE_COMBINED = 31,
        MOVE_TRIPOD = 32,
        HOLD_DISTANCE = 33,
        LEVEL_TORSO = 34,
        MOVE_RIPPLE = 35,
        MOVE_WAVE = 36,
        MOVE_CRAWL = 37,
    };
    enum Direction : uint8_t { CLOCKWISE = 0, ANTICLOCKWISE = 1 };
    using _type_type = uint8_t;
    uint8_t type = NO_REQUEST;
    uint8_t direction = CLOCKWISE;
    units::Duration duration_s = kDefaultPoseDuration;
    std::string name;
    InterpolationType interpolation = InterpolationType::Linear;
    std::optional<units::Length> distance_m;
    std::optional<PoseDelta> pose_delta;
    // Optional absolute (torso_coxa, coxa_femur, femur_tibia) angles per leg, in degrees.
    std::array<std::optional<std::array<double, 3>>, 6> joint_targets{};
    bool save_pose = false;
    bool restore_pose = false;

    bool hasJointTargets() const {
        for (const auto& target : joint_targets)
            if (target) return true;
        return false;
    }
    bool hasBodyTarget() const {
        return body_pose.has_torso_pose || body_pose.has_head_orientation || body_pose.has_toe_positions;
    }
    bool hasPoseTarget() const {
        return hasBodyTarget() || pose_delta || hasJointTargets() || restore_pose;
    }
    bool hasFiniteDuration() const {
        return std::isfinite(duration_s.numerical_value_in(s));
    }
    bool hasPositiveDuration() const {
        return hasFiniteDuration() && duration_s > 0.0 * s;
    }
    bool validToeTargetType() const {
        return type == SINGLE_POSE || !body_pose.has_toe_positions;
    }
    bool validRelativePose() const {
        return !pose_delta ||
               (type == SINGLE_POSE && !hasBodyTarget() && pose_delta->valid() && hasPositiveDuration());
    }
    bool isSensorGait() const {
        return type == HOLD_DISTANCE || type == LEVEL_TORSO;
    }
    bool validSensorPose() const {
        return !isSensorGait() || (hasPositiveDuration() && !hasBodyTarget());
    }
    bool validDistance() const {
        return type == HOLD_DISTANCE
                   ? distance_m && std::isfinite(distance_m->numerical_value_in(m)) && *distance_m > 0.0 * m
                   : !distance_m;
    }
    bool validPoseOptions() const {
        for (const auto& target : joint_targets)
            if (target)
                for (double angle : *target)
                    if (!std::isfinite(angle)) return false;
        if (restore_pose && (save_pose || hasBodyTarget() || pose_delta || hasJointTargets())) return false;
        if (hasJointTargets() && (pose_delta || body_pose.has_toe_positions)) return false;
        return !(save_pose || restore_pose || hasJointTargets()) ||
               (type == SINGLE_POSE && hasPositiveDuration());
    }
    rumblex_interfaces::msg::BodyPose body_pose{};
};
constexpr bool isLocomotion(uint32_t type) {
    return type == MovementRequest::MOVE_COMBINED || type == MovementRequest::MOVE_TRIPOD ||
           type == MovementRequest::MOVE_RIPPLE || type == MovementRequest::MOVE_WAVE ||
           type == MovementRequest::MOVE_CRAWL;
}
}  // namespace brain
