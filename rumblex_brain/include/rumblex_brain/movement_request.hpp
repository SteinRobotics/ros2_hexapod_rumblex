#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

#include "rumblex_interfaces/msg/body_pose.hpp"
#include "rumblex_utils/units.hpp"
namespace brain {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
namespace units = rumblex_geometry::units;
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
        CONTINUOUS_MOVE = 31,
        CONTINUOUS_RUNNING = 32,
        HOLD_DISTANCE = 33,
        LEVEL_TORSO = 34,
    };
    enum Direction : uint8_t { CLOCKWISE = 0, ANTICLOCKWISE = 1 };
    using _type_type = uint8_t;
    uint8_t type = NO_REQUEST;
    uint8_t direction = CLOCKWISE;
    units::Duration duration_s = 1.0 * s;
    std::string name;
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
    bool validPoseOptions() const {
        for (const auto& target : joint_targets)
            if (target)
                for (double angle : *target)
                    if (!std::isfinite(angle)) return false;
        const bool has_pose =
            body_pose.has_torso_pose || body_pose.has_head_orientation || body_pose.has_toe_positions;
        if (restore_pose && (save_pose || has_pose || pose_delta || hasJointTargets())) return false;
        if (hasJointTargets() && (pose_delta || body_pose.has_toe_positions)) return false;
        return !(save_pose || restore_pose || hasJointTargets()) ||
               (type == SINGLE_POSE && std::isfinite(duration_s.numerical_value_in(s)) &&
                duration_s > 0.0 * s);
    }
    rumblex_interfaces::msg::BodyPose body_pose{};
};
}  // namespace brain
