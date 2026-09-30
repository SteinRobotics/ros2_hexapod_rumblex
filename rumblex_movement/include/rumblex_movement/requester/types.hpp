/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#pragma once

#include <mp-units/math.h>

#include <algorithm>
#include <cmath>
#include <magic_enum.hpp>
#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rumblex_interfaces/msg/orientation.hpp"
#include "rumblex_interfaces/msg/pose.hpp"
#include "units.hpp"

namespace rumblex_movement {

enum class ELegIndex {
    RightFront,
    RightMid,
    RightBack,
    LeftFront,
    LeftMid,
    LeftBack,
};

inline ELegIndex legNameToIndex(std::string_view name) {
    return magic_enum::enum_cast<ELegIndex>(name).value();
}

inline std::string legIndexToName(ELegIndex index) {
    return std::string(magic_enum::enum_name(index));
}

class CPosition {
   public:
    CPosition() = default;
    CPosition(double x_m, double y_m, double z_m)
        : CPosition(x_m * units::m, y_m * units::m, z_m * units::m) {
    }
    CPosition(units::Length x, units::Length y, units::Length z) : x(x), y(y), z(z) {
    }
    ~CPosition() = default;
    CPosition operator+(const CPosition& rhs) const {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }
    CPosition operator-(const CPosition& rhs) const {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    bool operator==(const CPosition& rhs) const {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }
    bool operator!=(const CPosition& rhs) const {
        return !(*this == rhs);
    }

    static inline bool almostEqual(const CPosition& a, const CPosition& b, units::Length tol) {
        return (mp_units::abs(a.x - b.x) <= tol) && (mp_units::abs(a.y - b.y) <= tol) &&
               (mp_units::abs(a.z - b.z) <= tol);
    }

    // Lengths are stored in metres; assignments may use any compatible unit.
    units::Length x = 0.0 * units::m;
    units::Length y = 0.0 * units::m;
    units::Length z = 0.0 * units::m;

    // Linear interpolation member: returns a point between this and 'target' at parameter alpha in [0,1]
    inline CPosition linearInterpolate(const CPosition& target, double alpha) const {
        return CPosition(x + (target.x - x) * alpha, y + (target.y - y) * alpha, z + (target.z - z) * alpha);
    }
};

class COrientation {
   public:
    COrientation() = default;
    COrientation(double roll_deg, double pitch_deg, double yaw_deg)
        : COrientation(roll_deg * units::deg, pitch_deg * units::deg, yaw_deg * units::deg) {
    }
    COrientation(units::Angle roll, units::Angle pitch, units::Angle yaw)
        : roll(roll), pitch(pitch), yaw(yaw) {
    }

    COrientation(const rumblex_interfaces::msg::Orientation& orientation)
        : COrientation(orientation.roll, orientation.pitch, orientation.yaw) {
    }

    ~COrientation() = default;

    bool operator==(const COrientation& rhs) const {
        return roll == rhs.roll && pitch == rhs.pitch && yaw == rhs.yaw;
    }
    bool operator!=(const COrientation& rhs) const {
        return !(*this == rhs);
    }

    units::Angle roll = 0.0 * units::deg;
    units::Angle pitch = 0.0 * units::deg;
    units::Angle yaw = 0.0 * units::deg;

    // Linear interpolation member: returns an orientation between this and 'target' at parameter alpha in [0,1]
    inline COrientation linearInterpolate(const COrientation& target, double alpha) const {
        return COrientation(roll + (target.roll - roll) * alpha, pitch + (target.pitch - pitch) * alpha,
                            yaw + (target.yaw - yaw) * alpha);
    }
};

class CPose {
   public:
    CPose() = default;
    CPose(double x, double y, double z, double roll, double pitch, double yaw)
        : position(x, y, z), orientation(roll, pitch, yaw) {};
    CPose(units::Length x, units::Length y, units::Length z, units::Angle roll, units::Angle pitch,
          units::Angle yaw)
        : position(x, y, z), orientation(roll, pitch, yaw) {
    }
    CPose(CPosition position, COrientation orientation) : position(position), orientation(orientation) {};

    CPose(const rumblex_interfaces::msg::Pose& pose)
        : position(pose.position.x, pose.position.y, pose.position.z),
          orientation(pose.orientation.roll, pose.orientation.pitch, pose.orientation.yaw) {};

    ~CPose() = default;

    bool operator==(const CPose& rhs) const {
        return position == rhs.position && orientation == rhs.orientation;
    }

    CPosition position;
    COrientation orientation;

    // Linear interpolation member
    inline CPose linearInterpolate(const CPose& target, double alpha) const {
        CPose out;
        out.position = position.linearInterpolate(target.position, alpha);
        out.orientation = orientation.linearInterpolate(target.orientation, alpha);
        return out;
    }
};

class CBodyCenterOffset {
   public:
    units::Length x = 0.0 * units::m;
    units::Length y = 0.0 * units::m;
    units::Angle psi = 0.0 * units::deg;
};

class CLegAngles {
   public:
    CLegAngles(double coxa_deg, double femur_deg, double tibia_deg)
        : CLegAngles(coxa_deg * units::deg, femur_deg * units::deg, tibia_deg * units::deg) {
    }
    CLegAngles(units::Angle coxa, units::Angle femur, units::Angle tibia)
        : coxa(coxa), femur(femur), tibia(tibia) {
    }
    CLegAngles() = default;
    ~CLegAngles() = default;

    units::Angle coxa = 0.0 * units::deg;
    units::Angle femur = 0.0 * units::deg;
    units::Angle tibia = 0.0 * units::deg;

    // Linear interpolation member: interpolate each joint angle (degrees)
    inline CLegAngles linearInterpolate(const CLegAngles& target, double alpha) const {
        return CLegAngles(coxa + (target.coxa - coxa) * alpha, femur + (target.femur - femur) * alpha,
                          tibia + (target.tibia - tibia) * alpha);
    }
};

class CLeg {
   public:
    CLeg() = default;
    CLeg(CLegAngles angles, CPosition foot_pos) : angles_(angles), foot_pos_(foot_pos) {};

    CLegAngles angles_;
    CPosition foot_pos_;
};

// --------------------------------------------------------
// ------------------  for future usage ------------------
// struct CJointDesc {
//     double offset_rad = 0.0;
//     double limit_min_rad = -M_PI_2;
//     double limit_max_rad = M_PI_2;
//     // axis info if needed later
// };

// struct CJointState {
//     double angle_rad = 0.0;
//     bool dirty = true;
// };

// struct CLink {
//     double length_m = 0.0;
// };

// struct CSegment {
//     CLink link;         // geometry: length
//     CJointDesc desc;    // geometry/limit/offset
//     CJointState state;  // runtime
// };

// struct CLegSegmentwise {
//     CSegment coxa;
//     CSegment femur;
//     CSegment tibia;
// };

// struct CBody {
//     CPose pose;
//     std::map<ELegIndex, CLegSegmentwise> legs;
//     std::map<ELegIndex, CBodyCenterOffset> bodyCenterOffsets;
// };

}  // namespace rumblex_movement
