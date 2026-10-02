/*******************************************************************************
 * Copyright (c) 2024 Christian Stein
 ******************************************************************************/

#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rumblex_interfaces/msg/pose.hpp"
#include "rumblex_utils/body_types.hpp"
#include "rumblex_utils/geometry.hpp"
#include "rumblex_utils/units.hpp"

namespace rumblex_geometry {

// Shared geometry model. Toe positions always remain in the BodyPose command
// frame; IK temporaries and forward-kinematics results are converted at the boundary.
class CBodyModel {
   public:
    explicit CBodyModel(std::shared_ptr<rclcpp::Node> node);
    ~CBodyModel() = default;

    void setToePosition(const ELegIndex index, const CPosition& target_toe_pos);
    void setLegAngles(const ELegIndex index, const CLegAngles& angles);
    void moveTorso(const std::map<ELegIndex, CPosition>& toe_targets,
                   const CPose torso = CPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    void moveTorso(const CPose torso);

    void setHeadOrientation(units::Angle yaw, units::Angle pitch);
    void setHeadOrientation(COrientation head);

    std::map<ELegIndex, CPosition> getToePositions() const;
    std::map<ELegIndex, CPosition> getStandingToePositions() const;
    std::map<ELegIndex, CPosition> getLaydownToePositions() const;

    std::map<ELegIndex, CLeg>& getLegs();
    CLegAngles& getLegAngles(ELegIndex index);
    std::map<ELegIndex, CLegAngles> getLegAngles();

    COrientation& getHeadOrientation() {
        return head_orientation_;
    };

    CPose& getTorsoPose() {
        return torso_pose_;
    };

   private:
    void initializeLegs(const std::map<ELegIndex, CPosition>& toe_targets, const CPose torso,
                        std::map<ELegIndex, CLeg>& legs);

    void logLegsPositions(std::map<ELegIndex, CLeg>& legs);
    void logLegPosition(const ELegIndex index, const CLeg& leg);
    void logHeadOrientation();
    void calcLegInverseKinematics(const CPosition& target_toe_pos, CLeg& leg, const ELegIndex& leg_index);
    void calcLegForwardKinematics(const CLegAngles target, CLeg& leg);
    CPosition rotate(const CPosition& point, const COrientation& rot);

    std::shared_ptr<rclcpp::Node> node_;

    // Parameters
    const units::Length coxa_length_;
    const units::Length coxa_height_;
    const units::Length femur_length_;
    const units::Length tibia_length_;
    const units::Area sq_femur_length_;
    const units::Area sq_tibia_length_;

    std::map<ELegIndex, CLeg> legs_;           // current values
    std::map<ELegIndex, CLeg> standing_legs_;  // change to shared pointer and make const
    std::map<ELegIndex, CLeg> laydown_legs_;   // change to shared pointer and make const
    std::map<ELegIndex, CTorsoCenterOffset> torso_center_offsets_;

    CPose torso_pose_ = {};
    COrientation head_orientation_ = {};
};

}  // namespace rumblex_geometry

namespace rumblex_geometry {

using units::deg;
using units::m;

inline constexpr bool LOG_KINEMATICS_LEG_ACTIVE = false;
inline constexpr bool LOG_KINEMATICS_HEAD_ACTIVE = false;

inline CBodyModel::CBodyModel(std::shared_ptr<rclcpp::Node> node)
    : node_(node),
      coxa_length_(node->declare_parameter<double>("coxa_length_m", rclcpp::PARAMETER_DOUBLE) * m),
      coxa_height_(node->declare_parameter<double>("coxa_height_m", rclcpp::PARAMETER_DOUBLE) * m),
      femur_length_(node->declare_parameter<double>("femur_length_m", rclcpp::PARAMETER_DOUBLE) * m),
      tibia_length_(node->declare_parameter<double>("tibia_length_m", rclcpp::PARAMETER_DOUBLE) * m),
      sq_femur_length_(femur_length_ * femur_length_),
      sq_tibia_length_(tibia_length_ * tibia_length_) {
    std::map<ELegIndex, std::string> leg_parameter_keys;
    for (auto leg_index : magic_enum::enum_values<ELegIndex>()) {
        const std::string canonical_name = legIndexToName(leg_index);
        std::string parameter_suffix =
            node_->declare_parameter<std::string>("leg_names." + canonical_name, canonical_name);
        leg_parameter_keys[leg_index] = parameter_suffix;
    }

    auto load_torso_center_offsets_from_parameters =
        [&](const std::string& parameter_root, const std::map<ELegIndex, std::string>& leg_parameter_names) {
            std::map<ELegIndex, CTorsoCenterOffset> offsets;

            for (const auto& [leg_index, parameter_suffix] : leg_parameter_names) {
                const std::string prefix = parameter_root + "." + parameter_suffix;
                CTorsoCenterOffset offset;
                offset.x = node_->declare_parameter<double>(prefix + ".x_m", 0.0) * m;
                offset.y = node_->declare_parameter<double>(prefix + ".y_m", 0.0) * m;
                offset.psi = node_->declare_parameter<double>(prefix + ".yaw_deg", 0.0) * deg;
                offsets[leg_index] = offset;
            }

            RCLCPP_INFO_STREAM(node_->get_logger(), "Loaded " << parameter_root
                                                              << " offsets from parameters ("
                                                              << offsets.size() << " entries).");
            return offsets;
        };

    auto load_toe_positions_from_parameters =
        [&](const std::string& parameter_root, const std::map<ELegIndex, std::string>& leg_parameter_names) {
            std::map<ELegIndex, CPosition> positions;
            for (const auto& [leg_index, parameter_suffix] : leg_parameter_names) {
                const std::string prefix = parameter_root + "." + parameter_suffix;
                positions[leg_index] = CPosition(node_->declare_parameter<double>(prefix + ".x", 0.0),
                                                 node_->declare_parameter<double>(prefix + ".y", 0.0),
                                                 node_->declare_parameter<double>(prefix + ".z", 0.0));
            }
            return positions;
        };

    torso_center_offsets_ = load_torso_center_offsets_from_parameters("leg_offsets", leg_parameter_keys);

    const auto toe_positions_standing =
        load_toe_positions_from_parameters("toe_positions_standing", leg_parameter_keys);
    initializeLegs(toe_positions_standing, torso_pose_, standing_legs_);

    const auto toe_positions_laydown =
        load_toe_positions_from_parameters("toe_positions_laydown", leg_parameter_keys);
    initializeLegs(toe_positions_laydown, torso_pose_, laydown_legs_);

    // Initialize current leg positions to laydown (robot starts laying down)
    initializeLegs(toe_positions_laydown, torso_pose_, legs_);
}

inline void CBodyModel::logLegsPositions(std::map<ELegIndex, CLeg>& legs) {
    if (!LOG_KINEMATICS_LEG_ACTIVE) return;
    RCLCPP_INFO_STREAM(node_->get_logger(), "----------------------------------------");
    for (const auto& [index, leg] : legs) {
        logLegPosition(index, leg);
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "----------------------------------------");
}

inline void CBodyModel::logLegPosition(const ELegIndex index, const CLeg& leg) {
    if (!LOG_KINEMATICS_LEG_ACTIVE) return;
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       magic_enum::enum_name(index)
                           << ": \tag: " << std::fixed << std::setprecision(3) << std::setw(3)
                           << leg.angles.torso_coxa.numerical_value_in(deg) << "°, " << std::setw(3)
                           << leg.angles.coxa_femur.numerical_value_in(deg) << "°, " << std::setw(3)
                           << leg.angles.femur_tibia.numerical_value_in(deg) << "°\t| x: " << std::fixed
                           << std::setprecision(3) << std::setw(3) << leg.toe_position.x.numerical_value_in(m)
                           << ", y: " << std::setw(3) << leg.toe_position.y.numerical_value_in(m)
                           << ", z: " << std::setw(3) << leg.toe_position.z.numerical_value_in(m));
}

inline void CBodyModel::logHeadOrientation() {
    if (!LOG_KINEMATICS_HEAD_ACTIVE) return;
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "Head: \tYaw: " << std::fixed << std::setprecision(3) << std::setw(3)
                                       << head_orientation_.yaw.numerical_value_in(deg)
                                       << "°, Pitch: " << std::setw(3)
                                       << head_orientation_.pitch.numerical_value_in(deg) << "°");
}

inline void CBodyModel::initializeLegs(const std::map<ELegIndex, CPosition>& toe_targets, const CPose torso,
                                       std::map<ELegIndex, CLeg>& legs) {
    for (const auto& [leg_index, toe_target] : toe_targets) {
        RCLCPP_DEBUG_STREAM(node_->get_logger(),
                            "Initializing leg "
                                << magic_enum::enum_name(leg_index)
                                << " to toe target position x: " << toe_target.x.numerical_value_in(m)
                                << ", y: " << toe_target.y.numerical_value_in(m)
                                << ", z: " << toe_target.z.numerical_value_in(m));
        auto leg = CLeg();
        CPosition coxa_position(torso_center_offsets_.at(leg_index).x, torso_center_offsets_.at(leg_index).y,
                                0.0 * m);
        CPosition leg_base = rotate(coxa_position, torso.orientation) + torso.position;
        CPosition toe_rel = toe_target - leg_base;
        calcLegInverseKinematics(toe_rel, leg, leg_index);
        leg.toe_position = toe_target;
        legs[leg_index] = leg;
    }
    logLegsPositions(legs);
}

inline void CBodyModel::moveTorso(const std::map<ELegIndex, CPosition>& toe_targets, const CPose torso) {
    torso_pose_ = torso;

    for (auto& [leg_index, toe_target] : toe_targets) {
        auto& leg = legs_.at(leg_index);
        CPosition coxa_position(torso_center_offsets_.at(leg_index).x, torso_center_offsets_.at(leg_index).y,
                                0.0 * m);
        CPosition leg_base = rotate(coxa_position, torso.orientation) + torso.position;
        CPosition toe_rel = toe_target - leg_base;
        calcLegInverseKinematics(toe_rel, leg, leg_index);
        leg.toe_position = toe_target;
    }
    logLegsPositions(legs_);
}

inline void CBodyModel::moveTorso(const CPose torso) {
    torso_pose_ = torso;

    auto toe_targets = getStandingToePositions();

    for (auto& [leg_index, toe_target] : toe_targets) {
        auto& leg = legs_.at(leg_index);
        CPosition coxa_position(torso_center_offsets_.at(leg_index).x, torso_center_offsets_.at(leg_index).y,
                                0.0 * m);
        CPosition leg_base = rotate(coxa_position, torso.orientation) + torso.position;
        CPosition toe_rel = toe_target - leg_base;
        calcLegInverseKinematics(toe_rel, leg, leg_index);
        leg.toe_position = toe_target;
    }
    logLegsPositions(legs_);
}

inline CPosition CBodyModel::rotate(const CPosition& point, const COrientation& orientation) {
    const auto px = point.x;
    const auto py = point.y;
    const auto pz = point.z;

    const auto cos_roll = mp_units::angular::cos(orientation.roll);
    const auto sin_roll = mp_units::angular::sin(orientation.roll);
    const auto cos_pitch = mp_units::angular::cos(orientation.pitch);
    const auto sin_pitch = mp_units::angular::sin(orientation.pitch);
    const auto cos_yaw = mp_units::angular::cos(orientation.yaw);
    const auto sin_yaw = mp_units::angular::sin(orientation.yaw);

    // Standard ZYX (yaw-pitch-roll) rotation matrix
    const auto rotated_x = cos_yaw * cos_pitch * px +
                           (cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll) * py +
                           (cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll) * pz;

    const auto rotated_y = sin_yaw * cos_pitch * px +
                           (sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll) * py +
                           (sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll) * pz;

    const auto rotated_z = -sin_pitch * px + cos_pitch * sin_roll * py + cos_pitch * cos_roll * pz;

    return {rotated_x, rotated_y, rotated_z};
}

inline void CBodyModel::calcLegInverseKinematics(const CPosition& target_toe_pos, CLeg& leg,
                                                 const ELegIndex& leg_index) {
    leg.toe_position = target_toe_pos;

    const auto torso_coxa_heading = mp_units::angular::atan2(target_toe_pos.x, target_toe_pos.y);
    const auto z_offset = coxa_height_ - target_toe_pos.z;

    const auto l_leg_top_view = mp_units::hypot(target_toe_pos.x, target_toe_pos.y);  // L1

    const auto horizontal = l_leg_top_view - coxa_length_;
    const auto sq_l = z_offset * z_offset + horizontal * horizontal;
    const auto L = mp_units::sqrt(sq_l);

    auto coxa_femur_cosine = (sq_tibia_length_ - sq_femur_length_ - sq_l) / (-2 * femur_length_ * L);
    if (mp_units::abs(coxa_femur_cosine) > 1.0 * mp_units::one) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "calcLegInverseKinematics: clamping coxa_femur input "
                                << coxa_femur_cosine.numerical_value_in(mp_units::one) << " for leg "
                                << magic_enum::enum_name(leg_index)
                                << " (target x: " << target_toe_pos.x.numerical_value_in(m)
                                << ", y: " << target_toe_pos.y.numerical_value_in(m)
                                << ", z: " << target_toe_pos.z.numerical_value_in(m) << ")");
        coxa_femur_cosine = std::clamp(coxa_femur_cosine, -1.0 * mp_units::one, 1.0 * mp_units::one);
    }
    const auto coxa_femur =
        mp_units::angular::acos(z_offset / L) + mp_units::angular::acos(coxa_femur_cosine) - 90.0 * deg;

    auto femur_tibia_cosine =
        (sq_l - sq_tibia_length_ - sq_femur_length_) / (-2 * femur_length_ * tibia_length_);
    if (mp_units::abs(femur_tibia_cosine) > 1.0 * mp_units::one) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "calcLegInverseKinematics: clamping femur_tibia input "
                                << femur_tibia_cosine.numerical_value_in(mp_units::one) << " for leg "
                                << magic_enum::enum_name(leg_index)
                                << " (target x: " << target_toe_pos.x.numerical_value_in(m)
                                << ", y: " << target_toe_pos.y.numerical_value_in(m)
                                << ", z: " << target_toe_pos.z.numerical_value_in(m) << ")");
        femur_tibia_cosine = std::clamp(femur_tibia_cosine, -1.0 * mp_units::one, 1.0 * mp_units::one);
    }
    const auto femur_tibia = mp_units::angular::acos(femur_tibia_cosine) - 90.0 * deg;

    leg.angles = CLegAngles(torso_coxa_heading - 90.0 * deg, coxa_femur, femur_tibia);

    // for leg local coordinate system we have to add the torso center offset
    leg.angles.torso_coxa += torso_center_offsets_[leg_index].psi;

    if (leg.angles.torso_coxa > 180.0 * deg) {
        leg.angles.torso_coxa -= 360.0 * deg;
    } else if (leg.angles.torso_coxa < -180.0 * deg) {
        leg.angles.torso_coxa += 360.0 * deg;
    }

    leg.toe_position.x += torso_center_offsets_[leg_index].x;
    leg.toe_position.y += torso_center_offsets_[leg_index].y;
}

inline void CBodyModel::calcLegForwardKinematics(const CLegAngles target, CLeg& leg) {
    leg.angles = target;

    const auto coxa_heading = 90.0 * deg + target.torso_coxa;
    const auto femur_elevation = target.coxa_femur;
    const auto tibia_elevation = -90.0 * deg + target.coxa_femur + target.femur_tibia;
    const auto reach = coxa_length_ + femur_length_ * mp_units::angular::cos(femur_elevation) +
                       tibia_length_ * mp_units::angular::cos(tibia_elevation);

    leg.toe_position =
        CPosition(reach * mp_units::angular::sin(coxa_heading), reach * mp_units::angular::cos(coxa_heading),
                  coxa_height_ + femur_length_ * mp_units::angular::sin(femur_elevation) +
                      tibia_length_ * mp_units::angular::sin(tibia_elevation));
}

inline void CBodyModel::setToePosition(const ELegIndex leg_index, const CPosition& target_toe_pos) {
    moveTorso({{leg_index, target_toe_pos}}, torso_pose_);
}

inline void CBodyModel::setLegAngles(const ELegIndex index, const CLegAngles& angles) {
    auto& leg = legs_.at(index);

    // transform to leg coordinate system by subtracting the angle psi
    CLegAngles target_angles = angles;
    target_angles.torso_coxa -= torso_center_offsets_[index].psi;

    calcLegForwardKinematics(target_angles, leg);

    // transform back to robot coordinate system by adding torso center offset and adding psi
    leg.toe_position =
        leg.toe_position +
        rotate(CPosition(torso_center_offsets_[index].x, torso_center_offsets_[index].y, 0.0 * m),
               torso_pose_.orientation) +
        torso_pose_.position;
    leg.angles.torso_coxa += torso_center_offsets_[index].psi;

    logLegPosition(index, leg);
}

inline void CBodyModel::setHeadOrientation(COrientation head) {
    head_orientation_ = head;
    logHeadOrientation();
}

inline void CBodyModel::setHeadOrientation(units::Angle yaw, units::Angle pitch) {
    head_orientation_.yaw = yaw;
    head_orientation_.pitch = pitch;
    logHeadOrientation();
}

inline std::map<ELegIndex, CLeg>& CBodyModel::getLegs() {
    return legs_;
}

inline CLegAngles& CBodyModel::getLegAngles(ELegIndex index) {
    return legs_.at(index).angles;
}

inline std::map<ELegIndex, CLegAngles> CBodyModel::getLegAngles() {
    std::map<ELegIndex, CLegAngles> leg_angles;
    for (auto& [index, leg] : legs_) {
        leg_angles[index] = leg.angles;
    }
    return leg_angles;
}

inline std::map<ELegIndex, CPosition> CBodyModel::getToePositions() const {
    std::map<ELegIndex, CPosition> positions;
    for (auto& [leg_index, leg] : legs_) {
        positions[leg_index] = leg.toe_position;
    }
    return positions;
}

inline std::map<ELegIndex, CPosition> CBodyModel::getStandingToePositions() const {
    std::map<ELegIndex, CPosition> toe_targets;
    for (auto& [leg_index, leg] : standing_legs_) {
        toe_targets[leg_index] = leg.toe_position;
    }
    return toe_targets;
}

inline std::map<ELegIndex, CPosition> CBodyModel::getLaydownToePositions() const {
    std::map<ELegIndex, CPosition> toe_targets;
    for (auto& [leg_index, leg] : laydown_legs_) {
        toe_targets[leg_index] = leg.toe_position;
    }
    return toe_targets;
}

}  // namespace rumblex_geometry
