/*******************************************************************************
 * Copyright (c) 2024 Christian Stein
 ******************************************************************************/

#include "requester/kinematics.hpp"

#include <mp-units/math.h>

#include <algorithm>
#include <limits>
#include <stdexcept>

#include "sensor_msgs/msg/joint_state.hpp"

using namespace std;

namespace rumblex_movement {

using units::deg;
using units::m;

#define LOG_KINEMATICS_LEG_ACTIVE false
#define LOG_KINEMATICS_HEAD_ACTIVE false

CKinematics::CKinematics(std::shared_ptr<rclcpp::Node> node)
    : node_(node),
      COXA_LENGTH(node->declare_parameter<double>("COXA_LENGTH", rclcpp::PARAMETER_DOUBLE) * m),
      COXA_HEIGHT(node->declare_parameter<double>("COXA_HEIGHT", rclcpp::PARAMETER_DOUBLE) * m),
      FEMUR_LENGTH(node->declare_parameter<double>("FEMUR_LENGTH", rclcpp::PARAMETER_DOUBLE) * m),
      TIBIA_LENGTH(node->declare_parameter<double>("TIBIA_LENGTH", rclcpp::PARAMETER_DOUBLE) * m),
      sq_femur_length_(FEMUR_LENGTH * FEMUR_LENGTH),
      sq_tibia_length_(TIBIA_LENGTH * TIBIA_LENGTH) {
    std::map<ELegIndex, std::string> leg_parameter_keys;
    for (auto leg_index : magic_enum::enum_values<ELegIndex>()) {
        const std::string canonical_name = std::string(magic_enum::enum_name(leg_index));
        std::string parameter_suffix =
            node_->declare_parameter<std::string>("leg_names." + canonical_name, canonical_name);
        leg_parameter_keys[leg_index] = parameter_suffix;
    }

    auto loadTorsoCenterOffsetsFromParameters =
        [&](const std::string& parameter_root, const std::map<ELegIndex, std::string>& leg_parameter_names) {
            std::map<ELegIndex, CTorsoCenterOffset> offsets;

            for (const auto& [leg_index, parameter_suffix] : leg_parameter_names) {
                const std::string prefix = parameter_root + "." + parameter_suffix;
                CTorsoCenterOffset offset;
                offset.x = node_->declare_parameter<double>(prefix + ".CENTER_TO_COXA_X", 0.0) * m;
                offset.y = node_->declare_parameter<double>(prefix + ".CENTER_TO_COXA_Y", 0.0) * m;
                offset.psi = node_->declare_parameter<double>(prefix + ".OFFSET_COXA_ANGLE_DEG", 0.0) * deg;
                offsets[leg_index] = offset;
            }

            RCLCPP_INFO_STREAM(node_->get_logger(), "Loaded " << parameter_root
                                                              << " offsets from parameters ("
                                                              << offsets.size() << " entries).");
            return offsets;
        };

    auto loadToePositionsFromParameters = [&](const std::string& parameter_root,
                                              const std::map<ELegIndex, std::string>& leg_parameter_names) {
        std::map<ELegIndex, CPosition> positions;
        for (const auto& [leg_index, parameter_suffix] : leg_parameter_names) {
            const std::string prefix = parameter_root + "." + parameter_suffix;
            positions[leg_index] = CPosition(node_->declare_parameter<double>(prefix + ".x", 0.0),
                                             node_->declare_parameter<double>(prefix + ".y", 0.0),
                                             node_->declare_parameter<double>(prefix + ".z", 0.0));
        }
        return positions;
    };

    torsoCenterOffsets_ = loadTorsoCenterOffsetsFromParameters("leg_offsets", leg_parameter_keys);

    const auto toe_positions_standing =
        loadToePositionsFromParameters("footPositions_standing", leg_parameter_keys);
    initializeLegs(toe_positions_standing, torso_, legsStanding_);

    const auto toe_positions_laydown =
        loadToePositionsFromParameters("footPositions_laydown", leg_parameter_keys);
    initializeLegs(toe_positions_laydown, torso_, legsLayDown_);

    // Initialize current leg positions to laydown (robot starts laying down)
    initializeLegs(toe_positions_laydown, torso_, legs_);
}

void CKinematics::logLegsPositions(std::map<ELegIndex, CLeg>& legs) {
    if (!LOG_KINEMATICS_LEG_ACTIVE) return;
    RCLCPP_INFO_STREAM(node_->get_logger(), "----------------------------------------");
    for (const auto& [index, leg] : legs) {
        logLegPosition(index, leg);
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "----------------------------------------");
}

void CKinematics::logLegPosition(const ELegIndex index, const CLeg& leg) {
    if (!LOG_KINEMATICS_LEG_ACTIVE) return;
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       magic_enum::enum_name(index)
                           << ": \tag: " << std::fixed << std::setprecision(3) << std::setw(3)
                           << leg.angles_.torso_coxa.numerical_value_in(deg) << "°, " << std::setw(3)
                           << leg.angles_.coxa_femur.numerical_value_in(deg) << "°, " << std::setw(3)
                           << leg.angles_.femur_tibia.numerical_value_in(deg) << "°\t| x: " << std::fixed
                           << std::setprecision(3) << std::setw(3) << leg.toe_pos_.x.numerical_value_in(m)
                           << ", y: " << std::setw(3) << leg.toe_pos_.y.numerical_value_in(m)
                           << ", z: " << std::setw(3) << leg.toe_pos_.z.numerical_value_in(m));
}

void CKinematics::logHeadPosition() {
    if (!LOG_KINEMATICS_HEAD_ACTIVE) return;
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "Head: \tYaw: " << std::fixed << std::setprecision(3) << std::setw(3)
                                       << head_.yaw.numerical_value_in(deg) << "°, Pitch: " << std::setw(3)
                                       << head_.pitch.numerical_value_in(deg) << "°");
}

void CKinematics::initializeLegs(const std::map<ELegIndex, CPosition>& toeTargets, const CPose torso,
                                 std::map<ELegIndex, CLeg>& legs) {
    for (const auto& [leg_index, toe_target] : toeTargets) {
        RCLCPP_DEBUG_STREAM(node_->get_logger(),
                            "Initializing leg "
                                << magic_enum::enum_name(leg_index)
                                << " to toe target position x: " << toe_target.x.numerical_value_in(m)
                                << ", y: " << toe_target.y.numerical_value_in(m)
                                << ", z: " << toe_target.z.numerical_value_in(m));
        auto leg = CLeg();
        CPosition coxa_position(torsoCenterOffsets_.at(leg_index).x, torsoCenterOffsets_.at(leg_index).y,
                                0.0 * m);
        CPosition leg_base = rotate(coxa_position, torso.orientation) + torso.position;
        CPosition toe_rel = toe_target - leg_base;
        calcLegInverseKinematics(toe_rel, leg, leg_index);
        legs[leg_index] = leg;
    }
    logLegsPositions(legs);
}

void CKinematics::moveTorso(const std::map<ELegIndex, CPosition>& toe_targets, const CPose torso) {
    torso_ = torso;

    for (auto& [leg_index, toe_target] : toe_targets) {
        auto& leg = legs_.at(leg_index);
        CPosition coxa_position(torsoCenterOffsets_.at(leg_index).x, torsoCenterOffsets_.at(leg_index).y,
                                0.0 * m);
        CPosition leg_base = rotate(coxa_position, torso.orientation) + torso.position;
        CPosition toe_rel = toe_target - leg_base;
        calcLegInverseKinematics(toe_rel, leg, leg_index);
    }
    logLegsPositions(legs_);
}

void CKinematics::moveTorso(const CPose torso) {
    torso_ = torso;

    auto toe_targets = getLegsStandingPositions();

    for (auto& [leg_index, toe_target] : toe_targets) {
        auto& leg = legs_.at(leg_index);
        CPosition coxa_position(torsoCenterOffsets_.at(leg_index).x, torsoCenterOffsets_.at(leg_index).y,
                                0.0 * m);
        CPosition leg_base = rotate(coxa_position, torso.orientation) + torso.position;
        CPosition toe_rel = toe_target - leg_base;
        calcLegInverseKinematics(toe_rel, leg, leg_index);
    }
    logLegsPositions(legs_);
}

CPosition CKinematics::rotate(const CPosition& point, const COrientation& orientation) {
    const auto px = point.x;
    const auto py = point.y;
    const auto pz = point.z;

    const auto cosRoll = mp_units::angular::cos(orientation.roll);
    const auto sinRoll = mp_units::angular::sin(orientation.roll);
    const auto cosPitch = mp_units::angular::cos(orientation.pitch);
    const auto sinPitch = mp_units::angular::sin(orientation.pitch);
    const auto cosYaw = mp_units::angular::cos(orientation.yaw);
    const auto sinYaw = mp_units::angular::sin(orientation.yaw);

    // Standard ZYX (yaw-pitch-roll) rotation matrix
    const auto rotatedX = cosYaw * cosPitch * px + (cosYaw * sinPitch * sinRoll - sinYaw * cosRoll) * py +
                          (cosYaw * sinPitch * cosRoll + sinYaw * sinRoll) * pz;

    const auto rotatedY = sinYaw * cosPitch * px + (sinYaw * sinPitch * sinRoll + cosYaw * cosRoll) * py +
                          (sinYaw * sinPitch * cosRoll - cosYaw * sinRoll) * pz;

    const auto rotatedZ = -sinPitch * px + cosPitch * sinRoll * py + cosPitch * cosRoll * pz;

    return {rotatedX, rotatedY, rotatedZ};
}

void CKinematics::calcLegInverseKinematics(const CPosition& targetToePos, CLeg& leg,
                                           const ELegIndex& legIndex) {
    leg.toe_pos_ = targetToePos;

    const auto torso_coxa_heading = mp_units::angular::atan2(targetToePos.x, targetToePos.y);
    const auto zOffset = COXA_HEIGHT - targetToePos.z;

    const auto lLegTopView = mp_units::hypot(targetToePos.x, targetToePos.y);  // L1

    const auto horizontal = lLegTopView - COXA_LENGTH;
    const auto sqL = zOffset * zOffset + horizontal * horizontal;
    const auto L = mp_units::sqrt(sqL);

    auto coxa_femur_cosine = (sq_tibia_length_ - sq_femur_length_ - sqL) / (-2 * FEMUR_LENGTH * L);
    if (mp_units::abs(coxa_femur_cosine) > 1.0 * mp_units::one) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "calcLegInverseKinematics: clamping coxa_femur input "
                                                     << coxa_femur_cosine.numerical_value_in(mp_units::one)
                                                     << " for leg " << magic_enum::enum_name(legIndex)
                                                     << " (target x: " << targetToePos.x.numerical_value_in(m)
                                                     << ", y: " << targetToePos.y.numerical_value_in(m)
                                                     << ", z: " << targetToePos.z.numerical_value_in(m)
                                                     << ")");
        coxa_femur_cosine = std::clamp(coxa_femur_cosine, -1.0 * mp_units::one, 1.0 * mp_units::one);
    }
    const auto coxa_femur =
        mp_units::angular::acos(zOffset / L) + mp_units::angular::acos(coxa_femur_cosine) - 90.0 * deg;

    auto femur_tibia_cosine =
        (sqL - sq_tibia_length_ - sq_femur_length_) / (-2 * FEMUR_LENGTH * TIBIA_LENGTH);
    if (mp_units::abs(femur_tibia_cosine) > 1.0 * mp_units::one) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "calcLegInverseKinematics: clamping femur_tibia input "
                                                     << femur_tibia_cosine.numerical_value_in(mp_units::one)
                                                     << " for leg " << magic_enum::enum_name(legIndex)
                                                     << " (target x: " << targetToePos.x.numerical_value_in(m)
                                                     << ", y: " << targetToePos.y.numerical_value_in(m)
                                                     << ", z: " << targetToePos.z.numerical_value_in(m)
                                                     << ")");
        femur_tibia_cosine = std::clamp(femur_tibia_cosine, -1.0 * mp_units::one, 1.0 * mp_units::one);
    }
    const auto femur_tibia = mp_units::angular::acos(femur_tibia_cosine) - 90.0 * deg;

    leg.angles_ = CLegAngles(torso_coxa_heading - 90.0 * deg, coxa_femur, femur_tibia);

    // for leg local coordinate system we have to add the torso center offset
    leg.angles_.torso_coxa += torsoCenterOffsets_[legIndex].psi;

    if (leg.angles_.torso_coxa > 180.0 * deg) {
        leg.angles_.torso_coxa -= 360.0 * deg;
    } else if (leg.angles_.torso_coxa < -180.0 * deg) {
        leg.angles_.torso_coxa += 360.0 * deg;
    }

    leg.toe_pos_.x += torsoCenterOffsets_[legIndex].x;
    leg.toe_pos_.y += torsoCenterOffsets_[legIndex].y;
}

void CKinematics::calcLegForwardKinematics(const CLegAngles target, CLeg& leg) {
    leg.angles_ = target;

    const auto coxa_heading = 90.0 * deg + target.torso_coxa;
    const auto femur_elevation = target.coxa_femur;
    const auto tibia_elevation = -90.0 * deg + target.coxa_femur + target.femur_tibia;
    const auto reach = COXA_LENGTH + FEMUR_LENGTH * mp_units::angular::cos(femur_elevation) +
                       TIBIA_LENGTH * mp_units::angular::cos(tibia_elevation);

    leg.toe_pos_ =
        CPosition(reach * mp_units::angular::sin(coxa_heading), reach * mp_units::angular::cos(coxa_heading),
                  COXA_HEIGHT + FEMUR_LENGTH * mp_units::angular::sin(femur_elevation) +
                      TIBIA_LENGTH * mp_units::angular::sin(tibia_elevation));
}

void CKinematics::setSingleToe(const ELegIndex legIndex, const CPosition& targetToePos) {
    moveTorso({{legIndex, targetToePos}}, torso_);
}

void CKinematics::setLegAngles(const ELegIndex index, const CLegAngles& angles) {
    auto& leg = legs_.at(index);

    // transform to leg coordinate system by subtracting the angle psi
    CLegAngles targetAngles = angles;
    targetAngles.torso_coxa -= torsoCenterOffsets_[index].psi;

    calcLegForwardKinematics(targetAngles, leg);

    // transform back to robot coordinate system by adding torso center offset and adding psi
    leg.toe_pos_.x += torsoCenterOffsets_[index].x;
    leg.toe_pos_.y += torsoCenterOffsets_[index].y;
    leg.angles_.torso_coxa += torsoCenterOffsets_[index].psi;

    logLegPosition(index, leg);
}

void CKinematics::setHead(COrientation head) {
    head_ = head;
    logHeadPosition();
}

void CKinematics::setHead(units::Angle yaw, units::Angle pitch) {
    head_.yaw = yaw;
    head_.pitch = pitch;
    logHeadPosition();
}

std::map<ELegIndex, CLeg>& CKinematics::getLegs() {
    return legs_;
}

CLegAngles& CKinematics::getAngles(ELegIndex index) {
    return legs_.at(index).angles_;
}

std::map<ELegIndex, CLegAngles> CKinematics::getLegsAngles() {
    std::map<ELegIndex, CLegAngles> legAngles;
    for (auto& [index, leg] : legs_) {
        legAngles[index] = leg.angles_;
    }
    return legAngles;
}

std::map<ELegIndex, CPosition> CKinematics::getLegsPositions() const {
    std::map<ELegIndex, CPosition> positions;
    for (auto& [legIndex, leg] : legs_) {
        positions[legIndex] = leg.toe_pos_;
    }
    return positions;
}

std::map<ELegIndex, CPosition> CKinematics::getLegsStandingPositions() const {
    std::map<ELegIndex, CPosition> toeTargets;
    for (auto& [legIndex, leg] : legsStanding_) {
        toeTargets[legIndex] = leg.toe_pos_;
    }
    return toeTargets;
}

std::map<ELegIndex, CPosition> CKinematics::getLegsLayDownPositions() const {
    std::map<ELegIndex, CPosition> toeTargets;
    for (auto& [legIndex, leg] : legsLayDown_) {
        toeTargets[legIndex] = leg.toe_pos_;
    }
    return toeTargets;
}

}  // namespace rumblex_movement
