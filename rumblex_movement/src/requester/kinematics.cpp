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

    auto loadBodyCenterOffsetsFromParameters =
        [&](const std::string& parameter_root, const std::map<ELegIndex, std::string>& leg_parameter_names) {
            std::map<ELegIndex, CBodyCenterOffset> offsets;

            for (const auto& [leg_index, parameter_suffix] : leg_parameter_names) {
                const std::string prefix = parameter_root + "." + parameter_suffix;
                CBodyCenterOffset offset;
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

    auto loadFootPositionsFromParameters = [&](const std::string& parameter_root,
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

    bodyCenterOffsets_ = loadBodyCenterOffsetsFromParameters("leg_offsets", leg_parameter_keys);

    const auto foot_positions_standing =
        loadFootPositionsFromParameters("footPositions_standing", leg_parameter_keys);
    initializeLegs(foot_positions_standing, body_, legsStanding_);

    const auto foot_positions_laydown =
        loadFootPositionsFromParameters("footPositions_laydown", leg_parameter_keys);
    initializeLegs(foot_positions_laydown, body_, legsLayDown_);

    // Initialize current leg positions to laydown (robot starts laying down)
    initializeLegs(foot_positions_laydown, body_, legs_);
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
                           << leg.angles_.coxa.numerical_value_in(deg) << "°, " << std::setw(3)
                           << leg.angles_.femur.numerical_value_in(deg) << "°, " << std::setw(3)
                           << leg.angles_.tibia.numerical_value_in(deg) << "°\t| x: " << std::fixed
                           << std::setprecision(3) << std::setw(3) << leg.foot_pos_.x.numerical_value_in(m)
                           << ", y: " << std::setw(3) << leg.foot_pos_.y.numerical_value_in(m)
                           << ", z: " << std::setw(3) << leg.foot_pos_.z.numerical_value_in(m));
}

void CKinematics::logHeadPosition() {
    if (!LOG_KINEMATICS_HEAD_ACTIVE) return;
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "Head: \tYaw: " << std::fixed << std::setprecision(3) << std::setw(3)
                                       << head_.yaw.numerical_value_in(deg) << "°, Pitch: " << std::setw(3)
                                       << head_.pitch.numerical_value_in(deg) << "°");
}

void CKinematics::initializeLegs(const std::map<ELegIndex, CPosition>& footTargets, const CPose body,
                                 std::map<ELegIndex, CLeg>& legs) {
    for (const auto& [leg_index, foot_target] : footTargets) {
        RCLCPP_DEBUG_STREAM(node_->get_logger(),
                            "Initializing leg "
                                << magic_enum::enum_name(leg_index)
                                << " to foot target position x: " << foot_target.x.numerical_value_in(m)
                                << ", y: " << foot_target.y.numerical_value_in(m)
                                << ", z: " << foot_target.z.numerical_value_in(m));
        auto leg = CLeg();
        CPosition coxa_position(bodyCenterOffsets_.at(leg_index).x, bodyCenterOffsets_.at(leg_index).y,
                                0.0 * m);
        CPosition leg_base = rotate(coxa_position, body.orientation) + body.position;
        CPosition foot_rel = foot_target - leg_base;
        calcLegInverseKinematics(foot_rel, leg, leg_index);
        legs[leg_index] = leg;
    }
    logLegsPositions(legs);
}

void CKinematics::moveBody(const std::map<ELegIndex, CPosition>& foot_targets, const CPose body) {
    body_ = body;

    for (auto& [leg_index, foot_target] : foot_targets) {
        auto& leg = legs_.at(leg_index);
        CPosition coxa_position(bodyCenterOffsets_.at(leg_index).x, bodyCenterOffsets_.at(leg_index).y,
                                0.0 * m);
        CPosition leg_base = rotate(coxa_position, body.orientation) + body.position;
        CPosition foot_rel = foot_target - leg_base;
        calcLegInverseKinematics(foot_rel, leg, leg_index);
    }
    logLegsPositions(legs_);
}

void CKinematics::moveBody(const CPose body) {
    body_ = body;

    auto foot_targets = getLegsStandingPositions();

    for (auto& [leg_index, foot_target] : foot_targets) {
        auto& leg = legs_.at(leg_index);
        CPosition coxa_position(bodyCenterOffsets_.at(leg_index).x, bodyCenterOffsets_.at(leg_index).y,
                                0.0 * m);
        CPosition leg_base = rotate(coxa_position, body.orientation) + body.position;
        CPosition foot_rel = foot_target - leg_base;
        calcLegInverseKinematics(foot_rel, leg, leg_index);
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

void CKinematics::calcLegInverseKinematics(const CPosition& targetFeetPos, CLeg& leg,
                                           const ELegIndex& legIndex) {
    leg.foot_pos_ = targetFeetPos;

    const auto agCoxa = mp_units::angular::atan2(targetFeetPos.x, targetFeetPos.y);
    const auto zOffset = COXA_HEIGHT - targetFeetPos.z;

    const auto lLegTopView = mp_units::hypot(targetFeetPos.x, targetFeetPos.y);  // L1

    const auto horizontal = lLegTopView - COXA_LENGTH;
    const auto sqL = zOffset * zOffset + horizontal * horizontal;
    const auto L = mp_units::sqrt(sqL);

    auto tmpFemur = (sq_tibia_length_ - sq_femur_length_ - sqL) / (-2 * FEMUR_LENGTH * L);
    if (mp_units::abs(tmpFemur) > 1.0 * mp_units::one) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "calcLegInverseKinematics: clamping femur input "
                                << tmpFemur.numerical_value_in(mp_units::one) << " for leg "
                                << magic_enum::enum_name(legIndex)
                                << " (target x: " << targetFeetPos.x.numerical_value_in(m)
                                << ", y: " << targetFeetPos.y.numerical_value_in(m)
                                << ", z: " << targetFeetPos.z.numerical_value_in(m) << ")");
        tmpFemur = std::clamp(tmpFemur, -1.0 * mp_units::one, 1.0 * mp_units::one);
    }
    const auto agFemur =
        mp_units::angular::acos(zOffset / L) + mp_units::angular::acos(tmpFemur) - 90.0 * deg;

    auto tmpTibia = (sqL - sq_tibia_length_ - sq_femur_length_) / (-2 * FEMUR_LENGTH * TIBIA_LENGTH);
    if (mp_units::abs(tmpTibia) > 1.0 * mp_units::one) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "calcLegInverseKinematics: clamping tibia input "
                                << tmpTibia.numerical_value_in(mp_units::one) << " for leg "
                                << magic_enum::enum_name(legIndex)
                                << " (target x: " << targetFeetPos.x.numerical_value_in(m)
                                << ", y: " << targetFeetPos.y.numerical_value_in(m)
                                << ", z: " << targetFeetPos.z.numerical_value_in(m) << ")");
        tmpTibia = std::clamp(tmpTibia, -1.0 * mp_units::one, 1.0 * mp_units::one);
    }
    const auto agTibia = mp_units::angular::acos(tmpTibia) - 90.0 * deg;

    leg.angles_ = CLegAngles(agCoxa - 90.0 * deg, agFemur, agTibia);

    // for leg local coordinate system we have to add the body center offset
    leg.angles_.coxa += bodyCenterOffsets_[legIndex].psi;

    if (leg.angles_.coxa > 180.0 * deg) {
        leg.angles_.coxa -= 360.0 * deg;
    } else if (leg.angles_.coxa < -180.0 * deg) {
        leg.angles_.coxa += 360.0 * deg;
    }

    leg.foot_pos_.x += bodyCenterOffsets_[legIndex].x;
    leg.foot_pos_.y += bodyCenterOffsets_[legIndex].y;
}

void CKinematics::calcLegForwardKinematics(const CLegAngles target, CLeg& leg) {
    leg.angles_ = target;

    const auto coxa = 90.0 * deg + target.coxa;
    const auto femur = target.femur;
    const auto tibia = -90.0 * deg + target.femur + target.tibia;
    const auto reach = COXA_LENGTH + FEMUR_LENGTH * mp_units::angular::cos(femur) +
                       TIBIA_LENGTH * mp_units::angular::cos(tibia);

    leg.foot_pos_ = CPosition(reach * mp_units::angular::sin(coxa), reach * mp_units::angular::cos(coxa),
                              COXA_HEIGHT + FEMUR_LENGTH * mp_units::angular::sin(femur) +
                                  TIBIA_LENGTH * mp_units::angular::sin(tibia));
}

void CKinematics::setSingleFeet(const ELegIndex legIndex, const CPosition& targetFeetPos) {
    moveBody({{legIndex, targetFeetPos}}, body_);
}

void CKinematics::setLegAngles(const ELegIndex index, const CLegAngles& angles) {
    auto& leg = legs_.at(index);

    // transform to leg coordinate system by subtracting the angle psi
    CLegAngles targetAngles = angles;
    targetAngles.coxa -= bodyCenterOffsets_[index].psi;

    calcLegForwardKinematics(targetAngles, leg);

    // transform back to robot coordinate system by adding body center offset and adding psi
    leg.foot_pos_.x += bodyCenterOffsets_[index].x;
    leg.foot_pos_.y += bodyCenterOffsets_[index].y;
    leg.angles_.coxa += bodyCenterOffsets_[index].psi;

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
        positions[legIndex] = leg.foot_pos_;
    }
    return positions;
}

std::map<ELegIndex, CPosition> CKinematics::getLegsStandingPositions() const {
    std::map<ELegIndex, CPosition> footTargets;
    for (auto& [legIndex, leg] : legsStanding_) {
        footTargets[legIndex] = leg.foot_pos_;
    }
    return footTargets;
}

std::map<ELegIndex, CPosition> CKinematics::getLegsLayDownPositions() const {
    std::map<ELegIndex, CPosition> footTargets;
    for (auto& [legIndex, leg] : legsLayDown_) {
        footTargets[legIndex] = leg.foot_pos_;
    }
    return footTargets;
}

}  // namespace rumblex_movement
